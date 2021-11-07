#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MenuSystem.h>
#include <max6675.h>
#include <PID_v1.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include "RefNumericMenuItem.hpp"


// Temperature reading
void fillTempWindow();
double readTemp();

// Serial interface
void readSerialCommands(Stream& interface);
void printHelpSerial(Stream& interface);
void logTemp(float currentTemp);
Stream* loggingInterface = nullptr;
Stream* debuggingInterface = nullptr;

// ISR for rotary encoder
enum class EncoderRotation {
    NONE, LEFT, RIGHT
};
void updateEncoder();
EncoderRotation encoderRotated();
bool buttonPressed();
void handleEncoderInput(EncoderRotation rotation);

// User interface
void displayHome(unsigned long brewTimer);
void onSelected(MenuComponent* item);
void onSelectedNumeric(MenuComponent* component);

// Settings
void saveConfig();
void loadConfig();

#define CONFIG_VERSION "sp2"
#define CONFIG_START 32

struct StoreStruct {
    int brewSetpoint, steamSetpoint;
    double p, i, d;
    bool ponm;
    char version[4];
};
StoreStruct settings = {
    100, 140,
    1, 0, 0,
    false,
    CONFIG_VERSION
};


SSD1306AsciiAvrI2c oled;
bool inMenu = false;

class OledRenderer : public MenuComponentRenderer {
public:
    void render(const Menu& menu) const override {
        if (!inMenu) {
            return;
        }

        oled.clear();

        for (int i = 0; i < menu.get_num_components(); ++i) {
            const MenuComponent* component = menu.get_menu_component(i);
            component->render(*this);

            if (strcmp(component->get_name(), "PonM") == 0) {
                oled.print(settings.ponm ? F(": On") : F(": Off"));
            }

            if (component == menu.get_current_component()) {
                oled.print(F(" <<<"));
            }
            oled.println("");
        }
    }

    void render_menu_item(const MenuItem& menu_item) const override {
        oled.print(menu_item.get_name());
    }

    void render_back_menu_item(const BackMenuItem& menu_item) const override {
        oled.print(menu_item.get_name());
    }

    void render_numeric_menu_item(const NumericMenuItem& menu_item) const override {
        oled.print(menu_item.get_name());
        oled.print(F(": "));
        oled.print(menu_item.get_formatted_value());
    }

    void render_menu(const Menu& menu) const override {
        oled.print(menu.get_name());
    }
};

// Custom characters for pretty display
enum CustomChar : char {
    DEGREE = 0,
    CUP,
    STEAM,
};
// These should be const but the API won't allow it:
uint8_t DEGREE_CHAR[8] = {0x8, 0xf4, 0x8, 0x43, 0x4, 0x4, 0x43, 0x0};
uint8_t CUP_CHAR[8] = {0x0, 0x4, 0x8, 0x1f, 0xf2, 0xc, 0x1e, 0x0};
uint8_t STEAM_CHAR[8] = {0x0, 0xa, 0xa, 0xa, 0x0, 0x15, 0xa, 0x0};

// Menu system
OledRenderer renderer;
MenuSystem ms(renderer);
Menu setpointMenu("Setpoints");
RefNumericMenuItem<int> brewItem("Brew", &onSelectedNumeric, settings.brewSetpoint, 60, 140);
RefNumericMenuItem<int> steamItem("Steam", &onSelectedNumeric, settings.steamSetpoint, 80, 160);
Menu pidMenu("PID Settings");
RefNumericMenuItem<double> pItem("Kp", &onSelectedNumeric, settings.p, 0, 100, 0.1);
RefNumericMenuItem<double> iItem("Ki", &onSelectedNumeric, settings.i, 0, 100, 0.1);
RefNumericMenuItem<double> dItem("Kd", &onSelectedNumeric, settings.d, 0, 100, 0.1);
MenuItem togglePonmItem("PonM", &onSelected);
BackMenuItem backItem("Back", &onSelected, &ms);

// HC05 bluetooth
SoftwareSerial bluetooth(11, 12);

// Rotary encoder
// Top view of rotary encoder:
// Rotary A   -|   |-   Rotary Button
// Ground     -| O |
// Rotary B   -|___|-   Ground
const int ROTARY_A = 2;
const int ROTARY_B = 3;
const int ROTARY_BUTTON = 8;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

// Thermocouple
const int THERMO_DO = 4;   // num. 3
const int THERMO_CS = 5;   // num. 2
const int THERMO_CLK = 6;  // num. 1
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Temperature data
const int MAX_NUM_READINGS = 5;
double temps[MAX_NUM_READINGS] = {};
int tempIndex = 0;
double totalTemp = 0.0;
double averageTemp = 0.0;
unsigned long lastTempRead = 0;

// SSR
const int SSR = 7;

// Switches
const int STEAM_SWITCH = 9;
const int BREW_SWITCH = 10;

// PID
double setpoint;
double input, output;
PID pid(&input, &output, &setpoint, settings.p, settings.i, settings.d, DIRECT, settings.ponm ? P_ON_M : P_ON_E);
const int WINDOW_SIZE = 1000;
unsigned long windowStartTime;

void setup() {
    Serial.begin(115200);
    Serial.println(F("Silvia PID starting"));
    bluetooth.begin(115200);
    bluetooth.println(F("Silvia PID starting"));
    loadConfig();

    oled.begin(&Adafruit128x64, 0x3C);
    oled.setI2cClock(444 * 1e6);
    oled.setFont(System5x7);

    oled.clear();
    oled.set2X();
    oled.println(F("Silvia PID"));
    oled.println(F("starting"));
    oled.set1X();

    // fillTempWindow(); // TODO: remove
    input = averageTemp;

    windowStartTime = millis();
    setpoint = settings.brewSetpoint;

    pid.SetTunings(settings.p, settings.i, settings.d, settings.ponm ? P_ON_M : P_ON_E);
    pid.SetOutputLimits(0, WINDOW_SIZE);
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(250); // set pid to compute new output every 250 ms to allow temp readings to happen

    pinMode(SSR, OUTPUT);
    pinMode(STEAM_SWITCH, INPUT_PULLUP);
    pinMode(BREW_SWITCH, INPUT_PULLUP);

    pinMode(ROTARY_A, INPUT_PULLUP);
    pinMode(ROTARY_B, INPUT_PULLUP);
    pinMode(ROTARY_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_B), updateEncoder, CHANGE);

    ms.get_root_menu().add_menu(&setpointMenu);
    setpointMenu.add_item(&brewItem);
    setpointMenu.add_item(&steamItem);
    setpointMenu.add_item(&backItem);
    ms.get_root_menu().add_menu(&pidMenu);
    pidMenu.add_item(&pItem);
    pidMenu.add_item(&iItem);
    pidMenu.add_item(&dItem);
    pidMenu.add_item(&togglePonmItem);
    pidMenu.add_item(&backItem);
    ms.get_root_menu().add_item(&backItem);

    delay(500);
    oled.clear();
    Serial.println(F("Ready"));
    bluetooth.println(F("Ready"));
}

void loop() {
    // Provide serial interface
    readSerialCommands(Serial);
    readSerialCommands(bluetooth);

    // Brew timer
    static unsigned long timerStart = 0;
    static unsigned long timerSaved = 0;
    static bool timerGoing = false;
    if (timerStart != 0) {
        if (timerGoing) {
            timerSaved = millis()/1000 - timerStart/1000;
        }
    }

    // Check if brew switch is pressed
    if (digitalRead(BREW_SWITCH) && !timerGoing) {
        // Start timer
        timerGoing = true;
        timerStart = millis();
    } else if (!digitalRead(BREW_SWITCH)) {
        timerGoing = false;
        timerStart = 0;
    }

    // Check brew/steam mode and set setpoint accordingly
    if (digitalRead(STEAM_SWITCH)) {
        setpoint = settings.steamSetpoint;
    } else {
        setpoint = settings.brewSetpoint;
    }

    // Handle rotary encoder rotation
    EncoderRotation rotation = encoderRotated();
    handleEncoderInput(rotation);

    // Show homescreen
    displayHome(timerSaved);

    // Read temperature data
    input = readTemp();
    logTemp(input);

    // Compute new PID output
    bool pidComputed = pid.Compute();

    // Control SSR using time proporional control
    unsigned long now = millis();
    if (now - windowStartTime > WINDOW_SIZE) {
        windowStartTime += WINDOW_SIZE;
    }
    if (now - windowStartTime < output) {
        digitalWrite(SSR, LOW); // TODO: HIGH
    } else {
        digitalWrite(SSR, LOW);
    }

    if (pidComputed && debuggingInterface) {
        char buffer[100];
        snprintf(buffer, 100, "PID computed new value: input=%ld output=%ld", round(input), round(output));
        debuggingInterface->println(buffer);
    }
}

void fillTempWindow() {
    while (temps[MAX_NUM_READINGS - 1] == 0) {
        readTemp();
        delay(220);
    }
}

double readTemp() {
    // Check if enough time has passed to do a new reading from chip
    if (millis()-lastTempRead <= 220) {
        return averageTemp;
    }

    double temp = thermocouple.readCelsius();
    lastTempRead = millis();

    if (isnan(temp)) {
        if (debuggingInterface) {
            debuggingInterface->println(F("Reading theromocouple failed, got NAN"));
        }
        return averageTemp;
    }

    totalTemp = (totalTemp + temp) - temps[tempIndex];
    temps[tempIndex] = temp;
    tempIndex++;
    if (tempIndex >= MAX_NUM_READINGS) {
        tempIndex = 0;
    }

    averageTemp = totalTemp / MAX_NUM_READINGS;
    return averageTemp;
}

void onSelectedNumeric(MenuComponent* component) {
    pid.SetTunings(settings.p, settings.i, settings.d, settings.ponm ? P_ON_M : P_ON_E);
    saveConfig();
    ms.display();
}

void onSelected(MenuComponent* component) {
    if (component->get_name() == backItem.get_name() && ms.get_current_menu() == &ms.get_root_menu()) {
        // In root menu, exit menu
        inMenu = false;
        ms.reset();
        oled.clear();
    } else if (component->get_name() == togglePonmItem.get_name()) {
        settings.ponm = !settings.ponm;
        pid.SetTunings(settings.p, settings.i, settings.d, settings.ponm ? P_ON_M : P_ON_E);
        saveConfig();
    }
}

void handleEncoderInput(EncoderRotation rotation) {
    if (rotation == EncoderRotation::RIGHT) {
        // Rotated right
        if (inMenu) {
            ms.next();
            ms.display();
        }
        else {
            // TODO: not needed?
            // oled.clear();
        }
    } else if (rotation == EncoderRotation::LEFT) {
        // Rotated left
        if (inMenu) {
            ms.prev();
            ms.display();
        }
    }

    // Handle rotary button
    if (buttonPressed()) {
        // Button pressed and debounced
        if (!inMenu) {
            inMenu = true;
            ms.display();
        } else {
            ms.select();
            ms.display();
        }
    }
}

void displayHome(unsigned long brewTimer) {
    static const unsigned long REFRESH_INTERVAL = 200;
    static bool lastInMenu = false;
    if (inMenu && !lastInMenu) {
        // Just switched to menu, display it immediately
        lastInMenu = true;
        ms.display();
        return;
    }
    else if (inMenu) {
        // We are in settings menu so shouldn't draw the home screen
        return;
    }

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < REFRESH_INTERVAL && !lastInMenu) {
        // Not time to update screen yet
        return;
    }

    if (lastInMenu) {
        oled.clear();
    }
    oled.setCursor(0, 0);
    oled.set2X();

    // int r = (rand() % 120) + 10; // temp
    // input = r;                   // temp

    // First row, example: "  97C/102C "
    const size_t BUFFER_SIZE = 12;
    char buffer[BUFFER_SIZE];
    snprintf(buffer, BUFFER_SIZE, " %3dC/%3dC ", static_cast<int>(input + 0.5), static_cast<int>(setpoint + 0.5));
    oled.println(buffer);

    // Second row, example: "I       BT", I = indicator, BT = brew timer
    // if (digitalRead(STEAM_SWITCH)) {
    //     oled.print(CustomChar::STEAM);
    // }
    // else {
    //     oled.print(CustomChar::CUP);
    // }
    // oled.print(' ');
    // snprintf(buffer, BUFFER_SIZE, "      %3lu  ", brewTimer);
    // snprintf(buffer, BUFFER_SIZE, "%c   %2lu  ", digitalRead(STEAM_SWITCH) ? CustomChar::STEAM : CustomChar::CUP, brewTimer);
    // oled.println(buffer);
    oled.println();

    // Third row, example: "Timer: 28"
    snprintf(buffer, BUFFER_SIZE, "       %3lu ", brewTimer);
    oled.println(buffer);

    if (digitalRead(STEAM_SWITCH)) {
        oled.println(F("Steaming"));
    }
    else {
        oled.println(F("        "));
    }

    oled.set1X();
    lastInMenu = false;
    lastUpdate = millis();
}

// ISR for rotary encoder
// This function is written by Adam Meyer: http://adam-meyer.com/arduino/Rotary_Encoder
// Licensed under the MIT license: https://opensource.org/licenses/mit-license.php
void updateEncoder() {
    int MSB = digitalRead(ROTARY_A); //MSB = most significant bit
    int LSB = digitalRead(ROTARY_B); //LSB = least significant bit

    int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

    lastEncoded = encoded; //store this value for next time
}

EncoderRotation encoderRotated() {
    static long lastEncoderValue = 0;
    if (encoderValue/4 > lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return EncoderRotation::LEFT;
    } else if (encoderValue/4 < lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return EncoderRotation::RIGHT;
    }
    return EncoderRotation::NONE;
}

bool buttonPressed() {
    static bool pressable = true;
    static unsigned long lastPress;

    if (!digitalRead(ROTARY_BUTTON) && pressable && millis() - lastPress > 200) {
        pressable = false;
        lastPress = millis();
        return true;
    } else if (digitalRead(ROTARY_BUTTON)) {
        pressable = true;
        return false;
    }

    return false;
}

void readSerialCommands(Stream& interface) {
    while (interface.available()) {
        String command = interface.readStringUntil('\n');
        command.trim();
        interface.println(command);
        Serial.println(command);

        if (command.startsWith(F("set"))) {
            // set - set a value for a setting
            int settingPos = command.indexOf(' ') + 1;
            int valuePos = command.indexOf(' ', settingPos) + 1;

            String setting = command.substring(settingPos, valuePos-1);
            String value = command.substring(valuePos);

            if (setting == F("p")) {
                settings.p = value.toFloat();
            } else if (setting == F("i")) {
                settings.i = value.toFloat();
            } else if (setting == F("d")) {
                settings.d = value.toFloat();
            } else if (setting == F("brewSetpoint") || setting == F("brewtemp")) {
                settings.brewSetpoint = value.toFloat();
            } else if (setting == F("steamSetpoint") || setting == F("steamtemp")) {
                settings.steamSetpoint = value.toFloat();
            } else if (setting == F("ponm")) {
                settings.ponm = (value == F("on") || value == F("true"));
            } else {
                interface.println("Unknown setting: " + setting);
                continue;
            }

            pid.SetTunings(settings.p, settings.i, settings.d, settings.ponm ? P_ON_M : P_ON_E);
            saveConfig();
            interface.println(F("Success"));

        } else if (command.startsWith(F("get"))) {
            // get - get the value of a setting
            int settingPos = command.indexOf(' ') + 1;
            String setting = command.substring(settingPos);

            if (setting == F("p")) {
                interface.println(settings.p);
            } else if (setting == F("i")) {
                interface.println(settings.i);
            } else if (setting == F("d")) {
                interface.println(settings.d);
            } else if (setting == F("brewSetpoint") || setting == F("brewtemp")) {
                interface.println(settings.brewSetpoint);
            } else if (setting == F("steamSetpoint") || setting == F("steamtemp")) {
                interface.println(settings.steamSetpoint);
            } else if (setting == F("ponm")) {
                interface.println(settings.ponm ? F("on") : F("off"));
            } else {
                interface.println("Unknown setting: " + setting);
                continue;
            }

        } else if (command.startsWith(F("log"))) {
            int settingPos = command.indexOf(' ');

            bool enable = false;
            if (settingPos == -1) {
                // only entered "log", just toggle logging on/off
                enable = (loggingInterface == nullptr);
            } else if (command.substring(settingPos+1) == F("on")) {
                enable = true;
            }

            loggingInterface = (enable ? &interface : nullptr);
            interface.println("Logging " + String(enable ? "enabled" : "disabled"));
        
        } else if (command.startsWith(F("debug"))) {
            int settingPos = command.indexOf(' ');

            bool enable = false;
            if (settingPos == -1) {
                // only entered "debug", just toggle debugging on/off
                enable = (debuggingInterface == nullptr);
            } else if (command.substring(settingPos+1) == F("on")) {
                enable = true;
            }

            debuggingInterface = (enable ? &interface : nullptr);
            interface.println("Debugging " + String(enable ? "enabled" : "disabled"));

        } else if (command.startsWith("c")) {
            char c = command[1];
            if (c == 'w') {
                ms.prev();
                ms.display();
            }
            else if (c == 's') {
                ms.next();
                ms.display();
            }
            else if (c == 'd') {
                if (!inMenu) {
                    inMenu = true;
                    ms.display();
                }
                else {
                    ms.select();
                    ms.display();
                }
            }
            else if (c == 'a') {
                ms.back();
                ms.display();
            }
            else if (c == 'b') {
                digitalWrite(11, !digitalRead(11)); // brew switch
            }
            else if (c == 'n') {
                digitalWrite(12, !digitalRead(12)); // steam switch
            }

        } else if (command.startsWith(F("help"))) {
            printHelpSerial(interface);

        } else {
            interface.println("Unknown command: " + command);
            printHelpSerial(interface);
        }
    }
}

void printHelpSerial(Stream& interface) {
    interface.println(F("Silvia PID commands:"));
    interface.println(F("  set SETTING VALUE - set the value of SETTING to VALUE"));
    interface.println(F("    Available settings:"));
    interface.println(F("      p:          float"));
    interface.println(F("      i:          float"));
    interface.println(F("      d:          float"));
    interface.println(F("      brewtemp:   float"));
    interface.println(F("      steamtemp:  float"));
    interface.println(F("      ponm:       on/off  (proportional on measurement)"));
    interface.println(F("  get SETTING - get the current value of SETTING"));
    interface.println(F("  log [on/off] - toggle logging of temperature values every second on the form: [ms since boot]: [temp]"));
    interface.println(F("  debug [on/off] - toggle debug printouts on the serial port (default off)"));
    interface.println(F("  help - show this help"));
}

void logTemp(float currentTemp) {
    static unsigned long lastLogTime = millis();

    if (loggingInterface) {
        unsigned long currentTime = millis();

        if (currentTime - lastLogTime > 1000) {
            // Log temperature value every second
            loggingInterface->println(String(currentTime) + ": " + String(currentTemp, 4));
            lastLogTime = currentTime;
        }
    }
}

void saveConfig() {
    EEPROM.put(CONFIG_START, settings);
}

void loadConfig() {
    // First check if we have valid settings stored
    StoreStruct tmp;
    EEPROM.get(CONFIG_START, tmp);

    if (strcmp(tmp.version, CONFIG_VERSION) != 0) {
        // Settings invalid, overwrite with default
        saveConfig();
    }

    // Now we can actually load the settings
    EEPROM.get(CONFIG_START, settings);
}
