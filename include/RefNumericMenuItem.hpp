#ifndef REFNUMERICMENUITEM_HPP
#define REFNUMERICMENUITEM_HPP

#include <MenuSystem.h>


// A sort of hackish class to store a reference to the supplied variable
// and automatically update the value when rendering.
// Assumes that the variable supplied as the value lives for the entire
// lifespan of the instance of this class, otherwise we are in UB land!
template<typename T>
class RefNumericMenuItem : public NumericMenuItem {
public:
    RefNumericMenuItem(const char* name, SelectFnPtr select_fn,
                       T& value, float min_value, float max_value,
                       float increment=1.0, FormatValueFnPtr format_value_fn = nullptr)
        : NumericMenuItem(name, select_fn, value, min_value, max_value, increment, format_value_fn),
          mValueRef(value)
        {}

    virtual void render(const MenuComponentRenderer& renderer) const override;

protected:
    virtual bool next(bool loop = false) override;
    virtual bool prev(bool loop = false) override;

private:
    T& mValueRef;
};

template<typename T>
void RefNumericMenuItem<T>::render(const MenuComponentRenderer& renderer) const
{
    // Well, this is *really* ugly, but we've gotten this far so might as well...
    const_cast<RefNumericMenuItem*>(this)->set_value(mValueRef);
    NumericMenuItem::render(renderer);
}

template<typename T>
bool RefNumericMenuItem<T>::next(bool loop)
{
    bool ret = NumericMenuItem::next(loop);
    mValueRef = _value;
    return ret;
}

template<typename T>
bool RefNumericMenuItem<T>::prev(bool loop)
{
    bool ret = NumericMenuItem::prev(loop);
    mValueRef = _value;
    return ret;
}

#endif // REFNUMERICMENUITEM_HPP
