#ifndef __COMETS_OVERLOADED__
#define __COMETS_OVERLOADED__

namespace comets
{

    // helper type for the visitor #4
    template <class... Ts>
    struct overloaded : Ts...
    {
        using Ts::operator()...;
    };
    // explicit deduction guide (not needed as of C++20)
    template <class... Ts>
    overloaded(Ts...) -> overloaded<Ts...>;
} // namespace comets

#endif
