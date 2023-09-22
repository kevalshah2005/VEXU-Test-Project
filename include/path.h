#ifndef __COMETS_PATH__
#define __COMETS_PATH__

#include "okapi/api.hpp"
#include <array>
#include <variant>

namespace comets
{

    struct path_target
    {
        std::initializer_list<okapi::PathfinderPoint> points;
        std::string_view name;
    };

    struct path_tag
    {
        std::string_view target;

        operator std::string() noexcept
        {
            return std::string(target);
        }
    };
    struct turn_tag
    {
        okapi::QAngle angle;
        operator okapi::QAngle() noexcept
        {
            return angle;
        }
    };

    using instruction = std::variant<path_tag, turn_tag>;

    template <size_t N>
    class instruction_list : public std::array<instruction, N>
    {
    };

    template <class... T>
    instruction_list(T &&...t) -> instruction_list<sizeof...(T)>;

} // namespace comets

#endif
