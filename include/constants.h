#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include "comets/vendor.h"
#include "comets/paths.h"
#include "comets/types.h"

namespace constants
{
    using namespace okapi;
    inline constexpr auto CHASSIS_GEARSET = okapi::AbstractMotor::gearset::green;
    inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
    inline constexpr auto CHASSIS_TPR = okapi::imev5GreenTPR;

    inline constexpr okapi::PathfinderLimits PATH_LIMITS = {
        1.0 * 0.66, // Maximum linear velocity of the Chassis in m/s
        2.0 * 0.66, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 * 0.66 // Maximum linear jerk of the Chassis in m/s/s/s
    };

    inline constexpr comets::path_plan PATHS[] = {
        {.name = "right_turn", .points = {{0_ft, 0_ft, 0_deg}, {3_ft, 3_ft, 90_deg}}},
        {.name = "straight", .points = {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}},
        {.name = "strafe_right", .points = {{0_ft, 0_ft, 0_deg}, {0_ft, 2_ft, 0_deg}}},
    };

    inline constexpr bool LEFT_REVERSED = false;
    inline constexpr bool RIGHT_REVERSED = true;

    inline constexpr int8_t FL_PORT = 20;
    inline constexpr int8_t FR_PORT = 11;
    inline constexpr int8_t BL_PORT = 10;
    inline constexpr int8_t BR_PORT = 9;

    namespace catapult
    {
        inline constexpr int8_t PORT = 12;
        inline constexpr bool REVERSED = false;
        inline constexpr auto POS_PIDF = comets::PIDF_Value{
            .P = 0.01,
            .I = 0.0,
            .D = 0.0,
            .F = 0.05};

        inline constexpr auto VEL_PIDF = comets::PIDF_Value{
            .P = 0.03,
            .I = 0.0,
            .D = 0.02,
            .F = 0.10};

        inline constexpr auto STORED_POSITION = 0.0;
        inline constexpr auto EXTENDED_POSITION = 550.0;

    } // namespace catapult

    // Max velocity of auton, in RPM
    inline constexpr double TURN_VEL_MULT = 0.3;

    inline constexpr double TELEOP_POLL_TIME = 10.0; // ms
}
#endif