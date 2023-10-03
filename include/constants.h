#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include "comets/paths.h"

namespace constants
{
    inline constexpr auto CHASSIS_GEARSET = AbstractMotor::gearset::green;
    inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
    inline constexpr auto CHASSIS_TPR = imev5GreenTPR;

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

    inline constexpr int8_t FL_PORT = 1;
    inline constexpr int8_t FR_PORT = 19;
    inline constexpr int8_t BL_PORT = 11;
    inline constexpr int8_t BR_PORT = 20;

    inline constexpr int8_t ARM_1_LEFT = 6;
    inline constexpr int8_t ARM_1_RIGHT = 16;
    inline constexpr int8_t ARM_2 = 7;
    inline constexpr int8_t CLAW = 10;

    inline constexpr int8_t ARM_1_SPEED = 100;
    inline constexpr int8_t ARM_2_SPEED = 100;
    inline constexpr int8_t CLAW_SPEED = 100;

    // Max velocity of auton, in RPM
    inline constexpr double TURN_VEL_MULT = 0.3;

    inline constexpr double TELEOP_POLL_TIME = 10.0; // ms
}
#endif