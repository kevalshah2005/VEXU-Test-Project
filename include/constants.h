#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <array>
#include "okapi/api.hpp"

namespace constants
{
    inline constexpr auto CHASSIS_GEARSET = AbstractMotor::gearset::green;
    inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
    inline constexpr auto CHASSIS_TPR = imev5GreenTPR;

    inline constexpr okapi::PathfinderLimits PATH_LIMITS = {
        1.0  * 0.66, // Maximum linear velocity of the Chassis in m/s
        2.0  * 0.66, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 * 0.66  // Maximum linear jerk of the Chassis in m/s/s/s
    };

    inline constexpr bool LEFT_REVERSED = false;
    inline constexpr bool RIGHT_REVERSED = true;

    inline constexpr int8_t FL_PORT = 20;
    inline constexpr int8_t FR_PORT = 11;
    inline constexpr int8_t BL_PORT = 10;
    inline constexpr int8_t BR_PORT = 9;

    // Max velocity of auton, in RPM
    inline constexpr double TURN_VEL_MULT = 0.3;

    inline constexpr double TELEOP_POLL_TIME = 10.0; // ms
}
#endif