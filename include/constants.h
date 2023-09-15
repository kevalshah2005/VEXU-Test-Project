#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <array>
#include "okapi/api.hpp"

namespace constants
{
    inline constexpr auto CHASSIS_GEARSET = AbstractMotor::gearset::green;
    inline constexpr auto CHASSIS_DIMS = {4_in, 12.5_in};
    inline constexpr auto CHASSIS_TPR = imev5GreenTPR;

    inline constexpr float AUTON_MAX_LINEAR = 1.0; // Maximum linear velocity of the Chassis in m/s
    inline constexpr float AUTON_MAX_ACCEL = 2.0;  // Maximum linear acceleration of the Chassis in m/s/s
    inline constexpr float AUTON_MAX_JERK = 10.0;  // Maximum linear jerk of the Chassis in m/s/s/s

    inline constexpr bool LEFT_REVERSED = false;
    inline constexpr bool RIGHT_REVERSED = true;

    inline constexpr int8_t FL_PORT = 19;
    inline constexpr int8_t FR_PORT = 11;
    inline constexpr int8_t BL_PORT = 10;
    inline constexpr int8_t BR_PORT = 9;

    // Max velocity of auton, in RPM
    inline constexpr double AUTO_MAX_VELO = 50;
}
#endif