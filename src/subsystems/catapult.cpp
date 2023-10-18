#include "subsystems/catapult.h"

#include <cassert>
#include "constants.h"
#include "comets/math.h"

static inline constexpr double IDLE_VELOCITY_ERROR_RANGE = 10.0;
static inline constexpr double IDLE_POSITION_ERROR_RANGE = 10.0;

static inline constexpr double STORED_POSITION = 0.0;
static inline constexpr double EXTENDED_POSITION = 550.0;

Catapult::Catapult() : m_motor(constants::ARM_PORT)
{
    m_motor.setPosPID(constants::ARM_POS_PIDF.F, constants::ARM_POS_PIDF.P, constants::ARM_POS_PIDF.I, constants::ARM_POS_PIDF.D);
    m_motor.setVelPID(constants::ARM_VEL_PIDF.F, constants::ARM_VEL_PIDF.P, constants::ARM_VEL_PIDF.I, constants::ARM_VEL_PIDF.D);
    m_motor.setReversed(constants::ARM_REVERSED);
    zero_position();
}

void Catapult::zero_position()
{
    m_motor.tarePosition();
}

void Catapult::wind_arm()
{
    set_position(EXTENDED_POSITION);
}

void Catapult::release_arm()
{
    // although the code would suggest that the release is the same speed as winding
    // but the real robot has elastic bands pulling it torwards the stored position.
    set_position(STORED_POSITION);
}

bool Catapult::is_motor_idle()
{
    const double v_error = m_motor.getVelocityError();
    const double p_error = m_motor.getPositionError();
    static constexpr auto in_range = [](double target, double range)
    {
        return comets::in_range(target, -range / 2, range / 2);
    };
    return in_range(v_error, IDLE_VELOCITY_ERROR_RANGE) &&
           in_range(p_error, IDLE_POSITION_ERROR_RANGE);
}

void Catapult::set_position(double position)
{
    m_motor.moveAbsolute(position, 400);
}
