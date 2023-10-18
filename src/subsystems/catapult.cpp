#include "constants.h"
#include "subsystems/catapult.h"

Catapult::Catapult() : m_motor(constants::ARM_PORT)
{
    m_motor.setPosPID(constants::ARM_POS_PIDF.F, constants::ARM_POS_PIDF.P, constants::ARM_POS_PIDF.I, constants::ARM_POS_PIDF.D);
    m_motor.setVelPID(constants::ARM_VEL_PIDF.F, constants::ARM_VEL_PIDF.P, constants::ARM_VEL_PIDF.I, constants::ARM_VEL_PIDF.D);
    m_motor.setReversed(constants::ARM_REVERSED);
    reset_pos();
}

double Catapult::get_pos()
{
    return m_motor.getPosition();
}

double Catapult::get_vel()
{
    return m_motor.getActualVelocity();
}

void Catapult::reset_pos()
{
    m_motor.tarePosition();
}

void Catapult::wind_arm()
{
    m_motor.moveAbsolute(550, 400);
}

void Catapult::release_arm()
{
    m_motor.moveAbsolute(0, 400);
}

bool Catapult::is_motor_resting()
{
    const auto pos = get_pos();
    if (pos > -10.0 || pos < 15.0)
    {
        return true;
    }
    return false;
}
