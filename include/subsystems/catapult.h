#ifndef __SUBSYSTEMS_CATAPULT_H__
#define __SUBSYSTEMS_CATAPULT_H__

#include "comets/vendor.h"
#include <memory>

/**
 * This code just implements an two position arm, but the real mechanism has
 * rubber bands pulling the arm to the zero position.
 */
class Catapult
{
public:
    Catapult();

    bool is_motor_idle() noexcept;

    void zero_position();

    void wind_arm();
    void release_arm();

private:
    okapi::Motor m_motor;

    void set_position(double position);
};

#endif
