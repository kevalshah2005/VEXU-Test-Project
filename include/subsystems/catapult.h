#ifndef __SUBSYSTEMS_CATAPULT_H__
#define __SUBSYSTEMS_CATAPULT_H__

#include "comets/vendor.h"
#include <memory>

class Catapult
{
public:
    Catapult();

    double get_pos();
    double get_vel();

    void reset_pos();

    void wind_arm();
    void release_arm();

    inline okapi::Motor& get_motor() {
        return m_motor;
    }

private:
    okapi::Motor m_motor;

    bool is_motor_resting();
};

#endif
