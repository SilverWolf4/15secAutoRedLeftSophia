#ifndef _PROS_CONFIGS_H_
#define _PROS_CONFIGS_H_
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//drivetrain
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup slide;
extern pros::adi::DigitalOut claw;
extern pros::Distance distance_sensor;
extern lemlib::Chassis chassis;

//other motors

#endif // _PROS_CONFIGS_H_