// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#ifndef _MYROBOT_H_
#define _MYROBOT_H_

#include <WPILib.h>
#include "Balance.h"

class MyRobot : public IterativeRobot
{
public:
    typedef enum { kFlightStick, kArcade, kTwoStick } DriveType;

    MyRobot();

    void RobotInit();

    void DisabledInit();
    void AutonomousInit();
    void TeleopInit();

    void DisabledPeriodic();
    void AutonomousPeriodic();
    void TeleopPeriodic();

    void DisabledContinuous();
    void AutonomousContinuous();
    void TeleopContinuous();

private:
    // Tank/Arcade drive with 2 CIM motors and 1 or 2 joysticks.
    SendableChooser driveChooser;
    DriveType driveMode;

    // driver station inputs
    Joystick joy_left,
    	     joy_right;

    // robot outputs
    Victor motor_left,
    	   motor_right;

    // robot control
    RobotDrive drive;

    // Gyro (rate of tilt sensor) input
    AnalogChannel tilt;

    // Bridge Balance Control
    Balance balance;
};

#endif // _MYROBOT_H_
