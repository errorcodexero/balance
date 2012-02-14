// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _MYROBOT_H_
#define _MYROBOT_H_

#include <WPILib.h>
#include "Balance.h"

class MyRobot : public IterativeRobot
{
public:
    typedef enum { kFlightStick, kArcade, kTwoStick } DriveType;
    typedef enum { kVoltage, kSpeed } ControlMode;

    MyRobot();

    void RobotInit();

    void DisableMotors();
    void EnableVoltageControl();
    void EnableSpeedControl();

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
    static void DisableMotor( CANJaguar& motor );
    static void EnableVoltageControl( CANJaguar& motor );
    static void EnableSpeedControl( CANJaguar& motor );

    // Tank/Arcade drive with 2 CIM motors and 1 or 2 joysticks.
    SendableChooser driveChooser;
    DriveType driveMode;
    SendableChooser controlChooser;
    ControlMode controlMode;

    // driver station inputs
    Joystick joy_right, joy_left;

    // robot outputs
    CANJaguar motor_right_1, motor_right_2,
		   motor_left_1, motor_left_2;

    // robot control
    RobotDrive drive;

    // Gyro (rate of tilt sensor) input
    AnalogChannel tilt;

    // Bridge Balance Control
    Balance balance;
};

#endif // _MYROBOT_H_
