// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _MYROBOT_H_
#define _MYROBOT_H_

#include <WPILib.h>
#include "Balance.h"
#include "Smart.h"

class MyRobot : public IterativeRobot
{
public:
    MyRobot();

    void RobotInit();

    void DisableMotors();
    void EnableVoltageControl();
    void EnableSpeedControl();
    void EnablePositionControl();

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
    ///////////////////////////////////////////////////////////////////
    // driver station
    ///////////////////////////////////////////////////////////////////

    // driver station inputs
    SmartJoystick joy_right, joy_left;

    ///////////////////////////////////////////////////////////////////
    // cRIO inputs and outputs
    ///////////////////////////////////////////////////////////////////

    // motor controllers
    CANJaguar motor_right_1, motor_right_2,
		   motor_left_1, motor_left_2;

    // Gyro (rate of pitch/yaw) inputs
    AnalogChannel pitch, yaw;

    // Compressor (control and sensor)
    Compressor compressor;

    // Cow-catcher control
    DoubleSolenoid catcher;

    // Ball picker-upper motor, bidirectional
    Relay upper;

    // Ball injector
    DoubleSolenoid injector;

    // Shooter wheel
    Victor shooter_1, shooter_2;
    Counter speedy;

    // Camera illuminator on/off
    Relay illuminator;

    ///////////////////////////////////////////////////////////////////
    // robot control
    ///////////////////////////////////////////////////////////////////

    // dashboard controls
    SendableChooser driveChooser;
    typedef enum { kFlightStick, kArcade, kXY, kTwoStick } DriveType;
    DriveType driveMode;

    SendableChooser controlChooser;
    typedef enum { kVoltage, kSpeed } ControlMode;
    ControlMode controlMode;

    // Tank/Arcade drive with 2 CIM motors and 1 or 2 joysticks.
    RobotDrive drive;

    // Bridge Balance Control
    Balance balance;

    ///////////////////////////////////////////////////////////////////
    // helper functions
    ///////////////////////////////////////////////////////////////////

    static void DisableMotor( CANJaguar& motor );
    static void EnableVoltageControl( CANJaguar& motor );
    static void EnableSpeedControl( CANJaguar& motor );
    static void EnablePositionControl( CANJaguar& motor );
};

#endif // _MYROBOT_H_
