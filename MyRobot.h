// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _MYROBOT_H_
#define _MYROBOT_H_

#include <WPILib.h>
#include "xCANJaguar.h"
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Balance.h"
#include "Pickup.h"
#include "Shooter.h"
#include "Target.h"

class MyRobot : public IterativeRobot
{
public:
    MyRobot();

    void RobotInit();

    void Safe();
    void DisableMotors();
    void EnableVoltageControl();
    void EnableSpeedControl();
    void EnablePositionControl();
    bool TurnToPosition( float angle, float tolerance );
    double MyRobot::GetJaguarPosition( xCANJaguar& jag, const char *name );
    void StopTheWorld();

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
    Joystick joy_right, joy_left;

    ///////////////////////////////////////////////////////////////////
    // cRIO inputs and outputs
    ///////////////////////////////////////////////////////////////////

    // motor controllers
    xCANJaguar motor_right_1, motor_right_2,
	      motor_left_1, motor_left_2;

    // Gyro (rate of pitch/yaw) inputs
    Gyro pitch, yaw;

    // Compressor (control and sensor)
    Compressor compressor;

    // Cow-catcher control
    Solenoid cowcatcher;

    // Ball picker-upper motor, bidirectional
    Relay ball_pickup;

    // Shooter components are encapsulated in Shooter class

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

    // Ball pickup control
    Pickup pickup;

    // Shooter control
    Shooter shooter;

    // Targeting
    Target target;
    Target::TargetLocation targetLocation;
    enum { kManual, kLooking, kTurning, kShooting, kNoTarget } fireControl;

    // Manual aiming "nudge"
    bool turning;
    bool turnComplete;

    ///////////////////////////////////////////////////////////////////
    // helper functions
    ///////////////////////////////////////////////////////////////////

    static void DisableMotor( xCANJaguar& motor );
    static void EnableVoltageControl( xCANJaguar& motor );
    static void EnableSpeedControl( xCANJaguar& motor );
    static void EnablePositionControl( xCANJaguar& motor );
};

#endif // _MYROBOT_H_
