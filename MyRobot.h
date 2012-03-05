// sample robot code
// Steve Tarr - team 1425 mentor

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
#include "OI.h"
#include "DriveCommand.h"
#include "TurnCommand.h"
#include "ShootCommand.h"

class MyRobot : public IterativeRobot
{
    friend class DriveCommand;
    friend class TurnCommand;
    friend class ShootCommand;

private:
    ///////////////////////////////////////////////////////////////////
    // driver station
    ///////////////////////////////////////////////////////////////////

    OI m_oi;

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

    // Tank/Arcade drive with 2 CIM motors and 1 or 2 joysticks.
    RobotDrive drive;

    // Ball pickup control
    Pickup pickup;

    // Shooter control
    Shooter shooter;

    // Targeting
    Target target;

    ///////////////////////////////////////////////////////////////////
    // commands
    ///////////////////////////////////////////////////////////////////

    DriveCommand m_driveCommand;
    TurnCommand  m_turnCommand;
    ShootCommand m_shootCommand;
    Balance m_balance;

    typedef enum { kManual, kTurn, kShoot, kBalance } DriveMode;
    DriveMode driveMode;

    ///////////////////////////////////////////////////////////////////
    // helper functions
    ///////////////////////////////////////////////////////////////////

    static void DisableMotor( xCANJaguar& motor );
    static void EnableVoltageControl( xCANJaguar& motor );
    static void EnableSpeedControl( xCANJaguar& motor );
    static void EnablePositionControl( xCANJaguar& motor );

public:
    MyRobot();

    void RobotInit();

    OI& GetOI() { return m_oi; };

    void Safe();
    void DisableMotors();
    void EnableVoltageControl();
    void EnableSpeedControl();
    void EnablePositionControl();
    bool TurnToPosition( float angle, float tolerance );
    double GetJaguarPosition( xCANJaguar& jag, const char *name );
    void StopTheWorld();

    static void ShowState( char *mode, char *state );

    void DisabledInit();
    void AutonomousInit();
    void TeleopInit();

    void DisabledPeriodic();
    void AutonomousPeriodic();
    void TeleopPeriodic();

    void DisabledContinuous();
    void AutonomousContinuous();
    void TeleopContinuous();

};

#endif // _MYROBOT_H_
