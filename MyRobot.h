// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _MYROBOT_H_
#define _MYROBOT_H_

#include <WPILib.h>
#include "xCANJaguar.h"
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Pickup.h"
#include "Shooter.h"
#include "Target.h"
#include "OI.h"
#include "AutoCommand.h"
#include "DriveCommand.h"
#include "TurnCommand.h"
#include "ShootCommand.h"

class MyRobot : public IterativeRobot
{
    friend class DriveCommand;
    friend class TurnCommand;
    friend class ShootCommand;
    friend class AutoCommand;
    friend class Balance;

private:
    // shaft encoder counts per inch of robot movement
    static double driveScale;

    // shaft encoder counts per degree of robot rotation
    static double turnScale;

    // default PID tolerance
    static double driveTolerance;
    static double turnTolerance;

    ///////////////////////////////////////////////////////////////////
    // driver station
    ///////////////////////////////////////////////////////////////////

    OI m_oi;

    ///////////////////////////////////////////////////////////////////
    // cRIO inputs and outputs
    ///////////////////////////////////////////////////////////////////

    // motor controllers
    xCANJaguar motor_right, motor_left;

    // Compressor (control and sensor)
    Compressor compressor;

    // Cow-catcher control
    Solenoid cowcatcher;

    // Ball picker-upper motor, bidirectional
    Relay ball_pickup;

    // Shooter components are encapsulated in Shooter class

    // Autonomous mode selector is encapsulated in AutoCommand class

    // Camera illuminator on/off
    Relay illuminator;

    // Magic lights on/off
    Relay magic;

    // Bridge support ram
    Solenoid tipper;

    // "on ramp" photo sensor
    DigitalInput onRamp;

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

    AutoCommand  m_autoCommand;
    DriveCommand m_driveCommand;
    TurnCommand  m_turnCommand;
    ShootCommand m_shootCommand;

    typedef enum { kManual, kTurn, kAim, kShoot } DriveMode;
    DriveMode m_driveMode;
    long m_driveTime;	// in milliseconds

    ///////////////////////////////////////////////////////////////////
    // helper functions
    ///////////////////////////////////////////////////////////////////

    static void DisableMotor( xCANJaguar& motor );
    static void EnableVoltageControl( xCANJaguar& motor );
    static void EnableSpeedControl( xCANJaguar& motor, double p, double i, double d );
    static void EnablePositionControl( xCANJaguar& motor, double p, double i, double d );

    double GetJaguarPosition( xCANJaguar& jag, const char *name );
    double GetJaguarDistance( xCANJaguar& jag, const char *name );
    double GetJaguarAngle( xCANJaguar& jag, const char *name );

public:
    MyRobot();

    OI& GetOI() { return m_oi; };

    void StopTheWorld();
    void Safe();
    void DisableMotors();
    void EnableVoltageControl();
    void EnableSpeedControl();
    void EnablePositionControl();
    bool DriveToPosition( float distance );
    bool TurnToAngle( float angle, float tolerance = (float) turnTolerance );

    static void ShowState( const char *mode, const char *state );

    virtual void RobotInit();

    virtual void DisabledInit();
    virtual void AutonomousInit();
    virtual void TeleopInit();

    virtual void DisabledPeriodic();
    virtual void AutonomousPeriodic();
    virtual void TeleopPeriodic();

    virtual void DisabledContinuous();
    virtual void AutonomousContinuous();
    virtual void TeleopContinuous();
};

#endif // _MYROBOT_H_
