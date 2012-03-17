// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

// CAN bus
#define	MOTOR_LEFT	5
#define	MOTOR_RIGHT	6

// analog inputs
#define	PITCH_ANAIN	1
#define	YAW_ANAIN	2

// digital inputs
#define PRESSURE_DIGIN	1
#define	BALL_RDY_DIGIN	2	// ball-in-place sensor, not used
#define	SHOOT_BOT_DIGIN	3
#define	SHOOT_TOP_DIGIN	4
#define	ENCODER_LEFT_A	5	// shaft encoder inputs, not used
#define	ENCODER_LEFT_B	6	// shaft encoder inputs, not used
#define	ENCODER_RIGHT_A	7	// shaft encoder inputs, not used
#define	ENCODER_RIGHT_B	8	// shaft encoder inputs, not used

// PWM outputs
#define	SHOOT_BOT_PWM	1
#define	SHOOT_TOP_PWM	2

// relay (Spike) outputs
#define	COMPRESSOR_RLY	1
#define	PICKUP_RLY	2
#define	ILLUMINATOR_RLY	3

// solenoid (pneumatic) outputs
#define	COWCATCHER_SOL	1
#define	INJECTOR_SOL	2

// shaft encoder counts
#define	ENCODER_COUNT	250	// or 300 or 360


MyRobot::MyRobot() :
    m_oi(),
    motor_right( MOTOR_RIGHT ),
    motor_left( MOTOR_LEFT ),
    pitch( PITCH_ANAIN ),
    yaw( YAW_ANAIN ),
    compressor( COMPRESSOR_RLY, PRESSURE_DIGIN ),
    cowcatcher( COWCATCHER_SOL ),
    ball_pickup( PICKUP_RLY, Relay::kBothDirections ),
    illuminator( ILLUMINATOR_RLY, Relay::kForwardOnly ),
    drive( motor_left, motor_right ),
    pickup( ball_pickup ),
    shooter( SHOOT_BOT_PWM, SHOOT_TOP_PWM, SHOOT_BOT_DIGIN, SHOOT_TOP_DIGIN, INJECTOR_SOL ),
    target(),
    m_autoCommand(*this),
    m_driveCommand(*this),
    m_turnCommand(*this),
    m_shootCommand(*this),
    m_balance(*this),
    driveMode(kManual),
    driveTime(0)
{
    printf("File Versions:\n%s\n", Version::GetVersions());
    RobotInit();
}

void MyRobot::RobotInit()
{
    Safe();
    // We don't care about the camera right now, just that it's instantiated.
    (void) AxisCamera::GetInstance();
    ShowState("Initialized","Idle");
}

void MyRobot::StopTheWorld()
{
    Safe();
    while (!IsDisabled())
	;
}

void MyRobot::Safe()
{
    compressor.Stop();
    m_driveCommand.Stop();
    m_turnCommand.Stop();
    m_shootCommand.Stop();
    m_balance.Stop();
    DisableMotors();
    pickup.Stop();
    shooter.Stop();
    cowcatcher.Set( false );
}

void MyRobot::ShowState( char *mode, char *state )
{
    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line1, mode);
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, state);
    lcd->UpdateLCD();

    SmartDashboard::Log(state, "Robot State");
}

void MyRobot::DisableMotor( xCANJaguar& motor )
{
    motor.DisableControl();
    motor.SetSafetyEnabled(false);	// CANJaguar *should* do this
}

void MyRobot::DisableMotors()
{
    drive.StopMotor();
    drive.SetSafetyEnabled( false );	// RobotDrive *should* do this

    DisableMotor( motor_left );	// RobotDrive *should* do this as well
    DisableMotor( motor_right );
}

void MyRobot::EnableVoltageControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kPercentVbus );
    motor.ConfigMaxOutputVoltage( 13.2 );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Coast );

    // force change in control mode
    motor.EnableControl();

    // Feed the watchdog now to avoid a race condition when enabling
    //   motorSafetyHelper with the previous timer already expired.
    motor.Set( 0.0F, 0 );

    // Now it's safe to enable.
    motor.SetSafetyEnabled( true );

    // Set the timer a little longer than default
    //   to allow for CAN timeouts and retries.
    motor.SetExpiration( 0.5 );

    // Feed the watchdog again with the new interval.
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableVoltageControl()
{
    EnableVoltageControl( motor_left );
    EnableVoltageControl( motor_right );

    drive.SetMaxOutput( 1.0 );	// 100% of Vbus

    // Feed the watchdog now to avoid a race condition when enabling
    //   motorSafetyHelper with the previous timer already expired.
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );

    // Now it's safe to enable.
    drive.SetSafetyEnabled( true );

    // Set the timer a little longer than default
    //   to allow for CAN timeouts and retries.
    drive.SetExpiration( 0.5 );

    // Feed the watchdog again with the new interval.
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
}

void MyRobot::EnableSpeedControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kSpeed );
    motor.ConfigMaxOutputVoltage( 13.2 );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Brake );
    motor.SetSpeedReference( xCANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( ENCODER_COUNT );
    motor.SetPID( 0.300, 0.003, 0.001 );

    // force change in control mode
    motor.EnableControl();

    // Feed the watchdog now to avoid a race condition when enabling
    //   motorSafetyHelper with the previous timer already expired.
    motor.Set( 0.0F, 0 );

    // Now it's safe to enable.
    motor.SetSafetyEnabled( true );

    // Set the timer a little longer than default
    //   to allow for CAN timeouts and retries.
    motor.SetExpiration( 0.5 );

    // Feed the watchdog again with the new interval.
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableSpeedControl()
{
    EnableSpeedControl( motor_left );
    EnableSpeedControl( motor_right );

    drive.SetMaxOutput( 300 );			// 300 RPM is close to practical top speed

    // Feed the watchdog now to avoid a race condition when enabling
    //   motorSafetyHelper with the previous timer already expired.
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );

    // Now it's safe to enable.
    drive.SetSafetyEnabled( true );

    // Set the timer a little longer than default
    //   to allow for CAN timeouts and retries.
    drive.SetExpiration( 0.5 );

    // Feed the watchdog again with the new interval.
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
}

void MyRobot::EnablePositionControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kPosition );
    motor.ConfigMaxOutputVoltage( 6. );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Brake );
    motor.SetPositionReference( xCANJaguar::kPosRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( ENCODER_COUNT );
    motor.SetPID( 500.0, 0.1, 40.0 );	// TBD: tune this for position control

    // force change in control mode
    motor.EnableControl( 0.0 );

    // Feed the watchdog now to avoid a race condition when enabling
    //   motorSafetyHelper with the previous timer already expired.
    motor.Set( 0.0F, 0 );

    // Now it's safe to enable.
    motor.SetSafetyEnabled( true );

    // Set the timer a little longer than default
    //   to allow for CAN timeouts and retries.
    motor.SetExpiration( 0.5 );

    // Feed the watchdog again with the new interval.
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnablePositionControl()
{
    EnablePositionControl( motor_left );
    EnablePositionControl( motor_right );

    // Bypass the RobotDrive class for this mode since it doesn't deal
    //   well with arbitrarily large setpoints for multiple wheel rotations.
    drive.SetSafetyEnabled( false );

    // Start the timer
    driveTime = GetFPGATime();
}

// shaft encoder counts per inch of robot movement (straight-line movement)
// driveScale = (1.0 inch / wheel circumference) * (wheel gear teeth / drive gear teeth);
//            = (  1.0    /      (8.0 * PI)    ) * (      36         /       17        );
//
const double MyRobot::driveScale = 0.08426;


// shaft encoder counts per degree of robot rotation (when turning in place)
// turnScale = (turn circumference / wheel circumference)
//             * (wheel gear teeth / drive gear teeth) / 360 degrees
//           = (    (19.25*PI)     /      (8.0*PI)      )
//             * (      36         /       17        ) / 360
//
const double MyRobot::turnScale = 0.01415;

double MyRobot::GetJaguarPosition( xCANJaguar& jag, const char *name )
{
    double position;

    jag.ClearError();
    position = jag.GetPosition();
    if (jag.StatusIsFatal()) {
	int code = jag.GetError().GetCode();
	printf("xCANJaguar %s error %d\n", name, code);
	jag.ClearError();
	if (jag.GetPowerCycled()) {
	    printf("xCANJaguar %s was power cycled\n", name);
	}
	int faults = jag.GetFaults();
	if (faults) {
	    printf("xCANJaguar %s faults 0x%x\n", name, faults);
	}
	StopTheWorld();
    }
    return position;
}

double MyRobot::GetJaguarDistance( xCANJaguar& jag, const char *name )
{
    return GetJaguarPosition(jag, name) / driveScale;
}

bool MyRobot::DriveToPosition( float distance, float tolerance )
{
    float left = GetJaguarDistance(motor_left,"left");
    float right = -GetJaguarDistance(motor_right,"right");

#if 0
    {
	long ms = ((long)GetFPGATime() - driveTime) / 1000;
	printf("ms %5ld left %g right %g\n", ms, left, right);
    }
#endif

    if (fabs(left - distance) < tolerance &&
	fabs(right - distance) < tolerance)
    {
	return true;
    }
    else
    {
	float pos = distance * driveScale;
	motor_left.Set(pos, 1);
	motor_right.Set(-pos, 1);
	xCANJaguar::UpdateSyncGroup(1);
	return false;
    }
}

double MyRobot::GetJaguarAngle( xCANJaguar& jag, const char *name )
{
    return GetJaguarPosition(jag, name) / turnScale;
}

bool MyRobot::TurnToAngle( float angle, float tolerance )
{
    float left = GetJaguarAngle(motor_left,"left");
    float right = GetJaguarAngle(motor_right,"right");

#if 0
    {
	long ms = ((long)GetFPGATime() - driveTime) / 1000;
	printf("ms %5ld left %g right %g\n", ms, left, right);
    }
#endif

    if (fabs(left - angle) < tolerance && fabs(right - angle) < tolerance)
    {
	return true;
    }
    else
    {
	float pos = angle * turnScale;
	motor_left.Set(pos, 1);
	motor_right.Set(pos, 1);
	xCANJaguar::UpdateSyncGroup(1);
	return false;
    }
}

START_ROBOT_CLASS(MyRobot);

