// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

MyRobot::MyRobot() :
    m_oi(),
    motor_right_1( 6 ),
    motor_right_2( 8 ),
    motor_left_1(  7 ),
    motor_left_2(  5 ),
    pitch( 1 ),
    yaw( 2 ),
    compressor( 1, 1 ),
    cowcatcher( 1 ),
    ball_pickup( 2, Relay::kBothDirections ),
    illuminator( 3, Relay::kForwardOnly ),
    drive( motor_left_1, motor_left_2, motor_right_1, motor_right_2 ),
    pickup( ball_pickup ),
    shooter( 1, 2, 3, 4, 2 ),
    target(),
    m_driveCommand(*this),
    m_turnCommand(*this),
    m_shootCommand(*this),
    m_balance( drive, pitch ),
    driveMode(kManual)
{
    printf("File Versions:\n%s\n", Version::GetVersions());
}

void MyRobot::RobotInit()
{
    m_balance.InitBalance();
    shooter.InitShooter();
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

    DisableMotor( motor_left_1 );	// RobotDrive *should* do this as well
    DisableMotor( motor_left_2 );
    DisableMotor( motor_right_1 );
    DisableMotor( motor_right_2 );
}

void MyRobot::EnableVoltageControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kPercentVbus );
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
    EnableVoltageControl( motor_left_1 );
    EnableVoltageControl( motor_right_1 );
    EnableVoltageControl( motor_left_2 );
    EnableVoltageControl( motor_right_2 );

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
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Coast );
    motor.SetSpeedReference( xCANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );  // or 250, or 300?
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
    EnableSpeedControl( motor_left_1 );
    EnableSpeedControl( motor_right_1 );
    EnableSpeedControl( motor_left_2 );
    EnableSpeedControl( motor_right_2 );

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
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Brake );
    motor.SetPositionReference( xCANJaguar::kPosRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );	// or 250, or 300?, adjust for gear ratio?
    motor.SetPID( 1000.0, 0.0, 10.0 );		// TBD: tune this for position control

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
    EnablePositionControl( motor_left_1 );
    EnablePositionControl( motor_right_1 );
    EnablePositionControl( motor_left_2 );
    EnablePositionControl( motor_right_2 );

    // Bypass the RobotDrive class for this mode since it doesn't deal
    //   well with arbitrarily large setpoints for multiple wheel rotations.
    drive.SetSafetyEnabled( false );
}

// shaft encoder counts per degree of robot rotation (when turning in place)
// turnScale = (wheelbase / wheel diameter) * (wheel gear teeth / drive gear teeth) / 360 degrees
//           = (  19.25   /      8.0      ) * (      36         /       17        ) / 360
//
const double MyRobot::turnScale = 0.01415;

double MyRobot::GetJaguarAngle( xCANJaguar& jag, const char *name )
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
    return position / turnScale;
}

bool MyRobot::TurnToPosition( float angle, float tolerance )
{
    // TBD: Tune scaling factor to match drive gear ration,
    //      wheel size and wheelbase.

    if (fabs(GetJaguarAngle(motor_left_1,"left_1") - angle) < tolerance &&
	fabs(GetJaguarAngle(motor_left_2,"left_2") - angle) < tolerance &&
	fabs(GetJaguarAngle(motor_right_1,"right_1") - angle) < tolerance &&
	fabs(GetJaguarAngle(motor_right_2,"right_2") - angle) < tolerance)
    {
	return true;
    }
    else
    {
	float pos = angle * turnScale;
	motor_left_1.Set(pos, 1);
	motor_left_2.Set(pos, 1);
	motor_right_1.Set(pos, 1);
	motor_right_2.Set(pos, 1);
	xCANJaguar::UpdateSyncGroup(1);
	return false;
    }
}

START_ROBOT_CLASS(MyRobot);

