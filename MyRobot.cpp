// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include <math.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

MyRobot::MyRobot() :
    joy_right( 1, 4, 7 ),
    joy_left(  2, 4, 7 ),
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
    driveChooser(),
    driveMode( kFlightStick ),
    controlChooser(),
    controlMode( kVoltage ),
    drive( motor_left_1, motor_left_2, motor_right_1, motor_right_2 ),
    balance( drive, pitch ),
    pickup( ball_pickup ),
    shooter( 1, 2, 3, 4, 2 ),
    target(),
    fireControl(kManual)
{
    printf("File Versions:\n%s\n", Version::GetVersions());

    driveChooser.AddDefault("FlightStick", (void *) kFlightStick);
    driveChooser.AddObject("Arcade",       (void *) kArcade);
    driveChooser.AddObject("X-Y",          (void *) kXY);
    driveChooser.AddObject("TwoStick",     (void *) kTwoStick);
    SmartDashboard::GetInstance()->PutData("Drive", &driveChooser);

    controlChooser.AddDefault("Voltage", (void *) kVoltage);
    controlChooser.AddObject("Speed",    (void *) kSpeed);
    SmartDashboard::GetInstance()->PutData("Control", &controlChooser);
}

void MyRobot::RobotInit()
{
    balance.InitBalance();
    shooter.InitShooter();
    Safe();

    SmartDashboard::Log("Initialized", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Initialized");
    lcd->UpdateLCD();
}

void MyRobot::Safe()
{
    compressor.Stop();
    balance.Stop();
    DisableMotors();
    pickup.Stop();
    shooter.Stop();
    cowcatcher.Set( false );
    fireControl = kManual;
}

void MyRobot::StopTheWorld()
{
    Safe();
    while (true) ;
    // no return
}

void MyRobot::DisableMotor( xCANJaguar& motor )
{
    motor.DisableControl();
    motor.SetSafetyEnabled(false);
}

void MyRobot::DisableMotors()
{
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetSafetyEnabled( false );

    DisableMotor( motor_left_1 );
    DisableMotor( motor_left_2 );
    DisableMotor( motor_right_1 );
    DisableMotor( motor_right_2 );
}

void MyRobot::EnableVoltageControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kPercentVbus );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Coast );
//    motor.SetSafetyEnabled( true );
    motor.EnableControl();
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableVoltageControl()
{
    EnableVoltageControl( motor_left_1 );
    EnableVoltageControl( motor_right_1 );
    EnableVoltageControl( motor_left_2 );
    EnableVoltageControl( motor_right_2 );

    drive.SetMaxOutput( 1.0 );	// 100% of Vbus
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetSafetyEnabled( true );
}

void MyRobot::EnableSpeedControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kSpeed );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Coast );
    motor.SetSpeedReference( xCANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );  // or 250, or 300?
    motor.SetPID( 0.300, 0.003, 0.001 );
//    motor.SetSafetyEnabled( true );
    motor.EnableControl();
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableSpeedControl()
{
    EnableSpeedControl( motor_left_1 );
    EnableSpeedControl( motor_right_1 );
    EnableSpeedControl( motor_left_2 );
    EnableSpeedControl( motor_right_2 );

    drive.SetMaxOutput( 200 );			// 200 RPM is somewhat slower than top speed
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetSafetyEnabled( true );
}

void MyRobot::EnablePositionControl( xCANJaguar& motor )
{
    motor.ChangeControlMode( xCANJaguar::kPosition );
    motor.ConfigNeutralMode( xCANJaguar::kNeutralMode_Brake );
    motor.SetPositionReference( xCANJaguar::kPosRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );	// or 250, or 300?, adjust for gear ratio?
    motor.SetPID( 1000.0, 0.0, 10.0 );		// TBD: tune this for position control
//    motor.SetSafetyEnabled( true );
    motor.EnableControl( 0.0 );
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnablePositionControl()
{
    EnablePositionControl( motor_left_1 );
    EnablePositionControl( motor_right_1 );
    EnablePositionControl( motor_left_2 );
    EnablePositionControl( motor_right_2 );

    drive.SetSafetyEnabled( false );  // bypass the RobotDrive class for this mode
}

const float turnScale = 0.01415;	// calculated value

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
    return position / turnScale;
}

bool MyRobot::TurnToPosition( float angle, float tolerance )
{
    // TBD: Tune scaling factor to match drive gear ration,
    //      wheel size and wheelbase.

    if (fabs(GetJaguarPosition(motor_left_1,"left_1") - angle) < tolerance &&
	fabs(GetJaguarPosition(motor_left_2,"left_2") - angle) < tolerance &&
	fabs(GetJaguarPosition(motor_right_1,"right_1") - angle) < tolerance &&
	fabs(GetJaguarPosition(motor_right_2,"right_2") - angle) < tolerance)
    {
	DisableMotors();
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

