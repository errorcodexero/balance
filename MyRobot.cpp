// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

MyRobot::MyRobot() :
    joy_right( 1, "Right" ),
    joy_left(  2, "Left" ),
    motor_right_1( 6 ),
    motor_right_2( 8 ),
    motor_left_1(  7 ),
    motor_left_2(  5 ),
    pitch( 1 ),
    yaw( 2 ),
    compressor( 1, 1 ),
    cowcatcher( 1 ),
    ball_pickup( 2, Relay::kBothDirections ),
    ball_loaded( 2 ),
    ball_injector( 2 ),
    shooter_bottom( 1 ),
    shooter_top( 2 ),
    shot_speed_bottom( 3, false ),
    shot_speed_top( 4, false ),
    illuminator( 3, Relay::kForwardOnly ),
    driveChooser(),
    driveMode( kFlightStick ),
    controlChooser(),
    controlMode( kVoltage ),
    drive( motor_left_1, motor_left_2, motor_right_1, motor_right_2 ),
    balance( drive, pitch ),
    pickup( ball_pickup, ball_loaded ),
    lastPickup( 0 ),
    shooter( shooter_bottom, shooter_top, shot_speed_bottom, shot_speed_top )
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
    DisableMotors();

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
    ball_injector.Set( false );
}

void MyRobot::DisableMotor( CANJaguar& motor )
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

void MyRobot::EnableVoltageControl( CANJaguar& motor )
{
    motor.ChangeControlMode( CANJaguar::kPercentVbus );
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Coast );
    motor.SetSafetyEnabled( true );
    motor.EnableControl();
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableVoltageControl()
{
    EnableVoltageControl( motor_left_1 );
    EnableVoltageControl( motor_right_1 );
    EnableVoltageControl( motor_left_2 );
    EnableVoltageControl( motor_right_2 );

    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetMaxOutput( 1.0 );	// 100% of Vbus
    drive.SetSafetyEnabled( true );
}

void MyRobot::EnableSpeedControl( CANJaguar& motor )
{
    motor.ChangeControlMode( CANJaguar::kSpeed );
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Coast );
    motor.SetSpeedReference( CANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );  // or 250, or 300?
    motor.SetPID( 0.300, 0.003, 0.001 );
    motor.SetSafetyEnabled( true );
    motor.EnableControl();
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnableSpeedControl()
{
    EnableSpeedControl( motor_left_1 );
    EnableSpeedControl( motor_right_1 );
    EnableSpeedControl( motor_left_2 );
    EnableSpeedControl( motor_right_2 );

    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetMaxOutput( 200 );			// 200 RPM is somewhat slower than top speed
    drive.SetSafetyEnabled( true );
}

void MyRobot::EnablePositionControl( CANJaguar& motor )
{
    motor.ChangeControlMode( CANJaguar::kPosition );
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Brake );
    motor.SetPositionReference( CANJaguar::kPosRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );	// or 250, or 300?, adjust for gear ratio?
    motor.SetPID( 1000.0, 0.0, 10.0 );		// TBD: tune this for position control
    motor.SetSafetyEnabled( true );
    motor.EnableControl( 0.0 );
    motor.Set( 0.0F, 0 );
}

void MyRobot::EnablePositionControl()
{
    EnablePositionControl( motor_left_1 );
    EnablePositionControl( motor_right_1 );
    EnablePositionControl( motor_left_2 );
    EnablePositionControl( motor_right_2 );

    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetMaxOutput( 100 );			 // 100 revolutions?
    drive.SetSafetyEnabled( true );
}

START_ROBOT_CLASS(MyRobot);

