// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

MyRobot::MyRobot() :
    driveChooser(),
    driveMode(kFlightStick),
    controlChooser(),
    controlMode(kVoltage),
    joy_right( 1 ),
    joy_left(  2 ),
    motor_right_1( 6 ),
    motor_right_2( 8 ),
    motor_left_1(  7 ),
    motor_left_2(  5 ),
    drive( motor_left_1, motor_left_2, motor_right_1, motor_right_2 ),
    tilt( 1 ),
    balance( drive, tilt )
{
    printf("File Versions:\n%s\n", Version::GetVersions());

    driveChooser.AddDefault("FlightStick", (void *) kFlightStick);
    driveChooser.AddObject("Arcade",       (void *) kArcade);
    driveChooser.AddObject("TwoStick",     (void *) kTwoStick);
    SmartDashboard::GetInstance()->PutData("Drive", &driveChooser);

    controlChooser.AddDefault("Voltage", (void *) kVoltage);
    controlChooser.AddObject("Speed",    (void *) kSpeed);
    SmartDashboard::GetInstance()->PutData("Control", &controlChooser);
}

void MyRobot::RobotInit()
{
    balance.InitBalance();
    drive.Drive(0.0F, 0.0F);

    DisableMotors();

    SmartDashboard::Log("Initialized", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Initialized");
    lcd->UpdateLCD();
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
    motor.Set( 0.0F, 0 );
    motor.EnableControl();
//  motor.SetSafetyEnabled( true );
}

void MyRobot::EnableVoltageControl()
{
    EnableVoltageControl( motor_left_1 );
    EnableVoltageControl( motor_right_1 );
    EnableVoltageControl( motor_left_2 );
    EnableVoltageControl( motor_right_2 );

    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetMaxOutput( 1.0 );	// 100% of Vbus
//  drive.SetSafetyEnabled( true );
}

void MyRobot::EnableSpeedControl( CANJaguar& motor )
{
    motor.ChangeControlMode( CANJaguar::kSpeed );
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Coast );
    motor.SetSpeedReference( CANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );  // or 250, or 300?
    motor.SetPID( 0.300, 0.003, 0.001 );
    motor.Set( 0.0F, 0 );
    motor.EnableControl();
//  motor.SetSafetyEnabled( true );
}

void MyRobot::EnableSpeedControl()
{
    EnableSpeedControl( motor_left_1 );
    EnableSpeedControl( motor_right_1 );
    EnableSpeedControl( motor_left_2 );
    EnableSpeedControl( motor_right_2 );

    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetMaxOutput( 100 );  // 100 RPM
//  drive.SetSafetyEnabled( true );
}

START_ROBOT_CLASS(MyRobot);

