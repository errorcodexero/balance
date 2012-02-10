// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

// WPILib Includes
#include <WPILib.h>

// Our Includes
#include "MyRobot.h"
#include "Version.h"

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

#define	ROBOTDRIVE_SYNC_GROUP	0x80	// this is the same sync group RobotDrive uses

MyRobot::MyRobot() :
    driveChooser(),
    driveMode(kFlightStick),
    joy_right( 1 ),	// driver station right-hand joystick on USB port 1
    joy_left( 2 ),	// driver station left-hand joystick on USB port 2
    motor_right_1( 5 ),
    motor_right_2( 6 ),
    motor_left_1( 7 ),
    motor_left_2( 8 ),
    drive( motor_left_1, motor_left_2, motor_right_1, motor_right_2 ),
    tilt( 1 ),
    balance( drive, tilt )
{
    driveChooser.AddDefault("FlightStick", (void *) kFlightStick);
    driveChooser.AddObject("Arcade",       (void *) kArcade);
    driveChooser.AddObject("TwoStick",     (void *) kTwoStick);
    SmartDashboard::GetInstance()->PutData("Drive", &driveChooser);

    printf("File Versions:\n%s\n", Version::GetVersions());
}

void MyRobot::RobotInit()
{
    balance.InitBalance();
    drive.Drive(0.0F, 0.0F);
    drive.SetInvertedMotor( RobotDrive::kFrontLeftMotor,  false );
    drive.SetInvertedMotor( RobotDrive::kRearLeftMotor,   false );
    drive.SetInvertedMotor( RobotDrive::kFrontRightMotor, true );
    drive.SetInvertedMotor( RobotDrive::kRearRightMotor,  true );

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
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Brake );
    motor.EnableControl();
    motor.SetSafetyEnabled( true );

    motor.Set( 0.0F, ROBOTDRIVE_SYNC_GROUP );
}

void MyRobot::EnableVoltageControl()
{
    EnableVoltageControl( motor_left_1 );
    EnableVoltageControl( motor_right_1 );
    EnableVoltageControl( motor_left_2 );
    EnableVoltageControl( motor_right_2 );

    // CANJaguar::UpdateSyncGroup( ROBOTDRIVE_SYNC_GROUP );

    drive.SetMaxOutput( 1.0 );
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetSafetyEnabled( true );
}

void MyRobot::EnableSpeedControl( CANJaguar& motor )
{
    motor.ChangeControlMode( CANJaguar::kSpeed );
    motor.ConfigNeutralMode( CANJaguar::kNeutralMode_Brake );
    motor.SetSpeedReference( CANJaguar::kSpeedRef_QuadEncoder );
    motor.ConfigEncoderCodesPerRev( 360 );  // or 250, or 300?
    motor.SetPID( 0.300, 0.003, 0.001 );
    motor.EnableControl();
    motor.SetSafetyEnabled( true );

    motor.Set( 0.0F, ROBOTDRIVE_SYNC_GROUP );
}

void MyRobot::EnableSpeedControl()
{
    EnableSpeedControl( motor_left_1 );
    EnableSpeedControl( motor_right_1 );
    EnableSpeedControl( motor_left_2 );
    EnableSpeedControl( motor_right_2 );
    // CANJaguar::UpdateSyncGroup( ROBOTDRIVE_SYNC_GROUP );

    drive.SetMaxOutput( 100 );  // 100 RPM
    drive.SetLeftRightMotorOutputs( 0.0F, 0.0F );
    drive.SetSafetyEnabled( true );
}

START_ROBOT_CLASS(MyRobot);

