// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

// WPILib Includes
#include <WPILib.h>

// Our Includes
#include "MyRobot.h"
#include "Version.h"

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

MyRobot::MyRobot() :

    driveChooser(),
    driveMode(kFlightStick),
    joy_left( 1 ),	// driver station left-hand joystick on USB port 1
    joy_right( 2 ),	// driver station right-hand joystick on USB port 2
    motor_left( 1 ),	// left Victor/CIM on PWM 1
    motor_right( 2 ),	// left Victor/CIM on PWM 2
    drive( motor_left, motor_right ),
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
    balance.Stop();
    drive.StopMotor();

    SmartDashboard::Log("Initialized", "Robot State");

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Initialized");
    lcd->UpdateLCD();

}

START_ROBOT_CLASS(MyRobot);

