// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "TurnCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float turnTolerance = 0.50;

TurnCommand::TurnCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
}

void TurnCommand::Start()
{
    m_robot.EnablePositionControl();
    turnTimer.Start();
    turnComplete = false;

    MyRobot::ShowState("Teleop", "Manual Turn");
}

void TurnCommand::Stop()
{
    m_robot.DisableMotors();
    turnTimer.Stop();
}

bool TurnCommand::Run()
{
    OI& oi = m_robot.GetOI();

    if (!turnComplete) {

	float angle = oi.TurnLeft10()  ? -10.
		    : oi.TurnRight10() ? 10.
		    : oi.TurnLeft3()   ? -3.
		    : oi.TurnRight3()  ? 3.
		    : 0.;  // can't happen

	const float tolerance = 0.50;

	turnComplete = m_robot.TurnToPosition(angle, tolerance);

	if (turnComplete || turnTimer.HasPeriodPassed(2.0)) {
	    printf("Manual turn %s: %g %g %g %g\n",
	      turnComplete ? "complete" : "TIMEOUT",
	      m_robot.GetJaguarPosition(m_robot.motor_left_1,"motor_left_1"),
	      m_robot.GetJaguarPosition(m_robot.motor_left_2,"motor_left_2"),
	      m_robot.GetJaguarPosition(m_robot.motor_right_1,"motor_right_1"),
	      m_robot.GetJaguarPosition(m_robot.motor_right_2,"motor_right_2"));
	    m_robot.DisableMotors();
	    turnComplete = true;  // even if it's not true
	    MyRobot::ShowState("Teleop", "Turn Complete");
	}
    }

    return false;
}
