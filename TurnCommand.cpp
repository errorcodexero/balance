// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "TurnCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float turnTolerance = 0.80;

TurnCommand::TurnCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    turnTimer.Start();
}

void TurnCommand::Start()
{
    m_robot.EnablePositionControl();
    turnTimer.Reset();
    turnComplete = false;

    printf("Start turn\n");
    MyRobot::ShowState("Turn", "Start");
}

void TurnCommand::Stop()
{
    printf("Stop turn\n");
    m_robot.DisableMotors();
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

	turnComplete = m_robot.TurnToAngle(angle, tolerance);

	if (turnComplete || turnTimer.HasPeriodPassed(2.0)) {
	    // printf("Manual turn %s: %g %g %g %g\n",
	    //   turnComplete ? "complete" : "TIMEOUT",
	    //   m_robot.GetJaguarAngle(m_robot.motor_left_1,"left_1"),
	    //   m_robot.GetJaguarAngle(m_robot.motor_left_2,"left_2"),
	    //   m_robot.GetJaguarAngle(m_robot.motor_right_1,"right_1"),
	    //   m_robot.GetJaguarAngle(m_robot.motor_right_2,"right_2"));
	    m_robot.DisableMotors();
	    turnComplete = true;  // even if it's not true
	    MyRobot::ShowState("Turn", "Complete");
	}
    }

    return false;
}
