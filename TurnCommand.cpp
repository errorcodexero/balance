// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "TurnCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float TurnCommand::turnTimeout = 2.0;

TurnCommand::TurnCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    m_turnTimer.Start();
}

void TurnCommand::Start( float angle )
{
    m_angle = angle;

    m_robot.EnablePositionControl();
    m_turnTimer.Reset();
    m_turnComplete = false;

    printf("Start turn %g\n", m_angle);
    MyRobot::ShowState("Turn", "Start");
}

void TurnCommand::Stop()
{
    printf("Stop turn\n");
    m_robot.DisableMotors();
}

bool TurnCommand::Run()
{
    if (!m_turnComplete) {
	m_turnComplete = m_robot.TurnToAngle(m_angle);
	if (m_turnComplete || (m_turnTimer.Get() > turnTimeout)) {
	    if (m_robot.GetOI().Teach()) {
		printf("Manual turn %s: m_angle %g left %g right %g\n",
		   m_turnComplete ? "complete" : "TIMEOUT", m_angle,
		   m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
		   m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
	    }
	    m_robot.DisableMotors();
	    MyRobot::ShowState("Turn", "Complete");
	    m_turnComplete = true;  // force termination
	}
    }

    return m_turnComplete;
}
