// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "TurnCommand.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float TurnCommand::turnTolerance = 0.50;

TurnCommand::TurnCommand( MyRobot& theRobot ) : m_robot(theRobot)
{
    m_turnTimer.Start();
}

void TurnCommand::Start()
{
    OI& oi = m_robot.GetOI();

    m_angle = oi.TurnLeft10()  ? -10.
	    : oi.TurnRight10() ? 10.
	    : oi.TurnLeft3()   ? -3.
	    : oi.TurnRight3()  ? 3.
	    : 0.;  // can't happen

    m_turnDebug = oi.Teach();

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
	m_turnComplete = m_robot.TurnToAngle(m_angle, turnTolerance);
	if (m_turnDebug) {
	    printf("Manual turn %s: m_angle %g left %g right %g\n",
	       m_turnComplete ? "complete" : "turning", m_angle,
	       m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
	       m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
	    m_turnComplete = false;
	} else {
	    if (m_turnComplete || m_turnTimer.Get() > 0.8) {
		printf("Manual turn %s: m_angle %g left %g right %g\n",
		   m_turnComplete ? "complete" : "TIMEOUT", m_angle,
		   m_robot.GetJaguarAngle(m_robot.motor_left,"left"),
		   m_robot.GetJaguarAngle(m_robot.motor_right,"right"));
		m_robot.DisableMotors();
		MyRobot::ShowState("Turn", "Complete");
		m_turnComplete = true;  // force termination
	    }
	}
    }

    return m_turnComplete;
}
