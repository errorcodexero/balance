// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void MyRobot::TeleopInit()
{
    Safe();
    compressor.Start();
    m_turnCommand.Stop();
    m_shootCommand.Stop();
    m_autoCommand.Stop();
    shooter.InitShooter();

    m_driveMode = kManual;
    m_driveCommand.Start();
}

void MyRobot::TeleopPeriodic()
{
    OI& oi = GetOI();
    DriveMode newMode = m_driveMode;
    bool buttonDown = false;
    double turnAngle = 0.;

    if (oi.TurnLeft10()) {
	buttonDown = true;
	newMode = kTurn;
	turnAngle = -10.;
    } else if (oi.TurnRight10()) {
	buttonDown = true;
	newMode = kTurn;
	turnAngle = 10.;
    } else if (oi.TurnLeft3()) {
	buttonDown = true;
	newMode = kTurn;
	turnAngle = -3.;
    } else if (oi.TurnRight3()) {
	buttonDown = true;
	newMode = kTurn;
	turnAngle = 3.;
    } else if (oi.TurnAuto()) {
	buttonDown = true;
	newMode = kShoot;
    }

    if (newMode != m_driveMode) {
	switch (m_driveMode) {
	case kManual:
	    m_driveCommand.Stop();
	    break;
	case kTurn:
	    m_turnCommand.Stop();
	    break;
	case kShoot:
	    m_shootCommand.Stop();
	    break;
	}

	switch (newMode) {
	case kManual:
	    m_driveCommand.Start();
	    break;
	case kTurn:
	    m_turnCommand.Start(turnAngle);
	    break;
	case kShoot:
	    m_shootCommand.Start(Target::kTop, ShootCommand::kSpinUp);
	    break;
	}

	m_driveMode = newMode;
    }

    bool done = false;
    switch (m_driveMode) {
    case kManual:
	done = m_driveCommand.Run();
	break;
    case kTurn:
	done = m_turnCommand.Run();
	break;
    case kShoot:
	done = m_shootCommand.Run();
	break;
    }

    if (done && !buttonDown) {
	// button released, stop this mode
	switch (m_driveMode) {
	case kManual:
	    m_driveCommand.Stop();
	    break;
	case kTurn:
	    m_turnCommand.Stop();
	    break;
	case kShoot:
	    m_shootCommand.Stop();
	    break;
	}

	// return to manual control
	m_driveCommand.Start();
	m_driveMode = kManual;
    }
}

void MyRobot::TeleopContinuous()
{
    taskDelay(1);
}
