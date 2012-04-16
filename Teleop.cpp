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

    driveMode = kManual;
    m_driveCommand.Start();
}

void MyRobot::TeleopPeriodic()
{
    OI& oi = GetOI();
    DriveMode newMode;
    bool buttonDown;

    if (oi.TurnLeft10() || oi.TurnRight10() || oi.TurnLeft3() || oi.TurnRight3()) {
	buttonDown = true;
	newMode = kTurn;
    } else if (oi.TargetTop() || oi.TargetLeft() || oi.TargetRight() || oi.TargetBottom()) {
	buttonDown = true;
	newMode = kShoot;
    } else {
	buttonDown = false;
	newMode = driveMode;
    }

    if (driveMode != newMode) {
	switch (driveMode) {
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
	    m_turnCommand.Start();
	    break;
	case kShoot:
	    m_shootCommand.Start();
	    break;
	}

	driveMode = newMode;
    }

    bool done = false;
    switch (driveMode) {
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
	// stop this mode
	switch (driveMode) {
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
	driveMode = kManual;
    }
}

void MyRobot::TeleopContinuous()
{
    taskDelay(1);
}
