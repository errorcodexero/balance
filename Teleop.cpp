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
    bool driverButtonDown = false;
    bool gunnerButtonDown = false;
    double turnAngle = 0.;
    Target::TargetID targetID = Target::kCenter;
    ShootCommand::FireControl shootControl = ShootCommand::kDone;

    // Check the gunner controls, then the driver controls.
    // That gives the driver priority if they're both holding buttons down.
    if (oi.TargetTop()) {
	gunnerButtonDown = true;
	newMode = kShoot;
	targetID = Target::kTop;
	shootControl = ShootCommand::kDone;
    } else if (oi.TargetLeft()) {
	gunnerButtonDown = true;
	newMode = kShoot;
	targetID = Target::kLeft;
	shootControl = ShootCommand::kDone;
    } else if (oi.TargetRight()) {
	gunnerButtonDown = true;
	newMode = kShoot;
	targetID = Target::kRight;
	shootControl = ShootCommand::kDone;
    } else if (oi.TargetBottom()) {
	gunnerButtonDown = true;
	newMode = kShoot;
	targetID = Target::kTop;
	shootControl = ShootCommand::kSpinUp;
    }

    if (oi.TurnLeft10()) {
	driverButtonDown = true;
	newMode = kTurn;
	turnAngle = -10.;
    } else if (oi.TurnRight10()) {
	driverButtonDown = true;
	newMode = kTurn;
	turnAngle = 10.;
    } else if (oi.TurnLeft3()) {
	driverButtonDown = true;
	newMode = kTurn;
	turnAngle = -3.;
    } else if (oi.TurnRight3()) {
	driverButtonDown = true;
	newMode = kTurn;
	turnAngle = 3.;
    } else if (oi.TurnAuto()) {
	driverButtonDown = true;
	newMode = kAim;
	targetID = Target::kTop;
	shootControl = ShootCommand::kSpinUp;
    }

    if (newMode != m_driveMode) {
	switch (m_driveMode) {
	case kManual:
	    m_driveCommand.Stop();
	    break;
	case kTurn:
	    m_turnCommand.Stop();
	    break;
	case kAim:
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
	case kAim:
	case kShoot:
	    m_shootCommand.Start(targetID, shootControl);
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
    case kAim:
	done = m_shootCommand.Run();
	// special case for driver-initiated aiming:
	// Allow the driver to interrupt the action
	// by releasing the button early.
	if (!driverButtonDown) {
	    done = true;
	}
	break;

    case kShoot:
	done = m_shootCommand.Run();
	// special case for auto aiming/shooting:
	// Allow the gunner to interrupt the action
	// by turning the shooter off.
	if (oi.Shooter() == 0) {
	    done = true;
	}
	break;
    }

    if (done && !driverButtonDown && !gunnerButtonDown) {
	// special actions completed and all buttons released:
	// return to default (manual control) drive mode
	switch (m_driveMode) {
	case kManual:
	    m_driveCommand.Stop();
	    break;
	case kTurn:
	    m_turnCommand.Stop();
	    break;
	case kAim:
	case kShoot:
	    m_shootCommand.Stop();
	    break;
	}
	m_driveCommand.Start();
	m_driveMode = kManual;
    }
}

void MyRobot::TeleopContinuous()
{
    taskDelay(1);
}
