// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
#include "AutoCommand.h"
#include "MyRobot.h"
#include "Target.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

const float AutoCommand::cameraWarmup = 1.50;
const float AutoCommand::aimTolerance = 0.50;
const float AutoCommand::holdTimeout  = 0.50;
const float AutoCommand::turnTimeLimit = 2.0;
const float AutoCommand::shotTimeLimit = 8.0;
const float AutoCommand::driveScale = 0.027;	// determined by experiment
const float AutoCommand::largeTurnTolerance = 1.2;

const char *AutoCommand::StateName( AutoState state )
{
    const char *stateName = NULL;
    
    switch (state) {
    case kWait:		stateName = "Wait";	break;
    case kLights:	stateName = "Lights";	break;
    case kCamera:	stateName = "Camera";	break;
    case kAction:	stateName = "Action";	break;
    case kHold:		stateName = "Hold";	break;
    case kShooting:	stateName = "Shooting";	break;
    case kTurning:	stateName = "Turning";	break;
    case kDriving:	stateName = "Driving";	break;
    case kStopped:	stateName = "Stopped";	break;
    default:		stateName = "unknown";	break;
    }

    return stateName;
}

AutoCommand::AutoCommand( MyRobot& theRobot ) :
    m_robot(theRobot),
    m_selector1(3),
    m_selector2(4)
{
    Init();
}

void AutoCommand::Init()
{
    m_autoSequence1 = 0;
    m_autoSequence2 = 0;

    m_initialWait   = 0.;
    m_shotAngle     = 0.;
    m_shotDistance  = 0.;
    m_driveAngle    = 0.;
    m_driveDistance = 0.;

    m_autoState = kStopped;
    m_shotCount = 0;
    m_autoTimer.Start();
}

void AutoCommand::Stop()
{
    m_robot.DisableMotors();
    m_robot.shooter.Stop();
    m_autoState = kStopped;
}

void AutoCommand::Start()
{
    Init();
    m_robot.DisableMotors();
    m_robot.shooter.InitShooter();

    // get autonomous program from analog channel
    // autoSequence1 controls startup delay and shooting
    // autoSequence2 controls initial position and driving

    m_autoSequence1 = (int)(m_selector1.GetVoltage() * 9.0 / 5.0 + 0.5);
    m_autoSequence2 = (int)(m_selector2.GetVoltage() * 9.0 / 5.0 + 0.5);
    printf("Starting auto sequence %d/%d\n", m_autoSequence1, m_autoSequence2);

    // autoSequence1 positions 0..4 shoot using camera to adjust aim
    // autoSequence1 positions 5..9 shoot using initial field position
    // initial wait is 0..4 seconds
    m_initialWait = (float)((m_autoSequence1 < 5) ? (m_autoSequence1) : (m_autoSequence1 - 5));

    m_autoState = kWait;
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

bool AutoCommand::Run()
{
    m_robot.shooter.Run();

    switch (m_autoState) {
    case kWait:		StateWait();	break;
    case kLights:	StateLights();	break;
    case kCamera:	StateCamera();	break;
    case kAction:	StateAction();	break;
    case kHold:		StateHold();	break;
    case kShooting:	StateShooting(); break;
    case kTurning:	StateTurning();	break;
    case kDriving:	StateDriving();	break;
    case kStopped:
    default:		return true;
    }
    return false;
}

void AutoCommand::StateWait()
{
    if (m_autoTimer.Get() < m_initialWait) {
	return;
    }

    switch (m_autoSequence2) {
    default: // can't happen
    case 0: // unknown position, no bridge
	m_shotAngle = 0.0;
	m_shotDistance = 0.0;
	m_driveAngle = 0.0;
	m_driveDistance = 0.0;
	break;

    case 1: // left side, no bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = -19.5;	// -21.4;
	m_driveDistance = 0.0;
	break;

    case 2: // center, no bridge
	m_shotAngle = 0.0;
	m_shotDistance = 124.0;
	m_driveAngle = 0.0;
	m_driveDistance = 0.0;
	break;

    case 3: // right side, no bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = 19.5;	// 21.4;
	m_driveDistance = 0.0;
	break;

    case 4: // left side, center bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = -(19.5 + 20.6);	// -(21.4 + 24.8);
	m_driveDistance = -150.;
	break;

    case 5: // center, center bridge
	m_shotAngle = 0.0;
	m_shotDistance = 124.0;
	m_driveAngle = 0.0;
	m_driveDistance = -136.;
	break;

    case 6: // right side, center bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = (19.5 + 20.6);	// (21.4 + 24.8);
	m_driveDistance = -150.;
	break;

    case 7: // left side, right bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = -(21.4 + 55.5);
	m_driveDistance = -241.;
	break;

    case 8: // center, right bridge
	m_shotAngle = 0.0;
	m_shotDistance = 124.0;
	m_driveAngle = -44.9;
	m_driveDistance = -193.;
	break;

    case 9: // right side, right bridge
	m_shotAngle = 0.0;
	m_shotDistance = 120.0;
	m_driveAngle = 21.4 - 28.2;
	m_driveDistance = -155.;
	break;
    }

    // autoSequence1 positions 0..4 shoot using camera to adjust aim
    // autoSequence1 positions 5..9 shoot using initial field position

    if (m_autoSequence1 < 5) {
	m_robot.illuminator.Set( Relay::kOn );
	m_autoState = kLights;
    } else if (m_shotDistance != 0.0) {
	printf("Starting shooter, distance = %g\n", m_shotDistance);
	m_robot.shooter.SetTarget(2, m_shotDistance);
	m_robot.shooter.Start();
	m_shotCount = 0;
	m_autoState = kShooting;
    } else if (m_driveAngle != 0.0) {
	printf("Turning...\n");
	m_robot.EnablePositionControl();
	m_autoState = kTurning;
    } else if (m_driveDistance != 0.0) {
	printf("Preparing to move...\n");
	m_robot.EnableSpeedControl();
	m_robot.cowcatcher.Set(true);
	// m_robot.pickup.Forward();
	m_autoState = kDriving;
    } else {
	m_autoState = kStopped;
    }
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateLights()
{
    // must wait a while for the camera to adjust to the lighting change
    if (m_autoTimer.Get() < cameraWarmup) {
	return;
    }

    m_robot.target.StartAcquisition();
    m_autoState = kCamera;
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateCamera()
{
    if (!m_robot.target.ProcessingComplete()) {
	return;
    }
    if (m_robot.m_oi.Teach()) {
	printf("Target image processing complete\n");
    }
    if (m_robot.target.TargetsFound()) {
	Target::TargetLocation targetLocation = m_robot.target.GetTargetLocation(Target::kTop);
	if (m_robot.m_oi.Teach()) {
	    printf("Target \"%s\" center visible %d, angle %g, distance %g\n",
		Target::TargetName(targetLocation.id),
		targetLocation.valid,
		targetLocation.angle,
		targetLocation.distance);
	}
	if (targetLocation.valid && (fabs(targetLocation.angle) < aimTolerance)) {
	    m_shotDistance = targetLocation.distance;
	    if (m_robot.m_oi.Teach()) {
		printf("Starting shooter, distance = %g\n", m_shotDistance);
	    }
	    m_robot.illuminator.Set(Relay::kOff);
	    m_robot.shooter.SetTarget(2, m_shotDistance);
	    m_robot.shooter.Start();
	    m_shotCount = 0;
	    m_autoState = kShooting;
	} else {
	    m_shotAngle = targetLocation.angle;
	    if (m_robot.m_oi.Teach()) {
		printf("Turning %g degrees\n", m_shotAngle);
	    }
	    m_robot.EnablePositionControl();
	    m_autoState = kAction;
	}
    } else {
	if (m_robot.m_oi.Teach()) {
	    printf("No targets visible - can't happen!\n");
	}
	m_robot.illuminator.Set(Relay::kOff);
	if (m_shotDistance == 0.0) {
	    // Something's really wrong.  Give up.
	    // TBD: we could perhaps turn and drive based on initial field position
	    m_autoState = kStopped;
	} else {
	    if (m_robot.m_oi.Teach()) {
		printf("Shooting based on initial field position.\n");
		printf("Starting shooter, distance = %g\n", m_shotDistance);
	    }
	    m_robot.shooter.SetTarget(2, m_shotDistance);
	    m_robot.shooter.Start();
	    m_shotCount = 0;
	    m_autoState = kShooting;
	}
    }
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateAction()
{
    bool turnComplete = m_robot.TurnToAngle(m_shotAngle);

    if (!turnComplete && (m_autoTimer.Get() < turnTimeLimit)) {
	return;	// just wait
    }

    m_robot.DisableMotors();

    if (turnComplete) {
	printf("Turn complete...\n");
	m_autoState = kHold;
    } else {
	// turn failed, so it isn't safe to drive
	printf("Turn failed - abort!\n");
	m_autoState = kStopped;
    }
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateHold()
{
    if (m_autoTimer.Get() < holdTimeout) {
	return;
    }
    m_robot.target.StartAcquisition();
    m_autoState = kCamera;
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateShooting()
{
    bool shooterReady = m_robot.shooter.IsReady();

    if (!shooterReady && (m_autoTimer.Get() < shotTimeLimit)) {
	return;	// just wait
    }

    if (shooterReady) {
	if (m_robot.m_oi.Teach()) {
	    printf("Shoot!\n");
	}
	if (++m_shotCount <= 2) {
	    m_robot.shooter.Shoot();
	    if (m_robot.m_oi.Teach()) {
		printf("waiting for next shot...\n");
	    }
	    return;
	}
	if (m_robot.m_oi.Teach()) {
	    printf("Done shooting\n");
	}
    } else {
	printf("Shooter TIMEOUT\n");
    }

    m_robot.shooter.Stop();
    if (m_driveAngle != 0.0) {
	printf("Turning...\n");
	m_robot.EnablePositionControl();
	m_autoState = kTurning;
    } else if (m_driveDistance != 0.0) {
	printf("Preparing to move...\n");
	m_robot.EnableSpeedControl();
	m_robot.cowcatcher.Set(true);
	// m_robot.pickup.Forward();
	m_autoState = kDriving;
    } else {
	m_autoState = kStopped;
    }
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateTurning()
{
    bool turnComplete = m_robot.TurnToAngle(m_driveAngle, largeTurnTolerance);

    if (!turnComplete && (m_autoTimer.Get() < turnTimeLimit)) {
	return;	// just wait
    }

    if (turnComplete) {
	if (m_robot.m_oi.Teach()) {
	    printf("Turn complete...\n");
	}
	m_robot.EnableSpeedControl();
	m_robot.cowcatcher.Set(true);
	// m_robot.pickup.Forward();
	m_autoState = kDriving;
    } else {
	// turn failed, so it isn't safe to drive
	printf("Turn failed - abort!\n");
	m_robot.DisableMotors();
	m_autoState = kStopped;
    }
    m_autoTimer.Reset();
    MyRobot::ShowState("Autonomous", StateName(m_autoState));
}

void AutoCommand::StateDriving()
{
    double t = m_autoTimer.Get();
    // ramp up to full drive speed
    double s = (t < 0.8) ? t : 0.8;
    m_robot.drive.ArcadeDrive(-s, 0., false);
    if (t > -driveScale * m_driveDistance) {
	// we're done
	printf("Driving complete!\n");
	m_robot.DisableMotors();
	m_autoState = kStopped;
	MyRobot::ShowState("Autonomous", StateName(m_autoState));
    }
}

