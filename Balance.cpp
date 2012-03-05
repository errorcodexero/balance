// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "Balance.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

// Balance the bridge by using a pitch rate sensor to detect when the
// bridge starts to tilt.  We'll use one of the Analog Devices rate
// gyros supplied with the KOP or a similar COTS device from another
// vendor.  The boards shipped in the KOP in different years have
// varying sensitivities and maximum ranges:
//
// ADXRS150 (2007): +/-150 degree/s max, 12.5mV/degree/s out
// ADW22304 (2008): +/- 80 degree/s max, 12.5mV/degree/s out
// ADW22307 (2010): +/-250 degree/s max,  7.0mV/degree/s out
//
// Other readily-available parts include:
//
// ADXRS401: +/- 75 degree/s max, 15.0mV/degree/s out
// ADXRS300: +/-300 degree/s max,  5.0mV/degree/s out
// LPY503AL: +/- 30 degree/s max,  8.3mV/degree/s (1X) or
//				  33.3mV/degree/s (4X) out
//
// If the ramp tilts +/- 15 degrees in 250ms (check this!), then the
// peak output from the gyro will be no more than 0.75V for the gyros
// used in the 2007-2009 KOP, and no more than about 0.42V for the
// gyro used in the 2010-2012 KOP.  The cRIO analog input module has
// a range of +/-10V with a 12-bit resolution, so a 0.42V signal is
// (0.42V/20V)*4096 = 86 counts, which should give us an acceptable
// signal over electrical noise.  Mechanical noise as the robot tilts
// back and forth in the 6-wheel drive may be a problem!

#define	SENSITIVITY	0.0333	// 33.3mV/degree/s

#define	APPROACH_SPEED	0.50F
#define	RAMP_SPEED	0.50F
#define	BRAKE_SPEED	0.0F

#define	TILT_UP		10.0F
#define	TILT_DOWN	8.0F
#define	RAMP_TIME	3000	// milliseconds
#define	BRAKE_TIME	0	// milliseconds

// defaults from WPILib AnalogModule class:
// static const long  kTimebase              = 40000000;  // fixed 40 MHz clock
// static const long  kDefaultOversampleBits = 0;
// static const long  kDefaultAverageBits    = 10;
// static const float kDefaultSampleRate     = 50000.0;   // 20uS per raw sample
//
// defaults from WPILib Gyro class:
// static const UINT32 kOversampleBits       = 10;        // Gyro uses oversampling for accuracy
// static const UINT32 kAverageBits          = 0;
// static const float kSamplesPerSecond      = 50.0;      // 20mS after oversampling
// static const float kCalibrationSampleTime = 5.0;
// static const float kDefaultVoltsPerDegreePerSecond = 0.007;  // can be changed

Balance::Balance( RobotDrive& driveTrain, Gyro& pitchGyro ) :
    drive( driveTrain ),
    gyro( pitchGyro ),
    approach_speed( APPROACH_SPEED ),
    ramp_speed( RAMP_SPEED ),
    brake_speed( BRAKE_SPEED ),
    tilt_up( TILT_UP ),
    tilt_down( TILT_DOWN ),
    ramp_time( RAMP_TIME * 1000UL ),
    brake_time( BRAKE_TIME * 1000UL ),
    running( false ),
    state( kInitialized ),
    reverse( false ),
    speed( 0.0F ),
    tilt( 0.0F ),
    when( 0 )
{
    Preferences *pref = Preferences::GetInstance();
    bool saveNeeded = false;
    
    printf("In Balance constructor, pref = 0x%p\n", pref);
    if (!pref->ContainsKey( "Balance.approach_speed" )) {
	pref->PutDouble( "Balance.approach_speed", APPROACH_SPEED );
	printf("Preferences: save APPROACH_SPEED\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.ramp_speed" )) {
	pref->PutDouble( "Balance.ramp_speed", RAMP_SPEED );
	printf("Preferences: save RAMP_SPEED\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.brake_speed" )) {
	pref->PutDouble( "Balance.brake_speed", BRAKE_SPEED );
	printf("Preferences: save BRAKE_SPEED\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.tilt_up" )) {
	pref->PutDouble( "Balance.tilt_up", TILT_UP );
	printf("Preferences: save TILT_UP\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.tilt_down" )) {
	pref->PutDouble( "Balance.tilt_down", TILT_DOWN );
	printf("Preferences: save TILT_DOWN\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.ramp_time" )) {
	// timer in microseconds, preference value in milliseconds
	pref->PutInt( "Balance.ramp_time", RAMP_TIME );
	printf("Preferences: save RAMP_TIME\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.brake_time" )) {
	// timer in microseconds, preference value in milliseconds
	pref->PutInt( "Balance.brake_time", BRAKE_TIME );
	printf("Preferences: save BRAKE_TIME\n");
	saveNeeded = true;
    }
    if (saveNeeded) {
	pref->Save();
	printf("Preferences: saved\n");
    }

    InitBalance();
}

Balance::~Balance()
{
    drive.Drive( 0.0F, 0.0F );
}


void Balance::InitBalance()
{
    Preferences *pref = Preferences::GetInstance();

    approach_speed = pref->GetDouble( "Balance.approach_speed", APPROACH_SPEED );
    ramp_speed     = pref->GetDouble( "Balance.ramp_speed",     RAMP_SPEED     );
    brake_speed    = pref->GetDouble( "Balance.brake_speed",    BRAKE_SPEED    );

    printf("InitBalance: approach_speed = %4.2f\n", approach_speed);
    printf("InitBalance: ramp_speed = %4.2f\n", ramp_speed);
    printf("InitBalance: brake_speed = %4.2f\n", brake_speed);

    tilt_up        = pref->GetDouble( "Balance.tilt_up",   TILT_UP   );
    tilt_down      = pref->GetDouble( "Balance.tilt_down", TILT_DOWN );

    printf("InitBalance: tilt_up = %4.2f\n", tilt_up);
    printf("InitBalance: tilt_down = %4.2f\n", tilt_down);

    // timer in microseconds, preference value in milliseconds
    ramp_time      = (unsigned long) pref->GetInt( "Balance.ramp_time",   RAMP_TIME ) * 1000UL;
    brake_time     = (unsigned long) pref->GetInt( "Balance.brake_time", BRAKE_TIME ) * 1000UL;

    printf("InitBalance: ramp_time = %lu\n", (unsigned long) ramp_time / 1000UL);
    printf("InitBalance: brake_time = %lu\n", (unsigned long) brake_time / 1000UL);

    // set the gyro sensitivity so results will be in degrees
    gyro.SetSensitivity( SENSITIVITY );

    // reset the gyro to "level"
    gyro.Reset();

    speed = 0.0F;
    SmartDashboard::Log( speed, "Balance.speed" );

    tilt = 0.0F;
    SmartDashboard::Log( tilt, "Balance.tilt" );

    state = kInitialized;
    MyRobot::ShowState("Teleop","Balance Init");

    running = false;
}

void Balance::Start( bool startReverse, bool startOnRamp )
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // set the initial speed and position
    if (startOnRamp) {
	state = kOnRamp;
	MyRobot::ShowState("Teleop", "Balance On Ramp");
	speed = ramp_speed;
	SmartDashboard::Log( speed,  "Balance.speed" );
    } else {
	state = kApproach;
	MyRobot::ShowState("Teleop", "Balance Approach");
	speed = approach_speed;
	SmartDashboard::Log( speed,  "Balance.speed" );
    }
    reverse = startReverse;

    // start moving
    running = true;
}

void Balance::Stop()
{
    running = false;
    speed = 0.0;
    SmartDashboard::Log( speed,  "Balance.speed" );
    drive.Drive( 0.0F, 0.0F );
}

bool Balance::Run()
{
    // assume gyro is mounted so a "tilt up" rotation is negative when moving forward
    tilt = gyro.GetAngle();
    if (reverse) {
	tilt = -tilt;
    }
    SmartDashboard::Log( tilt,  "Balance.tilt" );

    if (IsRunning()) {
	// approaching the ramp
	if (state == kApproach) {
	    printf("kApproach: tilt %4.2f\n", tilt);
	    if (tilt > tilt_up) {
		state = kOnRamp;
		MyRobot::ShowState("Teleop", "Balance On Ramp");
		speed = ramp_speed;
		when = (long)(GetFPGATime() + ramp_time);
	    }
	}
	// climbing the ramp
	if (state == kOnRamp) {
	    long timeleft = when - (long) GetFPGATime();
	    printf("kOnRamp: tilt %4.2f time %ld\n", tilt, timeleft);
	    if ((timeleft <= 0) && (tilt < tilt_down)) {
		state = kBraking;
		MyRobot::ShowState("Teleop", "Balance Braking");
		speed = brake_speed;
		when = (long)(GetFPGATime() + brake_time);
	    }
	}
	// braking
	if (state == kBraking) {
	    long timeleft = when - (long) GetFPGATime();
	    printf("kBraking: tilt %4.2f time %ld\n", tilt, timeleft);
	    if (timeleft <= 0) {
		state = kBalanced; // or so we hope
		MyRobot::ShowState("Teleop", "Balance Balanced");
		printf("kBalanced\n");
		speed = 0.0F;
	    }
	}
	// else balanced; nothing to do here
	drive.Drive( reverse ? speed : -speed, 0.0F );
    }

    return false;	// stay in this state until interrupted by driver
}

float Balance::GetSpeed()
{
    return speed;
}

bool Balance::IsRunning()
{
    return (running);
}

bool Balance::IsOnRamp()
{
    return (state != kApproach);
}

bool Balance::IsBraking()
{
    return (state == kBraking);
}

bool Balance::IsBalanced()
{
    return (state == kBalanced);
}

