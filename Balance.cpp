// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#include <WPILib.h>
#include "Balance.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

// Balance the bridge by using a pitch rate sensor to detect when the
// bridge starts to tilt.  We'll use one of the Analog Devices rate
// gyros supplied with the KOP.  The boards shipped in different
// years have different sensitivities and maximum ranges:
//
// ADXRS150 (2007): +/-150 degree/s max, 12.5mV/degree/s out
// ADW22304 (2008): +/- 80 degree/s max, 12.5mV/degree/s out
// ADW22307 (2010): +/-250 degree/s max,  7.0mV/degree/s out
//
// Other readily-available parts include:
//
// ADXRS401: +/- 75 degree/s max, 15.0mV/degree/s out
// ADXRS300: +/-300 degree/s max,  5.0mV/degree/s out
//
// If the ramp tilts +/- 15 degrees in 250ms (check this!), then the
// peak output from the gyro will be no more than 0.75V for the gyros
// used in the 2007-2009 KOP, and no more than about 0.42V for the
// gyro used in the 2010-2012 KOP.  The cRIO analog input module has
// a range of +/-10V with a 12-bit resolution, so a 0.42V signal is
// (0.42V/20V)*4096 = 86 counts, which should be easily detectable.

#define	APPROACH_SPEED	0.30F
#define	RAMP_SPEED	0.30F
#define	BRAKE_SPEED	-0.30F

#define	TILT_UP		25
#define	TILT_DOWN	-12
#define	RAMP_TIME	1000	// milliseconds
#define	BRAKE_TIME	225	// milliseconds

// defaults from WPILib AnalogModule class:
// static const long  kDefaultAverageBits    = 7
// static const long  kDefaultOversampleBits = 0
// static const float kDefaultSampleRate     = 50000.0
//
// Sampling at 50,000 samples/sec and averaging over 7 bits
// (128 samples) yields an effective sample rate of approx.
// 400 samples/sec or 2.5ms/sample which is a good match to the
// above parts that include a two-pole 400Hz low-pass filter.

Balance::Balance( RobotDrive& driveTrain, AnalogChannel& pitchGyro ) :
    drive( driveTrain ),
    gyro( pitchGyro ),
    approach_speed( APPROACH_SPEED ),
    ramp_speed( RAMP_SPEED ),
    brake_speed( BRAKE_SPEED ),
    tilt_up( TILT_UP ),
    tilt_down( TILT_DOWN ),
    ramp_time( RAMP_TIME * 1000UL ),
    brake_time( BRAKE_TIME * 1000UL ),
    level( 512 ), // nominal 2.5V
    tilt_min( 0 ),
    tilt_max( 0 ),
    running( false ),
    state( kInitialized ),
    reverse( false ),
    speed( 0.0F ),
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
	pref->PutInt( "Balance.tilt_up", TILT_UP );
	printf("Preferences: save TILT_UP\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.tilt_down" )) {
	pref->PutInt( "Balance.tilt_down", TILT_DOWN );
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

    printf("InitBalance: approach_speed = %f\n", approach_speed);
    printf("InitBalance: ramp_speed = %f\n", ramp_speed);
    printf("InitBalance: brake_speed = %f\n", brake_speed);

    tilt_up        = pref->GetInt( "Balance.tilt_up",   TILT_UP   );
    tilt_down      = pref->GetInt( "Balance.tilt_down", TILT_DOWN );

    printf("InitBalance: tilt_up = %d\n", tilt_up);
    printf("InitBalance: tilt_down = %d\n", tilt_down);

    // timer in microseconds, preference value in milliseconds
    ramp_time      = (unsigned long) pref->GetInt( "Balance.ramp_time",   RAMP_TIME ) * 1000UL;
    brake_time     = (unsigned long) pref->GetInt( "Balance.brake_time", BRAKE_TIME ) * 1000UL;

    printf("InitBalance: ramp_time = %lu\n", (unsigned long) ramp_time);
    printf("InitBalance: brake_time = %lu\n", (unsigned long) brake_time);

    // read the gyro's averaged output before we start moving
    // in order to compensate for various offset voltages and drift

    // gyro.SetAverageBits( AnalogModule::kDefaultAverageBits );
    // gyro.SetOversampleBits( AnalogModule::kDefaultOversampleBits );
    // gyro.GetModule()->SetSampleRate( AnalogModule::kDefaultSampleRate );

    level = gyro.GetAverageValue();
    SmartDashboard::Log( level,  "Balance.level" );

    tilt_max = 0;
    SmartDashboard::Log( tilt_max,  "Balance.tilt_max" );

    tilt_min = 0;
    SmartDashboard::Log( tilt_min,  "Balance.tilt_min" );

    speed = 0.0F;
    SmartDashboard::Log( speed, "Balance.speed" );

    state = kInitialized;
    SmartDashboard::Log( "initialized", "Balance.state" );

    running = false;
}

void Balance::Start( bool startReverse, bool startOnRamp )
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // load configuration from preferences file or SmartDashboard
    InitBalance();

    // set the initial speed and position
    if (startOnRamp) {
	state = kOnRamp;
	SmartDashboard::Log( "onRamp",  "Balance.state" );
	speed = ramp_speed;
	SmartDashboard::Log( speed,  "Balance.speed" );
    } else {
	state = kApproach;
	SmartDashboard::Log( "approach",  "Balance.state" );
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

void Balance::Run()
{
    INT16 rotation = gyro.GetAverageValue();
    INT16 tilt;

    // assume gyro is mounted so a "tilt up" rotation is negative when moving forward
    if (reverse) {
	tilt = (rotation - level);
    } else {
	tilt = (level - rotation);
    }

    // log min and max values for debugging
    if (tilt < tilt_min) {
	tilt_min = tilt;
	SmartDashboard::Log( tilt, "Balance.tilt_min" );
    }
    if (tilt > tilt_max) {
	tilt_max = tilt;
	SmartDashboard::Log( tilt, "Balance.tilt_max" );
    }

    if (IsRunning()) {
	// approaching the ramp
	if (state == kApproach) {
	    printf("kApproach: tilt %d\n", tilt);
	    if (tilt > tilt_up) {
		state = kOnRamp;
		SmartDashboard::Log( "onRamp",  "Balance.state" );
		speed = ramp_speed;
		when = (long)(GetFPGATime() + ramp_time);
	    }
	}
	// climbing the ramp
	if (state == kOnRamp) {
	    long timeleft = when - (long) GetFPGATime();
	    printf("kOnRamp: tilt %d time %ld\n", tilt, timeleft);
	    if ((timeleft <= 0) && (tilt < tilt_down)) {
		state = kBraking;
		SmartDashboard::Log( "braking",  "Balance.state" );
		speed = brake_speed;
		when = (long)(GetFPGATime() + brake_time);
	    }
	}
	// braking
	if (state == kBraking) {
	    long timeleft = when - (long) GetFPGATime();
	    printf("kBraking: tilt %d time %ld\n", tilt, timeleft);
	    if (timeleft <= 0) {
		state = kBalanced; // or so we hope
		SmartDashboard::Log( "balanced",  "Balance.state" );
		printf("kBalanced\n");
		speed = 0.0F;
	    }
	}
	// else balanced; nothing to do here

	drive.Drive( reverse ? speed : -speed, 0.0F );
    }
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

