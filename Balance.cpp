// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

// WPILib Includes
#include <WPILib.h>

// Our Includes
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

#define	APPROACH_SPEED	0.3F
#define	RAMP_SPEED	0.5F
#define	BRAKE_SPEED	-0.1F

#define	TILT_UP		30
#define	TILT_DOWN	-30
#define	BRAKE_TIME	50		// milliseconds

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
    brake_time( BRAKE_TIME ),
    level( 512 ), // nominal 2.5V
    tilt_min( 0 ),
    tilt_max( 0 ),
    started( false ),
    state( kApproach ),
    reverse( false ),
    speed( 0.0F ),
    when( 0UL )
{
    // gyro.SetAverageBits( AnalogModule::kDefaultAverageBits );
    // gyro.SetOversampleBits( AnalogModule::kDefaultOversampleBits );
    // gyro.GetModule()->SetSampleRate( AnalogModule::kDefaultSampleRate );
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

    tilt_up        = pref->GetInt( "Balance.tilt_up",   TILT_UP   );
    tilt_down      = pref->GetInt( "Balance.tilt_down", TILT_DOWN );

    // timer in microseconds, preference value in milliseconds
    brake_time     = pref->GetInt( "Balance.brake_time", BRAKE_TIME ) * 1000UL;
}

void Balance::SavePreferences()
{
    InitBalance();

    Preferences *pref = Preferences::GetInstance();

    pref->PutDouble( "Balance.approach_speed", approach_speed );
    pref->PutDouble( "Balance.ramp_speed",     ramp_speed     );
    pref->PutDouble( "Balance.brake_speed",    brake_speed    );

    pref->PutInt( "Balance.tilt_up",   tilt_up   );
    pref->PutInt( "Balance.tilt_down", tilt_down );

    // timer in microseconds, preference value in milliseconds
    pref->PutInt( "Balance.brake_time", (int) (brake_time / 1000UL) );

    pref->Save();
}

void Balance::Start( bool startReverse, bool startOnRamp )
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // load configuration from preferences file or SmartDashboard
    InitBalance();

    // read the gyro's averaged output before we start moving
    // in order to compensate for various offset voltages and drift
    level = gyro.GetAverageValue();
    SmartDashboard::Log( level,  "Balance.level" );

    tilt_max = 0;
    SmartDashboard::Log( tilt_max,  "Balance.tilt_max" );
    tilt_min = 0;
    SmartDashboard::Log( tilt_min,  "Balance.tilt_min" );

    // set the initial speed and position
    reverse = startReverse;
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

    // start moving
    started = true;
    Run();
}

void Balance::Stop()
{
    started = false;
    speed = 0.0;
    SmartDashboard::Log( speed,  "Balance.speed" );
    drive.Drive( 0.0F, 0.0F );
}

void Balance::Run()
{
    if (IsRunning()) {
	INT16 rotation = gyro.GetAverageValue();
	INT16 tilt;
	// assume gyro is mounted so a "tilt up" rotation is positive when moving forward
	if (reverse) {
	    tilt = (level - rotation);
	} else {
	    tilt = (rotation - level);
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
	// approaching the ramp
	if (state == kApproach) {
	    if (tilt > tilt_up) {
		state = kOnRamp;
		speed = RAMP_SPEED;
	    }
	}
	// climbing the ramp
	if (state == kOnRamp) {
	    if (tilt < tilt_down) {
		state = kBraking;
		speed = brake_speed;
		when = GetFPGATime() + BRAKE_TIME;
	    }
	}
	// braking
	if (state == kBraking) {
	    if ((INT32)(GetFPGATime() - when) > 0) {
		state = kBalanced; // or so we hope
		speed = 0.0F;
	    }
	}
	// else balanced; nothing to do here
    }
    drive.Drive( reverse ? -speed : speed, 0.0F );
}

float Balance::GetSpeed()
{
    return speed;
}

bool Balance::IsRunning()
{
    return (started);
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

