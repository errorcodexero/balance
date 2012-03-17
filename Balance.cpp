// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include <math.h>
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
#define	RAMP_SPEED	0.30F	// drive this fast (as percentage of full speed) on the ramp
#define	TILT_LIMIT	5.0F	// what we consider "balanced"
#define	TILT_MAX	15.0F	// 15 degree ramp

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

Balance::Balance( MyRobot& theRobot ) :
    m_robot( theRobot ),
    ramp_speed( RAMP_SPEED ),
    tilt_limit( TILT_LIMIT ),
    running( false ),
    speed( 0.0F ),
    tilt( 0.0F )
{
    Preferences *pref = Preferences::GetInstance();
    bool saveNeeded = false;
    
    // set the gyro sensitivity so results will be in degrees
    m_robot.pitch.SetSensitivity( SENSITIVITY );

    // reset the gyro to "level"
    m_robot.pitch.Reset();

    printf("In Balance constructor, pref = 0x%p\n", pref);
    if (!pref->ContainsKey( "Balance.ramp_speed" )) {
	pref->PutDouble( "Balance.ramp_speed", RAMP_SPEED );
	printf("Preferences: save RAMP_SPEED\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Balance.tilt_limit" )) {
	pref->PutDouble( "Balance.tilt_limit", TILT_LIMIT );
	printf("Preferences: save TILT_LIMIT\n");
	saveNeeded = true;
    }
    if (saveNeeded) {
	pref->Save();
	printf("Preferences: saved\n");
    }

    InitBalance();
}

void Balance::InitBalance()
{
    Preferences *pref = Preferences::GetInstance();

    ramp_speed = pref->GetDouble( "Balance.ramp_speed", RAMP_SPEED );
    printf("InitBalance: ramp_speed = %g\n", ramp_speed);

    tilt_limit = pref->GetDouble( "Balance.tilt_limit", TILT_LIMIT );
    printf("InitBalance: tilt_limit = %g\n", tilt_limit);

    tilt = 0.0F;
    SmartDashboard::Log( tilt, "Balance.tilt" );

    speed = 0.0F;

    running = false;
}

Balance::~Balance()
{
    m_robot.drive.Drive( 0.0F, 0.0F );
}

void Balance::Start()
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // enable position control
    m_robot.EnablePositionControl();

    // start moving
    running = true;
    Run();
}

void Balance::Stop()
{
    running = false;
    speed = 0.0;
    m_robot.DisableMotors();
}

bool Balance::Run()
{
    // assume gyro is mounted so a "tilt up" rotation is positive when moving forward
    tilt = m_robot.pitch.GetAngle();
    SmartDashboard::Log( tilt,  "Balance.tilt" );

    if (IsRunning()) {
	speed = IsBalanced() ? 0.0 : ((tilt / TILT_MAX) * ramp_speed);
	m_robot.drive.Drive(speed, 0.0F);
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

bool Balance::IsBalanced()
{
    return (fabs(tilt) < tilt_limit);
}

