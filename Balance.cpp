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
// ADW22304 (2008): +/-80 degree/s max, 12.5mV/degree/s out
// ADW22307 (2012): +/-250 degree/s max, 7.0mV/degree/s out
//
// Other readily-available parts include:
//
// ADXRS401: +/-75 degree/s max, 15mV/degree/s out
// ADXRS300: +/-300 degree/s max, 5.0mV/degree/s out
//
// If the ramp tilts +/- 15 degrees in 250ms (check this!), then the
// peak output from the gyro will be no more than 0.75V for the gyros
// used in the 2007 & 2008 KOP, and no more than about 0.42V for the
// gyro used in the 2012 KOP.  The cRIO analog input module has a
// range of +/-10V with a 12-bit resolution, so a 0.42V signal is
// (0.42V/20V)*4096 = 86 counts, which should be easily detectable.

#define	TILT	40

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
    level( 512 ),			// nominal 2.5V
    speed( 0.0F ),
    isOnRamp( false ),
    isBalanced( false )
{
    // gyro.SetAverageBits( AnalogModule::kDefaultAverageBits );
    // gyro.SetOversampleBits( AnalogModule::kDefaultOversampleBits );
    // gyro.GetModule()->SetSampleRate( AnalogModule::kDefaultSampleRate );
}

Balance::~Balance()
{
    drive.StopMotor();
}

void Balance::Start( float initialSpeed, bool startOnRamp )
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // read the gyro's averaged output before we start moving
    // in order to compensate for various offset voltages and drift
    level = gyro.GetAverageValue();

    speed = initialSpeed;
    isOnRamp = startOnRamp;
    isBalanced = false;

    // start moving
    Run();
}

void Balance::Stop()
{
    speed = 0.0F;
    drive.StopMotor();
}

void Balance::Run()
{
    INT16 tilt = gyro.GetAverageValue();

    if (IsRunning() && !isBalanced) {
	if (speed > 0) {
	    // moving forward
	    // assume gyro is mounted so a tilt "up" is positive
	    if (isOnRamp && (tilt - level) < -TILT) {
		isBalanced = true;
		drive.StopMotor();
	    } else {
		if (!isOnRamp && (tilt - level) > TILT) {
		    isOnRamp = true;
		    // reduce speed here?
		}
		drive.Drive( speed, 0.0F );
	    }
	} else {
	    // moving backward
	    // gyro outputs will be the opposite of above
	    if (isOnRamp && (tilt - level) > TILT) {
		isBalanced = true;
		drive.StopMotor();
	    } else {
		if (!isOnRamp && (tilt - level) < -TILT) {
		    isOnRamp = true;
		    // reduce speed here?
		}
		drive.Drive( speed, 0.0F );
	    }
	}
    }

    SmartDashboard::Log( level,       "Balance.level" );
    SmartDashboard::Log( tilt,        "Balance.tilt" );
    SmartDashboard::Log( speed,       "Balance.speed" );
    SmartDashboard::Log( isOnRamp,    "Balance.isOnRamp" );
    SmartDashboard::Log( isBalanced,  "Balance.isBalanced" );
}

float Balance::GetSpeed()
{
    return (isBalanced ? speed : 0.0F);
}

bool Balance::IsRunning()
{
    return (speed != 0.0F);
}

bool Balance::IsOnRamp()
{
    return isOnRamp;
}

bool Balance::IsBalanced()
{
    return isBalanced;
}

