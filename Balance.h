// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#ifndef _BALANCE_H_
#define _BALANCE_H_

#include <WPILib.h>

class Balance
{
private:
    RobotDrive& drive;
    AnalogChannel& gyro;

    // operating parameters configured from preferences/SmartDashboard
    float approach_speed;
    float ramp_speed;
    float brake_speed;
    INT16 tilt_up;
    INT16 tilt_down;
    UINT32 brake_time;

    // runtime state
    INT16 level;		// average gyro output when stopped
    INT16 tilt_min, tilt_max;	// instrumentation for debugging
    bool running;		// are we in control?
    enum { kInitialized, kApproach, kOnRamp, kBraking, kBalanced } state; // operating state
    bool reverse;		// running in reverse
    float speed;		// current motor speed
    UINT32 when;		// timestamp for braking   

public:
    Balance( RobotDrive& driveTrain, AnalogChannel& pitchGyro );
    ~Balance();
    
    void InitBalance();
    void Start( bool reverse, bool startOnRamp );
    void Stop();

    void Run();

    float GetSpeed();
    bool IsRunning();
    bool IsOnRamp();
    bool IsBraking();
    bool IsBalanced();
};

#endif // _BALANCE_H_
