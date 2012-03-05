// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _BALANCE_H_
#define _BALANCE_H_

#include <WPILib.h>

class Balance
{
private:
    RobotDrive& drive;
    Gyro& gyro;

    // operating parameters configured from preferences/SmartDashboard
    float approach_speed;
    float ramp_speed;
    float brake_speed;
    float tilt_up;
    float tilt_down;
    UINT32 ramp_time;
    UINT32 brake_time;

    // runtime state
    bool running;		// are we in control?
    enum { kInitialized, kApproach, kOnRamp, kBraking, kBalanced } state; // operating state
    bool reverse;		// running in reverse
    float speed;		// current motor speed
    float tilt;			// accumulated angle
    long when;			// timestamp for braking   

public:
    Balance( RobotDrive& driveTrain, Gyro& pitchGyro );
    ~Balance();
    
    void InitBalance();
    void Start( bool reverse = false, bool startOnRamp = false );
    void Stop();

    bool Run();

    float GetSpeed();
    bool IsRunning();
    bool IsOnRamp();
    bool IsBraking();
    bool IsBalanced();
};

#endif // _BALANCE_H_
