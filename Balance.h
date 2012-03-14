// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _BALANCE_H_
#define _BALANCE_H_

#include <WPILib.h>

class MyRobot;

class Balance
{
private:
    // pointer to owner
    MyRobot& m_robot;

    // operating parameters configured from preferences/SmartDashboard
    float ramp_speed;
    float tilt_limit;

    // runtime state
    bool running;		// are we in control?
    float speed;		// current motor speed
    float tilt;			// accumulated angle

public:
    Balance( MyRobot& theRobot );
    ~Balance();
    
    void InitBalance();
    void Start();
    void Stop();

    bool Run();

    float GetSpeed();
    bool IsRunning();
    bool IsBalanced();
};

#endif // _BALANCE_H_
