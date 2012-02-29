// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include <WPILib.h>
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"

class Shooter
{
private:
    /*PIDOutput*/ Victor &motor_bottom;
    /*PIDOutput*/ Victor &motor_top;
    /*PIDSource*/ xGearTooth &geartooth_bottom;
    /*PIDSource*/ xGearTooth &geartooth_top;

    // operating parameters configured from preferences/SmartDashboard
    float pid_p, pid_i, pid_d;

    // motor controllers
    xPIDController pid_bottom, pid_top;

    // runtime control
    float speed_bottom, speed_top;
    bool running;

public:
    Shooter( /*PIDOutput*/ Victor &mb, /*PIDOutput*/ Victor &mt,
    	     /*PIDSource*/ xGearTooth &gb, /*PIDSource*/ xGearTooth &gt );
    ~Shooter();
    
    void InitShooter();

    void Set( float speed );
    void Start();
    void Stop();
    void Run();

    bool IsRunning();
    bool OnTarget();
};

#endif // _SHOOTER_H_
