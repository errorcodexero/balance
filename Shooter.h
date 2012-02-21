// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include <WPILib.h>
#include "GearToothSource.h"

class Shooter
{
private:
    /*PIDOutput*/ Victor &motor_bottom;
    /*PIDOutput*/ Victor &motor_top;
    /*PIDSource*/ GearToothSource &sensor_bottom;
    /*PIDSource*/ GearToothSource &sensor_top;

    // operating parameters configured from preferences/SmartDashboard
    float pid_p, pid_i, pid_d;
    float speed_bottom, speed_top;

    // motor controllers
    PIDController pid_bottom, pid_top;

    // runtime control
    bool running;

public:
    Shooter( /*PIDOutput*/ Victor &mb, /*PIDOutput*/ Victor &mt,
    	     /*PIDSource*/ GearToothSource &gb, /*PIDSource*/ GearToothSource &gt );
    ~Shooter();
    
    void InitShooter();

    void Start();
    void Stop();
    void Run();

    bool IsRunning();
    bool OnTarget();
};

#endif // _SHOOTER_H_
