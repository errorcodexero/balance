// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include <WPILib.h>

class Shooter
{
private:
    PIDOutput &motor_bottom, &motor_top;
    PIDSource &sensor_bottom, &sensor_top;

    // operating parameters configured from preferences/SmartDashboard
    float pid_p, pid_i, pid_d;

    // motor controllers
    PIDController pid_bottom, pid_top;

    // runtime control
    bool running;

public:
    Shooter( PIDOutput &mb, PIDOutput &mt, PIDSource &gb, PIDSource &gt );
    ~Shooter();
    
    void InitShooter();
    void Start( float speed_bottom, float speed_top );
    void Stop();

    bool IsRunning();
    bool OnTarget();
};

#endif // _SHOOTER_H_
