// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include <WPILib.h>
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"

class Shooter
{
private:
    /*PIDOutput*/ Victor motor_bottom;
    /*PIDOutput*/ Victor motor_top;
    /*PIDSource*/ xGearTooth geartooth_bottom;
    /*PIDSource*/ xGearTooth geartooth_top;
    Solenoid injector;

    // operating parameters configured from preferences/SmartDashboard
    float pid_p, pid_i, pid_d;
    float drive_ratio;
    float tolerance;
    float shot_time;
    float release_time;

    // motor speed controllers
    xPIDController pid_bottom, pid_top;

    Timer motor_timer;
    Timer shot_timer;

    // runtime state
    float speed_bottom, speed_top;
    bool running;
    enum { kIdle, kShooting, kResetting } shooting;

    // operator control panel


public:
    Shooter( int bottom_motor_channel, int top_motor_channel,
    	     int bottom_geartooth_channel, int top_geartooth_channel,
	     int injector_channel );
    ~Shooter();
    
    void InitShooter();
    void Log();

    static float Ballistics( int target_height, float distance );

    void SetTarget( int height, float distance, float adjust );
    void SetSpeed( float speed );	// set motor speed
    void Start();			// start the motors
    void Stop();			// stop the motors

    void Shoot();			// shoot a ball
    void Reset();			// reset injector for next shot

    void Run();				// update motor and injector status

    bool IsRunning();
    bool IsShooting();
    bool IsReady();
};

#endif // _SHOOTER_H_
