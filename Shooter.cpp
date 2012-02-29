// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Shooter.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

// motor unloaded max speed: 20,700 RPM
// gear reduction 4:1
// sensor ration 1:15 (number of teeth on the sprocket)
// so theoretical unloaded max PPS is approx. 1300
// measured max PPS is approx. 1000

#define	MAX_PPS		1200.0F	// max pulses per second from gear tooth sensor
#define	DRIVE_RATIO	0.70F	// empirical value, provides some backspin
#define	PID_P		0.020F
#define	PID_I		0.000F
#define	PID_D		0.000F

Shooter::Shooter( /*PIDOutput*/ Victor &mb, /*PIDOutput*/ Victor &mt,
		  /*PIDSource*/ xGearTooth &gb, /*PIDSource*/ xGearTooth &gt ) :
    motor_bottom(mb), motor_top(mt),
    geartooth_bottom(gb), geartooth_top(gt),
    pid_p(0.0F), pid_i(0.0F), pid_d(0.0F),
    pid_bottom( pid_p, pid_i, pid_d, &geartooth_bottom, &motor_bottom ),
    pid_top( pid_p, pid_i, pid_d, &geartooth_top, &motor_top ),
    speed_bottom(0.0F), speed_top(0.0F),
    running(false)
{
    Preferences *pref = Preferences::GetInstance();
    bool saveNeeded = false;
    
    printf("In Shooter constructor, pref = 0x%p\n", pref);
    if (!pref->ContainsKey( "Shooter.p" )) {
	pref->PutDouble( "Shooter.p", PID_P );
	printf("Preferences: save P\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.i" )) {
	pref->PutDouble( "Shooter.i", PID_I );
	printf("Preferences: save I\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.d" )) {
	pref->PutDouble( "Shooter.d", PID_D );
	printf("Preferences: save D\n");
	saveNeeded = true;
    }
    if (saveNeeded) {
	pref->Save();
	printf("Preferences: saved\n");
    }

    InitShooter();
}

Shooter::~Shooter()
{
    Stop();
}

void Shooter::InitShooter()
{
    Stop();

    Preferences *pref = Preferences::GetInstance();

    pid_p = pref->GetDouble( "Shooter.p", PID_P );
    pid_i = pref->GetDouble( "Shooter.i", PID_I );
    pid_d = pref->GetDouble( "Shooter.d", PID_D );

    printf("InitShooter: pid_p = %f\n", pid_p);
    printf("InitShooter: pid_i = %f\n", pid_i);
    printf("InitShooter: pid_d = %f\n", pid_d);

    pid_bottom.SetInputRange( 0.0F, MAX_PPS );
    // PWMController doesn't like it when we use "1.0F" as the maximum.
    pid_bottom.SetOutputRange( 0.0F, 0.98F );
    // This needs some calibration...
    pid_bottom.SetTolerance( 5.0F );
    pid_bottom.SetPID( pid_p, pid_i, pid_d );

    pid_top.SetInputRange( 0.0F, MAX_PPS );
    // PWMController doesn't like it when we use "1.0F" as the maximum.
    pid_top.SetOutputRange( 0.0F, 0.98F );
    // This needs some calibration...
    pid_top.SetTolerance( 5.0F );
    pid_top.SetPID( pid_p, pid_i, pid_d );

    geartooth_bottom.SetAverageSize( 8 );
    geartooth_top.SetAverageSize( 8 );

    Set( 0.0F );
}

void Shooter::Set( float speed )
{
    // set motor speeds
    speed_bottom = speed * MAX_PPS;
    pid_bottom.SetSetpoint( speed_bottom );
    speed_top = speed_bottom * DRIVE_RATIO;
    pid_top.SetSetpoint( speed_top );
}

void Shooter::Start()
{
    if (IsRunning()) return;

    // enable the motor speed sensors (counters)
    geartooth_bottom.Start();
    geartooth_top.Start();

    // start the motor safety protection
    motor_bottom.SetSafetyEnabled(true);
    motor_top.SetSafetyEnabled(true);

    // start the PID controller
    pid_bottom.Enable();
    pid_top.Enable();

    running = true;
}

void Shooter::Stop()
{
    if (!IsRunning()) return;

    pid_bottom.Reset();
    pid_top.Reset();
    motor_bottom.PIDWrite( 0.0F );
    motor_bottom.SetSafetyEnabled(false);
    motor_top.PIDWrite( 0.0F );
    motor_top.SetSafetyEnabled(false);
    geartooth_bottom.Stop();
    geartooth_top.Stop();
    running = false;
}

void Shooter::Run()
{
    static int logCount = 0;
    if (IsRunning()) {
	if (++logCount >= 20) {
	    SmartDashboard::Log(speed_bottom, "b set");
	    SmartDashboard::Log(pid_bottom.GetInput(), "b spd");
	    SmartDashboard::Log(pid_bottom.GetError(), "b err");
	    SmartDashboard::Log(speed_top, "t set");
	    SmartDashboard::Log(pid_top.GetInput(), "t spd");
	    SmartDashboard::Log(pid_top.GetError(), "t err");
	    logCount = 0;
	}
    } else {
	logCount = 0;
    }
}

bool Shooter::IsRunning()
{
    return (running);
}

bool Shooter::OnTarget()
{
    return (pid_bottom.OnTarget() && pid_top.OnTarget());
}

