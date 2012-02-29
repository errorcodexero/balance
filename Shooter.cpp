// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Shooter.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

#define	PID_P	0.050F
#define	PID_I	0.000F
#define	PID_D	0.000F

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

    pid_bottom.SetInputRange( 0.0F, 1200.F );
    pid_bottom.SetOutputRange( 0.0F, 0.98F );
    pid_bottom.SetTolerance( 3.0F );
    pid_bottom.SetPID( pid_p, pid_i, pid_d );

    pid_top.SetInputRange( 0.0F, 1200.F );
    pid_top.SetOutputRange( 0.0F, 0.98F );
    pid_top.SetTolerance( 3.0F );
    pid_top.SetPID( pid_p, pid_i, pid_d );

    geartooth_bottom.SetAverageSize( 8 );
    geartooth_top.SetAverageSize( 8 );

    Set( 0.0F );
}

void Shooter::Set( float speed )
{
    // set motor speeds
    speed_bottom = speed;
    speed_top = speed * 0.7;	// empirical ratio
    pid_bottom.SetSetpoint( speed_bottom );
    pid_top.SetSetpoint( speed_top );
}

void Shooter::Start()
{
    if (IsRunning()) return;

    // load configuration from preferences file or SmartDashboard
    InitShooter();

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
	    SmartDashboard::Log(speed_bottom, "bot set");
	    SmartDashboard::Log(pid_bottom.GetInput(), "bot spd");
	    SmartDashboard::Log(pid_bottom.GetError(), "bot err");
	    SmartDashboard::Log(speed_top, "top set");
	    SmartDashboard::Log(pid_top.GetInput(), "top spd");
	    SmartDashboard::Log(pid_top.GetError(), "top err");
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

