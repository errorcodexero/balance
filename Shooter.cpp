// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "Shooter.h"
#include "GearToothSource.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

#define	PID_P	0.010F
#define	PID_I	0.000F
#define	PID_D	0.000F

Shooter::Shooter( /*PIDOutput*/ Victor &mb, /*PIDOutput*/ Victor &mt,
		  /*PIDSource*/ GearToothSource &gb, /*PIDSource*/ GearToothSource &gt ) :
    motor_bottom(mb), motor_top(mt),
    sensor_bottom(gb), sensor_top(gt),
    pid_p(0.0F), pid_i(0.0F), pid_d(0.0F),
    pid_bottom( pid_p, pid_i, pid_d, &sensor_bottom, &motor_bottom ),
    pid_top( pid_p, pid_i, pid_d, &sensor_top, &motor_top ),
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
}

void Shooter::Start( float speed )
{
    // once started, keep running with the original parameters
    if (IsRunning()) return;

    // load configuration from preferences file or SmartDashboard
    InitShooter();

    // set motor speeds
    speed_bottom = speed;
    speed_top = speed * 0.7;	// empirical ratio
    printf("Shooter: bottom = %6.0f\n", speed_bottom);
    printf("Shooter: top = %6.0f\n", speed_top);

    // enable the motor speed sensors (counters)
    sensor_bottom.Start();
    sensor_top.Start();

    // set the initial speed
#if 1 // debug
    motor_bottom.PIDWrite( speed_bottom / 5000. );
    motor_bottom.SetSafetyEnabled(false);
    motor_top.PIDWrite( speed_top / 5000. );
    motor_top.SetSafetyEnabled(false);
#else
    pid_bottom.SetPID( pid_p, pid_i, pid_d );
    pid_top.SetPID( pid_p, pid_i, pid_d );
    pid_bottom.SetSetpoint( speed_bottom );
    pid_top.SetSetpoint( speed_top );
    pid_bottom.Enable();
    pid_top.Enable();
#endif
    running = true;
}

void Shooter::Stop()
{
#if 1 // debug
    motor_bottom.PIDWrite( 0.0F );
    motor_bottom.SetSafetyEnabled(false);
    motor_top.PIDWrite( 0.0F );
    motor_top.SetSafetyEnabled(false);
#else
    pid_bottom.Disable();
    pid_top.Disable();
#endif
    sensor_bottom.Stop();
    sensor_top.Stop();
    running = false;
}

void Shooter::Run()
{
    static int logCount = 0;
    if (IsRunning()) {
	if (++logCount > 100) {
	    logCount = 0;
	    SmartDashboard::Log( sensor_bottom.PIDGet(), "bot" );
	    SmartDashboard::Log( sensor_top.PIDGet(), "top" );
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

