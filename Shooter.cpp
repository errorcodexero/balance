// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "Shooter.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

#define	PID_P	0.300F
#define	PID_I	0.000F
#define	PID_D	0.000F

Shooter::Shooter( PIDOutput &mb, PIDOutput &mt, PIDSource &gb, PIDSource &gt ) :
    motor_bottom(mb), motor_top(mt),
    sensor_bottom(gb), sensor_top(gt),
    pid_p(0.0F), pid_i(0.0F), pid_d(0.0F),
    pid_bottom( pid_p, pid_i, pid_d, &sensor_bottom, &motor_bottom ),
    pid_top( pid_p, pid_i, pid_d, &sensor_top, &motor_top ),
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

    pid_bottom.SetPID( pid_p, pid_i, pid_d );
    pid_top.SetPID( pid_p, pid_i, pid_d );
}

void Shooter::Start( float speed_bottom, float speed_top )
{
    if (!IsRunning()) {
	InitShooter();
	pid_bottom.Enable();
	pid_top.Enable();
	running = true;
    }

    pid_bottom.SetSetpoint( speed_bottom );
    pid_top.SetSetpoint( speed_top );
}

void Shooter::Stop()
{
    pid_bottom.Disable();
    pid_top.Disable();
    // motor_bottom.Disable();
    // motor_top.Disable();
    running = false;
}

bool Shooter::IsRunning()
{
    return (running);
}

bool Shooter::OnTarget()
{
    return (pid_bottom.OnTarget() && pid_top.OnTarget());
}

