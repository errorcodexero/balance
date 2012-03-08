// sample robot code
// Steve Tarr - team 1425 mentor

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
#define	PID_P		0.010F	// initial PID constants, can be tuned in preferences
#define	PID_I		0.001F
#define	PID_D		0.000F
#define	DRIVE_RATIO	0.70F	// empirical value, provides some backspin
#define	ADJUST		4.0F	// speed adjustment range (%)
#define	TOLERANCE	3.0F	// speed tolerance (%)
// stupid WPILib timers are only 1s resolution
#define	MOTOR_START	0.2F	// time to wait before encoder output is valid
#define SHOT_TIME	0.8F	// time to cycle injector up
#define RELEASE_TIME	1.2F	// time to cycle injector down

Shooter::Shooter( int bottom_motor_channel, int top_motor_channel,
		  int bottom_geartooth_channel, int top_geartooth_channel,
		  int injector_channel ) :
    motor_bottom(bottom_motor_channel),
    motor_top(top_motor_channel),
    geartooth_bottom(bottom_geartooth_channel),
    geartooth_top(top_geartooth_channel),
    injector(injector_channel),
    pid_p(PID_P), pid_i(PID_I), pid_d(PID_D), drive_ratio(DRIVE_RATIO),
    tolerance(TOLERANCE), shot_time(SHOT_TIME), release_time(RELEASE_TIME),
    pid_bottom( pid_p, pid_i, pid_d, &geartooth_bottom, &motor_bottom ),
    pid_top( pid_p, pid_i, pid_d, &geartooth_top, &motor_top ),
    speed_bottom(0.0F), speed_top(0.0F),
    running(false),
    shooting(kIdle)
{
    Preferences *pref = Preferences::GetInstance();
    bool saveNeeded = false;
    
    printf("In Shooter constructor, pref = 0x%p\n", pref);
    if (!pref->ContainsKey( "Shooter.pid_p" )) {
	pref->PutDouble( "Shooter.pid_p", PID_P );
	printf("Preferences: save P\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.pid_i" )) {
	pref->PutDouble( "Shooter.pid_i", PID_I );
	printf("Preferences: save I\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.pid_d" )) {
	pref->PutDouble( "Shooter.pid_d", PID_D );
	printf("Preferences: save D\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.drive_ratio" )) {
	pref->PutDouble( "Shooter.drive_ratio", DRIVE_RATIO );
	printf("Preferences: save drive_ratio\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.tolerance" )) {
	pref->PutDouble( "Shooter.tolerance", TOLERANCE );
	printf("Preferences: save tolerance\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.shot_time" )) {
	pref->PutDouble( "Shooter.shot_time", SHOT_TIME );
	printf("Preferences: save shot_time\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.release_time" )) {
	pref->PutDouble( "Shooter.release_time", RELEASE_TIME );
	printf("Preferences: save release_time\n");
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

    pid_p = pref->GetDouble( "Shooter.pid_p", PID_P );
    pid_i = pref->GetDouble( "Shooter.pid_i", PID_I );
    pid_d = pref->GetDouble( "Shooter.pid_d", PID_D );
    drive_ratio = pref->GetDouble( "Shooter.drive_ratio", DRIVE_RATIO );
    tolerance = pref->GetDouble( "Shooter.tolerance", TOLERANCE );
    shot_time = pref->GetDouble( "Shooter.shot_time", SHOT_TIME );
    release_time = pref->GetDouble( "Shooter.release_time", RELEASE_TIME );

    printf("InitShooter: pid_p = %7.4f\n", pid_p);
    printf("InitShooter: pid_i = %7.4f\n", pid_i);
    printf("InitShooter: pid_d = %7.4f\n", pid_d);
    printf("InitShooter: drive_ratio = %5.2f\n", drive_ratio);
    printf("InitShooter: tolerance = %4.1f\n", tolerance);
    printf("InitShooter: shot_time = %4.1f\n", shot_time);
    printf("InitShooter: release_time = %4.1f\n", release_time);

    pid_bottom.SetInputRange( 0.0F, MAX_PPS );
    // PWMController doesn't like it when we use "1.0F" as the maximum.
    pid_bottom.SetOutputRange( 0.0F, 0.99F );
    // This needs some calibration...
    pid_bottom.SetTolerance( tolerance );
    pid_bottom.SetPID( pid_p, pid_i, pid_d );

    pid_top.SetInputRange( 0.0F, MAX_PPS );
    // PWMController doesn't like it when we use "1.0F" as the maximum.
    pid_top.SetOutputRange( 0.0F, 0.99F );
    // This needs some calibration...
    pid_top.SetTolerance( tolerance );
    pid_top.SetPID( pid_p, pid_i, pid_d );

    geartooth_bottom.SetAverageSize( 8 );
    geartooth_top.SetAverageSize( 8 );

    geartooth_bottom.Start();
    geartooth_top.Start();

    motor_timer.Start();
    shot_timer.Start();

    Log();
}

void Shooter::Log()
{
    SmartDashboard::Log(speed_bottom, "b set");
    SmartDashboard::Log(pid_bottom.GetInput(), "b spd");
    SmartDashboard::Log(pid_bottom.GetError(), "b err");
    SmartDashboard::Log(speed_top, "t set");
    SmartDashboard::Log(pid_top.GetInput(), "t spd");
    SmartDashboard::Log(pid_top.GetError(), "t err");
    SmartDashboard::Log(IsReady(), "shooter");
}

void Shooter::SetSpeed( float speed )
{
    speed_bottom = speed * MAX_PPS;
    pid_bottom.SetSetpoint( speed_bottom );
    speed_top = speed_bottom * drive_ratio;
    pid_top.SetSetpoint( speed_top );
}

void Shooter::SetTarget( int target, float distance, float adjust )
{
    // these constants for distances in inches
    const float ballistic_low[3] = { -415.0, 136.3/12., -4.000/144. };	// wild guess
    const float ballistic_mid[3] = { -75.00, 98.33/12., -2.777/144. };
    const float ballistic_high[3] = { 265.0, 60.33/12., -1.556/144. };

    const float *coeff;

    switch (target) {
    case 0:
	coeff = ballistic_low;
	break;
    case 1:
	coeff = ballistic_mid;
	break;
    case 2:
	coeff = ballistic_high;
	break;
    default:
	// invalid
	return;
    }

    float speed = coeff[0] + distance * (coeff[1] + (distance * coeff[2]));
    speed *= (1.0 + adjust * ADJUST / 100.0F);
    printf("Shooter height %d distance %g adjust %g speed %g\n",
    		target, distance, adjust, speed);

    speed_bottom = speed;
    pid_bottom.SetSetpoint( speed_bottom );
    speed_top = speed_bottom * drive_ratio;
    pid_top.SetSetpoint( speed_top );

    Log();
}

void Shooter::Start()
{
    if (IsRunning()) return;

    // start the motor safety protection
    motor_bottom.SetSafetyEnabled(true);
    motor_top.SetSafetyEnabled(true);

    // start the PID controller
    pid_bottom.Enable();
    pid_top.Enable();

    motor_timer.Reset();
    running = true;

    Run();
    Log();
}

void Shooter::Stop()
{
    if (!IsRunning()) return;

    pid_bottom.Reset();
    pid_top.Reset();
    motor_bottom.Disable();
    motor_bottom.SetSafetyEnabled(false);
    motor_top.Disable();
    motor_top.SetSafetyEnabled(false);
    running = false;

    Reset();
}

void Shooter::Shoot()
{
    if (IsReady()) {
	injector.Set(true);
	shot_timer.Reset();
	shooting = kShooting;
    }
}

void Shooter::Reset()
{
    if (shooting != kIdle) {
	injector.Set(false);
	shot_timer.Reset();
	shooting = kResetting;
    }
}

void Shooter::Run()
{
    static int logCount = 0;
    if (IsRunning()) {
	pid_bottom.SetSetpoint( speed_bottom );
	pid_top.SetSetpoint( speed_top );
	if (++logCount >= 20) {
	    Log();
	    logCount = 0;
	}
    } else {
	logCount = 0;
    }

    switch (shooting) {
    case kIdle:
	break;
    case kShooting:
	if (shot_timer.Get() > shot_time) {
	    injector.Set(false);
	    shot_timer.Reset();
	    shooting = kResetting;
	}
	break;
    case kResetting:
	if (shot_timer.Get() > release_time) {
	    shooting = kIdle;
	}
	break;
    }
}

bool Shooter::IsRunning()
{
    return (running);
}

bool Shooter::IsShooting()
{
    return (shooting != kIdle);
}

bool Shooter::IsReady()
{
    return (IsRunning() && !IsShooting() &&
    	    (motor_timer.Get() > MOTOR_START) &&
	    pid_bottom.OnTarget() && pid_top.OnTarget());
}

