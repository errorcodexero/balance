// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Shooter.h"
#include "MyRobot.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

// motor unloaded max speed: 20,700 RPM
// gear reduction 4:1
// sensor ration 1:15 (number of teeth on the sprocket)
// so theoretical unloaded max PPS is approx. 1300
// measured max PPS is approx. 1000

#define	MAX_PPS		1200.0F	// max pulses per second from gear tooth sensor
#define	SHOOTER_P	0.010F	// initial PID constants, can be tuned in preferences
#define	SHOOTER_I	0.001F
#define	SHOOTER_D	0.000F
#define	DRIVE_RATIO	0.70F	// empirical value, provides some backspin
#define	ADJUST		0.04F	// speed adjustment range (+/-4%)
#define	TOLERANCE	3.0F	// speed tolerance (%)
#define	MOTOR_START	0.5F	// time to wait before encoder output is valid
#define SHOT_TIME	0.8F	// time to cycle injector up
#define RELEASE_TIME	1.4F	// time to cycle injector down

Shooter::Shooter( MyRobot& theRobot, int bottom_motor_channel, int top_motor_channel,
		  int bottom_geartooth_channel, int top_geartooth_channel,
		  int injector_channel ) :
    m_robot(theRobot),
    motor_bottom(bottom_motor_channel),
    motor_top(top_motor_channel),
    geartooth_bottom(bottom_geartooth_channel),
    geartooth_top(top_geartooth_channel),
    injector(injector_channel),
    pid_p(SHOOTER_P), pid_i(SHOOTER_I), pid_d(SHOOTER_D), drive_ratio(DRIVE_RATIO),
    tolerance(TOLERANCE), shot_time(SHOT_TIME), release_time(RELEASE_TIME),
    pid_bottom( pid_p, pid_i, pid_d, &geartooth_bottom, &motor_bottom ),
    pid_top( pid_p, pid_i, pid_d, &geartooth_top, &motor_top ),
    m_auto(false), m_speed(0.0F),
    speed_bottom(0.0F), speed_top(0.0F),
    running(false),
    shooting(kIdle)
{
    Preferences *pref = Preferences::GetInstance();
    bool saveNeeded = false;
    
    printf("In Shooter constructor, pref = 0x%p\n", pref);
    if (!pref->ContainsKey( "Shooter.pid_p" )) {
	pref->PutDouble( "Shooter.pid_p", SHOOTER_P );
	printf("Preferences: save P\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.pid_i" )) {
	pref->PutDouble( "Shooter.pid_i", SHOOTER_I );
	printf("Preferences: save I\n");
	saveNeeded = true;
    }
    if (!pref->ContainsKey( "Shooter.pid_d" )) {
	pref->PutDouble( "Shooter.pid_d", SHOOTER_D );
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

    pid_p = pref->GetDouble( "Shooter.pid_p", SHOOTER_P );
    pid_i = pref->GetDouble( "Shooter.pid_i", SHOOTER_I );
    pid_d = pref->GetDouble( "Shooter.pid_d", SHOOTER_D );
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
}

void Shooter::Log()
{
#if 0
    static int logCount = 0;

    if (IsRunning()) {
	if (++logCount >= 20) {
	    SmartDashboard::Log(m_speed, "set speed");
	    SmartDashboard::Log(speed_bottom, "b set");
	    SmartDashboard::Log(pid_bottom.GetInput(), "b spd");
	    SmartDashboard::Log(pid_bottom.GetError(), "b err");
	    SmartDashboard::Log(speed_top, "t set");
	    SmartDashboard::Log(pid_top.GetInput(), "t spd");
	    SmartDashboard::Log(pid_top.GetError(), "t err");
	    SmartDashboard::Log(IsReady(), "shooter");
	    logCount = 0;
	}
    } else {
	logCount = 0;
    }
#endif

    DriverStationLCD *lcd = DriverStationLCD::GetInstance();
    if (IsRunning()) {
	lcd->PrintfLine(DriverStationLCD::kUser_Line3,
	    "t %6.0f %6.0f",
	    speed_top, pid_top.GetInput());
	lcd->PrintfLine(DriverStationLCD::kUser_Line4,
	    "b %6.0f %6.0f",
	    speed_bottom, pid_bottom.GetInput());
    } else {
	lcd->PrintfLine(DriverStationLCD::kUser_Line3,
	    "t stopped");
	lcd->PrintfLine(DriverStationLCD::kUser_Line4,
	    "b stopped");
    }
    lcd->UpdateLCD();
}

float Shooter::Ballistics( int height, float distance )
{
    // these constants for distances in inches, speed in PPS
    const float swish_low[3] = { 3.702E+02, -1.293E+00, 2.145E-02 };
    const float backboard_low[3] = { 3.773E+02, 1.867E+00, 0.000E+00 };
//  const float swish_mid[3] = { 1.296E+02, 4.956E+00, -8.586E-03 };
    const float backboard_mid[3] = { 2.755E+02, 3.556E+00, -3.864E-03 };
    const float backboard_high[3] = { 6.2146E+02, 4.5341E-01, 3.3061E-03 };

    const float *coeff;

    switch (height) {
    case 0:
	coeff = (distance < 133.) ? swish_low : backboard_low;
	break;
    case 1:
	coeff = backboard_mid;
	break;
    case 2:
	coeff = backboard_high;
	break;
    default:
	// invalid
	return 0.;
    }

    return coeff[0] + distance * (coeff[1] + (distance * coeff[2]));
}

void Shooter::UpdateSpeed()
{
    float adjust = m_robot.GetOI().Adjust();

    if (m_auto) {
	// adjust is a +/-ADJUST% adjustment to base m_speed
	speed_bottom = m_speed * (1.0 + ((adjust * 2.0 - 1.0) * ADJUST));
    } else {
	// ignore m_speed; adjust is 30..95% of max speed
	speed_bottom = (0.300 + (adjust * 0.650)) * MAX_PPS;
    }
    speed_top = speed_bottom * drive_ratio;
    pid_bottom.SetSetpoint( speed_bottom );
    pid_top.SetSetpoint( speed_top );
}

void Shooter::SetManual()
{
    printf("Shooter::SetManual\n");
    m_auto = false;
    m_speed = 0.;
    UpdateSpeed();
}

void Shooter::SetTarget( int height, float distance )
{
    printf("Shooter::SetTarget %d %g\n", height, distance);
    m_auto = true;
    m_speed = Ballistics(height, distance);
    if (m_robot.GetOI().Teach()) {
	printf("SetTarget height %d distance %g speed %g\n",
		    height, distance, m_speed);
    }
    UpdateSpeed();
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
    if (IsRunning()) {
	UpdateSpeed();
    }
    Log();

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

bool Shooter::IsReady()
{
    return (IsRunning() && !IsShooting() &&
    	    (motor_timer.Get() > MOTOR_START) &&
	    pid_bottom.OnTarget() && pid_top.OnTarget());
}

