// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _AUTOCOMMAND_H_
#define _AUTOCOMMAND_H_

#include <WPILib.h>

class MyRobot;

class AutoCommand
{
public:
    AutoCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();

private:
    static const float cameraWarmup;
    static const float aimTolerance;
    static const float holdTimeout;
    static const float turnTimeLimit;
    static const float shotTimeLimit;

    // pointer to owner
    MyRobot& m_robot;

    // autonomous function selection
    AnalogChannel m_selector1;	// shooting control
    AnalogChannel m_selector2;	// driving control
    int m_autoSequence1;
    int m_autoSequence2;

    float m_initialWait;
    float m_shotAngle;
    float m_shotDistance;
    float m_driveAngle;
    float m_driveDistance;

    // internal state
    typedef enum { kWait, kLights, kCamera, kAction, kHold,
		   kShooting, kTurning, kDriving, kStopped } AutoState;

    AutoState m_autoState;
    int m_shotCount;
    Timer m_autoTimer;

    static const char *StateName( AutoState state );
    void Init();
    void StateWait();
    void StateLights();
    void StateCamera();
    void StateAction();
    void StateHold();
    void StateSpinUp();
    void StateShooting();
    void StateTurning();
    void StateDriving();
};

#endif // _AUTOCOMMAND_H_
