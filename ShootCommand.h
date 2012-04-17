// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _SHOOTCOMMAND_H_
#define _SHOOTCOMMAND_H_

#include <WPILib.h>
#include "Target.h"

class MyRobot;

class ShootCommand
{
public:
    ShootCommand( MyRobot& theRobot );

    typedef enum { kLights, kCamera, kAction, kSpinUp, kShoot1, kShoot2, kShoot3, kDone } FireControl;

    void Start( Target::TargetID targetID, FireControl lastState );
    void Stop();
    bool Run();

private:
    // pointer to owner
    MyRobot& m_robot;

    Target::TargetID m_targetID;
    Target::TargetLocation m_targetLocation;

    static const char *StateName( FireControl state );

    void ShootCommand::EnterState( FireControl newState );

    FireControl m_fireControl, m_lastState;
    Timer m_timer;

    static const float cameraWarmup;
    static const float turnTimeout;
    static const float aimTolerance;
};

#endif // _MYROBOT_H_
