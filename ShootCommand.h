// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _SHOOTCOMMAND_H_
#define _SHOOTCOMMAND_H_

#include <WPILib.h>
#include "Target.h"

class MyRobot;

class ShootCommand
{
private:
    // pointer to owner
    MyRobot& m_robot;

    Target::TargetLocation targetLocation;
    enum { kLights, kCamera, kAction, kShooting, kNoTarget } fireControl;
    Timer turnTimer;

    static const float aimTolerance;

public:
    ShootCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();
};

#endif // _MYROBOT_H_
