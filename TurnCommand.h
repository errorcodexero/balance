// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _TURNCOMMAND_H_
#define _TURNCOMMAND_H_

#include <WPILib.h>

class MyRobot;

class TurnCommand
{
private:
    // pointer to owner
    MyRobot& m_robot;

    float m_angle;
    Timer m_turnTimer;
    bool m_turnDebug;
    bool m_turnComplete;

    static const float turnTolerance;

public:
    TurnCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();
};

#endif // _MYROBOT_H_
