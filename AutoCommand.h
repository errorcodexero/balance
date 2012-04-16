// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _AUTOCOMMAND_H_
#define _AUTOCOMMAND_H_

#include <WPILib.h>

class MyRobot;

class AutoCommand
{
private:
    static const float shotTimeLimit;
    static const float turnTimeLimit;
    static const float driveTimeLimit;

    // pointer to owner
    MyRobot& m_robot;

    // autonomous function selection
    AnalogChannel selector;
    int autoSequence;
    float shotDistance;
    float turnAngle;
    float driveDistance;

    // internal state
    enum { kShooting, kTurning, kDriving, kStopped } autoState;
    int shotCount;
    Timer autoTimer;

public:
    AutoCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();
};

#endif // _AUTOCOMMAND_H_
