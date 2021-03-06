// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _DRIVECOMMAND_H_
#define _DRIVECOMMAND_H_

#include <WPILib.h>

class MyRobot;

class DriveCommand
{
private:
    // pointer to owner
    MyRobot& m_robot;

    enum ControlMode { kVoltage, kSpeed };
    ControlMode m_controlMode;

    // internal state
    bool m_prevTrigger;	// previous trigger state

public:
    DriveCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();
};

#endif // _MYROBOT_H_
