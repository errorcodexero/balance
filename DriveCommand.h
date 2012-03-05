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

    // dashboard controls
    SendableChooser driveChooser;
    typedef enum { kFlightStick, kArcade, kXY, kTwoStick } DriveType;
    DriveType driveMode;

    SendableChooser controlChooser;
    typedef enum { kVoltage, kSpeed } ControlMode;
    ControlMode controlMode;

    // internal state
    bool m_prevTrigger;	// previous trigger state

public:
    DriveCommand( MyRobot& theRobot );

    void Start();
    void Stop();
    bool Run();
};

#endif // _MYROBOT_H_
