// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#ifndef _BALANCE_H_
#define _BALANCE_H_

#include <WPILib.h>

class Balance
{
private:
    RobotDrive& drive;
    AnalogChannel& gyro;

    INT16 level;
    float speed;
    bool isOnRamp;
    bool isBalanced;

public:
    Balance( RobotDrive& driveTrain, AnalogChannel& pitchGyro );
    ~Balance();

    void Start( float initialSpeed, bool startOnRamp );
    void Stop();

    void Run();

    float GetSpeed();
    bool IsRunning();
    bool IsOnRamp();
    bool IsBalanced();
};

#endif // _BALANCE_H_
