// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _OI_H_
#define _OI_H_

#include <WPILib.h>
#include "xCANJaguar.h"
#include "xCounter.h"
#include "xGearTooth.h"
#include "xPIDController.h"
#include "Balance.h"
#include "Pickup.h"
#include "Shooter.h"
#include "Target.h"

class OI
{
private:
    DriverStationEnhancedIO *pIO;
    Joystick joy_right, joy_left;

public:
    OI();

    float GetLeftY()		{ return joy_left.GetY(); }
    float GetLeftX()		{ return joy_left.GetX(); }
    float GetLeftTwist()	{ return joy_left.GetTwist(); }
    float GetLeftThrottle()	{ return joy_left.GetThrottle(); }
    bool  GetLeftTrigger()	{ return joy_left.GetTrigger(); }
    bool  GetLeftTop()		{ return joy_right.GetTrigger(); }

    float GetRightY()		{ return joy_right.GetY(); }
    float GetRightX()		{ return joy_right.GetX(); }
    float GetRightTwist()	{ return joy_right.GetTwist(); }
    float GetRightThrottle()	{ return joy_right.GetThrottle(); }
    bool  GetRightTrigger()	{ return joy_right.GetTrigger(); }
    bool  GetRightTop()		{ return joy_right.GetTrigger(); }

    bool Brake()		{ return joy_right.GetRawButton(2); }
    bool TurnLeft10()		{ return joy_right.GetRawButton(3); }
    bool TurnRight10()		{ return joy_right.GetRawButton(4); }
    bool TurnLeft3()		{ return joy_right.GetRawButton(5); }
    bool TurnRight3()		{ return joy_right.GetRawButton(6); }

    // 3-position switch, ball pickup
    int   BallPickup()		{ return (int)(pIO->GetAnalogInRatio(1) * 2.0 + 0.5); }
    // 3-position switch, cowcatcher
    int   Cowcatcher()		{ return (int)(pIO->GetAnalogInRatio(2) * 2.0 + 0.5); }
    // potentiometer, shot speed
    float Adjust()		{ return pIO->GetAnalogInRatio(3); }
    // 3-position switch, shooter
    int   Shooter()		{ return (int)(pIO->GetAnalogInRatio(4) * 2.0 + 0.5); }
    // 3-position switch, unused
    int   Extra()		{ return (int)(pIO->GetAnalogInRatio(5) * 2.0 + 0.5); }

    // pushbutton, fire control
    bool  Shoot()		{ return pIO->GetDigital(1); }
    // key switch, teach mode
    bool  Teach()		{ return pIO->GetDigital(2); }
    // pushbutton, store
    bool  Store()		{ return pIO->GetDigital(3); }
    // pushbutton, target top
    bool  TargetTop()		{ return pIO->GetDigital(4); }
    // pushbutton, target left
    bool  TargetLeft()		{ return pIO->GetDigital(5); }
    // pushbutton, target right
    bool  TargetRight()		{ return pIO->GetDigital(6); }
    // pushbutton, target bottom
    bool  TargetBottom()	{ return pIO->GetDigital(7); }
    // toggle switch, illuminator
    bool  Illuminator()		{ return pIO->GetDigital(8); }
};

#endif // _OI_H_
