// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _PICKUP_H_
#define _PICKUP_H_

#include <WPILib.h>

class Pickup
{
private:
    // motor controller
    Relay &relay;

    // ball-in-place sensor
    Counter &counter;

    // runtime control
    bool running;

public:
    Pickup( Relay &motor_relay, Counter &ball_counter );
    ~Pickup();
    
    void Start();
    void Reverse();
    void Stop();

    // TBD: add e.g. IsBallInPosition
};

#endif // _PICKUP_H_
