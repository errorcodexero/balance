// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _PICKUP_H_
#define _PICKUP_H_

#include <WPILib.h>
#include "xCounter.h"

class Pickup
{
public:
    Pickup( Relay &motor_relay );
    ~Pickup();

    void Forward();
    void Reverse();
    void Stop();

    void SetDirection( int d );
    int GetDirection();

private:
    // motor controller
    Relay &relay;

    // runtime control
    int direction;	// -1 reverse, 0 stopped, 1 forward
};

#endif // _PICKUP_H_
