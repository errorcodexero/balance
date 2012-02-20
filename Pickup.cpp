// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "Pickup.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

Pickup::Pickup( Relay &motor_relay, Counter &ball_counter ) :
    relay( motor_relay ),
    counter( ball_counter )
{
    Stop();
}

Pickup::~Pickup()
{
    Stop();
}

void Pickup::Start()
{
    relay.Set( Relay::kForward );
}

void Pickup::Reverse()
{
    relay.Set( Relay::kReverse );
}

void Pickup::Stop()
{
    relay.Set( Relay::kOff );
}

