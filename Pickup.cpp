// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "Pickup.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

Pickup::Pickup( Relay &motor_relay, Counter &ball_counter ) :
    relay( motor_relay ),
    counter( ball_counter ),
    direction( 0 )
{
    Stop();
}

Pickup::~Pickup()
{
    Stop();
}

void Pickup::SetDirection( int d )
{
    direction = d;
    switch (direction) {
    case -1:
	relay.Set(Relay::kReverse);
	break;
    case 0:
	relay.Set(Relay::kOff);
	break;
    case 1:
	relay.Set(Relay::kForward);
	break;
    }
}

int Pickup::GetDirection()
{
    return direction;
}

void Pickup::Start()
{
    SetDirection( 1 );
}

void Pickup::Reverse()
{
    SetDirection( -1 );
}

void Pickup::Stop()
{
    SetDirection( 0 );
}

