// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "xCounter.h"
#include "Pickup.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

Pickup::Pickup( Relay &motor_relay ) :
    relay( motor_relay ),
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

void Pickup::Forward()
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

