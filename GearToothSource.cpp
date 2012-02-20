// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#include <WPILib.h>
#include "GearToothSource.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

GearToothSource::GearToothSource( UINT32 channel,
				  bool directionSensitive = false ) :
    GearTooth( channel, directionSensitive )
{
}

GearToothSource::GearToothSource( UINT32 slot, UINT32 channel,
				  bool directionSensitive = false ) :
    GearTooth( slot, channel, directionSensitive )
{
}

GearToothSource::GearToothSource( DigitalSource *source,
				  bool directionSensitive = false ) :
    GearTooth( source, directionSensitive )
{
}

GearToothSource::GearToothSource( DigitalSource &source,
				  bool directionSensitive = false ) :
    GearTooth( source, directionSensitive )
{
}

GearToothSource::~GearToothSource()
{
}

double GearToothSource::PIDGet()
{
    return  1.0 / GetPeriod();
}
