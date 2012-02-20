// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

// GearToothSource adds a PIDSource interface to GearTooth

#ifndef _GEARTOOTHSOURCE_H_
#define _GEARTOOTHSOURCE_H_

#include <WPILib.h>

class GearToothSource : public GearTooth, public PIDSource
{
public:
    GearToothSource(UINT32 channel,
    		    bool directionSensitive = false);

    GearToothSource(UINT32 slot, UINT32 channel,
		    bool directionSensitive = false);

    GearToothSource(DigitalSource *source,
		    bool directionSensitive = false);

    GearToothSource(DigitalSource &source,
		    bool directionSensitive = false);

    virtual ~GearToothSource();

    virtual double PIDGet();
};

#endif // _GEARTOOTHSOURCE_H_
