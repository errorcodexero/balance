/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

// sample robot code
// Steve Tarr - team 1425 mentor - 29-Jan-2012
//
// xGearTooth is a copy of the WPILib GearTooth class, but based on xCounter
// rather than Counter and with the addition of the PIDSource interface.

#ifndef xGEAR_TOOTH_H_
#define xGEAR_TOOTH_H_

#include "xCounter.h"
#include "PIDSource.h"

/**
 * Alias for counter class.
 * Implement the gear tooth sensor supplied by FIRST. Currently there is no reverse sensing on
 * the gear tooth sensor, but in future versions we might implement the necessary timing in the
 * FPGA to sense direction.
 */
class xGearTooth : public xCounter, public PIDSource
{
public:
	/// 55 uSec for threshold
	static const double kGearToothThreshold = 55e-6;
	xGearTooth(UINT32 channel, bool directionSensitive = false);
	xGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive = false);
	xGearTooth(DigitalSource *source, bool directionSensitive = false);
	xGearTooth(DigitalSource &source, bool directionSensitive = false);
	virtual ~xGearTooth();
	void EnableDirectionSensing(bool directionSensitive);
	virtual double PIDGet();
};

#endif
