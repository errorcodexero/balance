/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

// sample robot code
// Steve Tarr - team 1425 mentor
//
// xGearTooth is a copy of the WPILib GearTooth class, but based on xCounter
// rather than Counter and with the addition of the PIDSource interface.

#include "xGearTooth.h"

const double xGearTooth::kGearToothThreshold;

/**
 * Common code called by the constructors.
 */
void xGearTooth::EnableDirectionSensing(bool directionSensitive)
{
	if (directionSensitive)
	{
		SetPulseLengthMode(kGearToothThreshold);
	}
}

/**
 * Construct a GearTooth sensor given a channel.
 * 
 * The default module is assumed.
 * 
 * @param channel The GPIO channel on the digital module that the sensor is connected to.
 * @param directionSensitive Enable the pulse length decoding in hardware to specify count direction.
 */
xGearTooth::xGearTooth(UINT32 channel, bool directionSensitive)
	: xCounter(channel)
{
	EnableDirectionSensing(directionSensitive);
}

/**
 * Construct a GearTooth sensor given a channel and module.
 * 
 * @param moduleNumber The digital module (1 or 2).
 * @param channel The GPIO channel on the digital module that the sensor is connected to.
 * @param directionSensitive Enable the pulse length decoding in hardware to specify count direction.
 */
xGearTooth::xGearTooth(UINT8 moduleNumber, UINT32 channel, bool directionSensitive)
	: xCounter(moduleNumber, channel)
{
	EnableDirectionSensing(directionSensitive);
}

/**
 * Construct a GearTooth sensor given a digital input.
 * This should be used when sharing digial inputs.
 * 
 * @param source An object that fully descibes the input that the sensor is connected to.
 * @param directionSensitive Enable the pulse length decoding in hardware to specify count direction.
 */
xGearTooth::xGearTooth(DigitalSource *source, bool directionSensitive)
	: xCounter(source)
{
	EnableDirectionSensing(directionSensitive);
}

xGearTooth::xGearTooth(DigitalSource &source, bool directionSensitive): xCounter(source)
{
	EnableDirectionSensing(directionSensitive);
}

/**
 * Free the resources associated with a gear tooth sensor.
 */
xGearTooth::~xGearTooth()
{
}

/**
 * Return the rotation rate (PIDSource interface)
 */
double xGearTooth::PIDGet()
{
    return  1.0 / GetPeriod();
}

