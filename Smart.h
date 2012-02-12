// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _SMART_H_
#define _SMART_H_

#include <WPILib.h>

class SmartJoystick : public Joystick
{
public:
    explicit SmartJoystick( UINT32 port, const char* name );
    SmartJoystick( UINT32 port, UINT32 numAxisTypes, UINT32 numButtonTypes, const char* name );
    virtual ~SmartJoystick();

    virtual float GetX( JoystickHand hand = kRightHand );
    virtual float GetY( JoystickHand hand = kRightHand );
    virtual float GetZ();
    virtual float GetTwist();
    virtual float GetThrottle();

    virtual bool GetTrigger( JoystickHand hand = kRightHand );
    virtual bool GetTop( JoystickHand hand = kRightHand );
	
    virtual float GetMagnitude();
    virtual float GetDirectionRadians();
    virtual float GetDirectionDegrees();

private:
    DISALLOW_COPY_AND_ASSIGN( SmartJoystick );
    void InitSmartJoystick();
    char* LogName( const char* axisName );

    const char *m_name;

    const char *m_xName;
    const char *m_yName;
    const char *m_zName;
    const char *m_twistName;
    const char *m_throttleName;

    const char *m_magnitudeName;
    const char *m_directionName;

    const char *m_triggerName;
    const char *m_topName;
};

class SmartVictor : public Victor
{
public:
    explicit SmartVictor( UINT32 channel, const char* name );
    SmartVictor( UINT32 slot, UINT32 channel, const char* name );
    virtual ~SmartVictor();
    virtual void Set( float value, UINT8 syncGroup=0 );
    virtual void Disable();

private:
    DISALLOW_COPY_AND_ASSIGN( SmartVictor );
    const char *m_name;
};

class SmartJaguar : public Jaguar
{
public:
    explicit SmartJaguar( UINT32 channel, const char* name );
    SmartJaguar( UINT32 slot, UINT32 channel, const char* name );
    virtual ~SmartJaguar();
    virtual void Set( float value, UINT8 syncGroup=0 );
    virtual void Disable();

private:
    DISALLOW_COPY_AND_ASSIGN( SmartJaguar );
    const char *m_name;
};

class SmartCANJaguar : public CANJaguar
{
public:
    explicit SmartCANJaguar(UINT8 deviceNumber, const char* name, ControlMode controlMode = kPercentVbus);
    virtual ~SmartCANJaguar();
    virtual void Set( float value, UINT8 syncGroup=0 );
    virtual void Disable();

    void Log();

private:
    DISALLOW_COPY_AND_ASSIGN( SmartCANJaguar );
    void InitSmartCANJaguar();
    char* LogName( const char* varName );

    const char *m_name;

    const char *m_setName;
    const char *m_voltageName;
    const char *m_currentName;
    const char *m_speedName;
    const char *m_positionName;
};

class SmartPWM : public PWM
{
public:
    explicit SmartPWM( UINT32 channel, const char* name );
    SmartPWM( UINT32 slot, UINT32 channel, const char* name );
    virtual ~SmartPWM();
    virtual void SetRaw( UINT8 raw );

private:
    DISALLOW_COPY_AND_ASSIGN( SmartPWM );
    const char *m_name;
};

class SmartEncoder: public Encoder
{
public:
	SmartEncoder( UINT32 aChannel, UINT32 bChannel,
		      const char* name,
		      bool reverseDirection = false,
		      Encoder::EncodingType encodingType = k4X );

	SmartEncoder( UINT32 aSlot, UINT32 aChannel,
		      UINT32 bSlot, UINT32 bChannel,
		      const char* name,
		      bool reverseDirection = false,
		      Encoder::EncodingType encodingType = k4X );

	SmartEncoder( DigitalSource *aSource, DigitalSource *bSource,
		      const char* name,
		      bool reverseDirection = false,
		      Encoder::EncodingType encodingType = k4X );

	SmartEncoder( DigitalSource &aSource, DigitalSource &bSource,
		      const char* name,
		      bool reverseDirection = false,
		      Encoder::EncodingType encodingType = k4X );

	virtual ~SmartEncoder();

	// Override 'Get' methods
	INT32 Get();
	INT32 GetRaw();
	double GetPeriod();
	bool GetStopped();
	bool GetDirection();
	double GetDistance();
	double GetRate();
	double PIDGet();

private:
	DISALLOW_COPY_AND_ASSIGN( SmartEncoder );
	void InitSmartEncoder();
	char* LogName( const char* axisName );

	const char* m_name;

	const char* m_rawName;
	const char* m_periodName;
	const char* m_stoppedName;
	const char* m_directionName;
	const char* m_distanceName;
	const char* m_rateName;
	const char* m_pidName;
};

#endif // _SMART_H_
