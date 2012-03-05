// sample robot code
// Steve Tarr - team 1425 mentor

#include <WPILib.h>
#include "OI.h"
#include "Version.h"
static Version v( __FILE__ " " __DATE__ " " __TIME__ );

OI::OI() : pIO(NULL), joy_right(1), joy_left(2)
{
     DriverStation *pDS = DriverStation::GetInstance();
     pIO = &pDS->GetEnhancedIO();
}

