/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#include "simuservo.h"

void mote::walking::motors::SimuServo::apply()
{ }

mote::walking::motors::Servo *mote::walking::motors::SimuServoFactory::create()
{
	return new SimuServo();
}
