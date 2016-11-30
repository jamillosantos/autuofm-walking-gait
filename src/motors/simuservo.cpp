/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#include "simuservo.h"

mote::walking::motors::SimuServo::SimuServo(const std::string &name, bool inverted)
	: Servo(inverted), name(name)
{ }
