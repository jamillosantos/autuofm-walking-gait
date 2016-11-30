/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 28, 2016
 */

#include "servo.h"

mote::walking::motors::Servo::Servo(bool inverted)
	: inverted(inverted)
{ }

mote::walking::motors::Servo::Servo()
	: Servo(false)
{ }
