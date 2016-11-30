/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#ifndef WALKING_MOTORS_SIMUSERVO_H
#define WALKING_MOTORS_SIMUSERVO_H

#include "servo.h"

namespace mote
{
namespace walking
{
namespace motors
{
class SimuServo
	: public Servo
{
public:
	SimuServo(const std::string &name, bool inverted = false);
	std::string name;
};
}
}
}

#endif //WALKING_SIMUSERVO_H
