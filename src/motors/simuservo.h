/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#ifndef WALKING_SIMUSERVO_H
#define WALKING_SIMUSERVO_H

#include <src/networking/client.h>
#include "servo.h"
#include "factory.h"

namespace mote
{
namespace walking
{
namespace motors
{
class SimuServo
	: public Servo
{
private:
	std::string name;
public:
	virtual void apply() override;
};

class SimuServoFactory
	: Factory
{
	virtual Servo *create() override;
};
}
}
}

#endif //WALKING_SIMUSERVO_H
