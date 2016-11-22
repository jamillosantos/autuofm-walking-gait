/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_MOTORS_SIMUROBOT_H
#define WALKING_MOTORS_SIMUROBOT_H

#include "robot.h"

namespace mote
{
namespace walking
{
namespace motors
{
class SimuRobot
	: public Robot
{
public:
	SimuRobot(HumanoidPart &humanoidPart);

	virtual void init() override;
};
}
}
}


#endif //WALKING_MOTORS_SIMUROBOT_H
