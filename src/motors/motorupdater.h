/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#ifndef WALKING_MOTORSUPDATER_H
#define WALKING_MOTORSUPDATER_H

#include <thread>

#include "../engine/part.h"
#include "robot.h"

namespace mote
{
namespace walking
{
namespace motors
{
class MotorUpdater
{
private:
	volatile bool _running;

	std::unique_ptr<std::thread> _thread;

	HumanoidPart &_humanoidPart;

	void runTrampoline();
protected:
	virtual void updateState() = 0;
	virtual void flushChanges() = 0;
public:
	MotorUpdater(HumanoidPart &humanoidPart);

	void start();
	void stop();
};

}
}
}


#endif //WALKING_MOTORSUPDATER_H
