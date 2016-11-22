/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#include "motorsupdater.h"

mote::walking::motors::MotorUpdater::MotorUpdater(mote::walking::HumanoidPart &humanoidPart)
	: _running(false), _humanoidPart(humanoidPart), _robot(humanoidPart)
{ }


void mote::walking::motors::MotorUpdater::runTrampoline()
{
	while (this->_running)
	{
		_robot.update();
		this->flushChanges();
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

void mote::walking::motors::MotorUpdater::start()
{
	if (!this->_thread)
	{
		this->_running = true;
		this->_thread.reset(new std::thread(&MotorUpdater::runTrampoline, this));
	}
}

void mote::walking::motors::MotorUpdater::stop()
{
	if (this->_thread)
	{
		this->_running = false;
		this->_thread->join();
	}
}
