/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_MOTORS_SIMUMOTORSUPDATER_H
#define WALKING_MOTORS_SIMUMOTORSUPDATER_H

#include <src/networking/client.h>
#include "motorupdater.h"
#include "simurobot.h"

namespace mote
{
namespace walking
{
namespace motors
{
class SimuMotorUpdater
	: public MotorUpdater
{
private:
	SimuRobot _robot;
	networking::UDPClient &_client;
public:
	SimuMotorUpdater(networking::UDPClient &client, HumanoidPart &humanoidPart);

	void init();
protected:
	virtual void updateState() override;

	virtual void flushChanges() override;
};
}
}
}


#endif //WALKING_MOTORS_SIMUMOTORSUPDATER_H
