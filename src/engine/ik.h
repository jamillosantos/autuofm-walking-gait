/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#ifndef WALKING_ENGINE_IK_H
#define WALKING_ENGINE_IK_H

#include "part.h"
#include "../configuration/configuration.h"
#include "../sensors/imu.h"

namespace mote
{
namespace walking
{
class HumanoidIK
{
private:
	HumanoidPart &humanoid;
	HumanoidPart &humanoidPublished;
	Configuration &configuration;
	sensors::IMU &imu;
public:
	HumanoidIK(HumanoidPart &humanoid, HumanoidPart &humanoidPublished, Configuration& configuration, sensors::IMU &imu);

	void update();
};
}
}


#endif //WALKING_ENGINE_IK_H
