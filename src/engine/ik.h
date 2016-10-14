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
	Configuration &configuration;
	sensors::IMU &imu;
public:
	HumanoidIK(HumanoidPart &humanoid, Configuration& configuration, sensors::IMU &imu);

	void update(double _R_Leg_Speed, double _L_Leg_Speed, Leg rightLeg, Leg leftLeg, Arm rightArm, Arm leftArm);
};
}
}


#endif //WALKING_ENGINE_IK_H
