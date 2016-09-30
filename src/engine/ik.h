/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#ifndef WALKING_IK_H
#define WALKING_IK_H

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
	Humanoid &humanoid;
	Configuration &configuration;
	sensors::IMU &imu;
public:
	HumanoidIK(Humanoid &humanoid, Configuration& configuration, sensors::IMU &imu);

	void update(double _R_Leg_Speed, double _L_Leg_Speed, double _R_Leg_Ik[], double _L_Leg_Ik[], double _R_Arm[], double _L_Arm[]);
};
}
}


#endif //WALKING_IK_H
