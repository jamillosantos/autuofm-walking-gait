/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_ROBOT_H
#define WALKING_ROBOT_H

#include <chrono>
#include <thread>

#include "../data/vector.h"

namespace mote
{
namespace walking
{

class Robot
{
private:
	data::Vector3d _velocity;
	float _velocityTheta;
public:
	void run();

	void init();

	void standInit(double _Speed);

	void Stand_Init_T(double _Speed, int T);

	void Omni_Gait(double vx, double vy, double vt);
};
}
}


#endif //WALKING_ROBOT_H
