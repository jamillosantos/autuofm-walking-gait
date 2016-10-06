/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_ROBOT_H
#define WALKING_ROBOT_H

#include <chrono>
#include <thread>
#include <cmath>

#include "../data/vector.h"
#include "../data/servo.h"
#include "../configuration/configuration.h"
#include "../consts.h"
#include "ik.h"

namespace mote
{
namespace walking
{
/*
class Ankle
{
public:
	Servo lateral;
	Servo frontal;
};

class Leg
{
public:
	Servo kneeLateral;
};

class Hip
{
public:
	Servo transversal;
	Servo lateral;
	Servo frontal;
};
*/

enum class RobotState
{
	FallenRight,
	FallenLeft,
	FallenBack,
	FallenFront,
	NormalStand
};

class Robot
{
private:
	data::Vector3d _velocity;
	float _velocityTheta;
	Configuration &configuration;
	HumanoidIK ik;
	sensors::IMU imu;
protected:
	RobotState getRobotState(double roll, double pitch);
public:
	void run();

	void init();

	void standInit(double speed);

	void standInitT(double speed, int time);

	void omniGait(double vx, double vy, double vt);

	void checkRobotState();
};
}
}


#endif //WALKING_ROBOT_H
