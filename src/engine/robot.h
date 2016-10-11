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

#define No_Motion                    0x64
#define Stop_Motion                  0x70

//internal motion number value
#define Stand_Up_Front               1
#define Stand_Up_Back                2
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
	data::Vector2td _velocity;
	Configuration &configuration;
	Humanoid humanoid;
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

	/**
	 * Updates internal head value.
	 *
	 * @param pan
	 * @param tilt
	 * @param panSpeed
	 * @param tiltSpeed
	 */
	void setHead(double pan, double tilt, double panSpeed, double tiltSpeed);

	int Check_Robot_Fall;
	/**
	 * The motion request
	 */
	int Motion_Ins;

	int System_Voltage;
	int Internal_Motion_Request;
	int Actuators_Update;
};
}
}


#endif //WALKING_ROBOT_H
