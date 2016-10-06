/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_ROBOT_H
#define WALKING_ROBOT_H

#include <chrono>
#include <thread>

#include "../data/vector.h"
#include "../data/servo.h"

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
class Robot
{
private:
	data::Vector3d _velocity;
	float _velocityTheta;
public:
// 	Hip rightHip;
// 	Hip leftHip;
//
// 	Leg rightLeg;
// 	Leg leftLeg;
//
// 	Ankle rightAnkle;
// 	Ankle leftAnkle;

	void run();

	void init();

	void standInit(double _Speed);

	void Stand_Init_T(double _Speed, int T);

	void Omni_Gait(double vx, double vy, double vt);
};
}
}


#endif //WALKING_ROBOT_H
