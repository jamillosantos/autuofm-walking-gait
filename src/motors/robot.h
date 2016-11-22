/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_ROBOT_H
#define WALKING_ROBOT_H

#include <memory>
#include <src/engine/part.h>
#include "servo.h"
#include "factory.h"

namespace mote
{
namespace walking
{
namespace motors
{
namespace robot
{
class Neck
{
public:
	std::unique_ptr<Servo> lateral;
	std::unique_ptr<Servo> transversal;

	Neck(Servo* lateral, Servo* transversal);

	Neck();
};

class Arm
{
public:
	std::unique_ptr<Servo> shoulderLateral;
	std::unique_ptr<Servo> shoulderFrontal;
	std::unique_ptr<Servo> elbow;

	Arm(Servo* shoulderLateral, Servo* shoulderFrontal, Servo* elbow);

	Arm();
};

class Hip
{
public:
	std::unique_ptr<Servo> lateral;
	std::unique_ptr<Servo> frontal;
	std::unique_ptr<Servo> transversal;

	Hip(Servo* lateral, Servo* frontal, Servo* transversal);

	Hip();
};

class Leg
{
public:
	std::unique_ptr<Servo> kneeLateral;

	Leg(Servo* kneeLateral);

	Leg();
};

class Foot
{
public:
	std::unique_ptr<Servo> lateral;
	std::unique_ptr<Servo> frontal;

	Foot(Servo* lateral, Servo* frontal);

	Foot();
};
}

class Robot
{
private:
	HumanoidPart &_humanoidPart;
public:
	robot::Neck neck;

	robot::Arm rightArm;
	robot::Arm leftArm;

	robot::Hip rightHip;
	robot::Hip leftHip;

	robot::Leg rightLeg;
	robot::Leg leftLeg;

	robot::Foot rightFoot;
	robot::Foot leftFoot;

	Robot(HumanoidPart &humanoidPart);

	virtual void init(Factory &factory);

	virtual void update();
};
}
}
}

#endif //WALKING_ROBOT_H
