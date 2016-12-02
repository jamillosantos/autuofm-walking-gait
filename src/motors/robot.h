/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_ROBOT_H
#define WALKING_ROBOT_H

#include <memory>

#include "../engine/part.h"
#include "servo.h"

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
	std::vector<Servo*> _servos;
protected:
	Servo* add(Servo* servo);
	void clear();
public:
	data::Vector3d com;

	data::Vector3d rightLegPosition;
	data::Vector3d leftLegPosition;

	data::Vector3d rightArmPosition;
	data::Vector3d leftArmPosition;

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

	virtual void init() = 0;

	virtual void update();

	std::vector<Servo*> &servos();
};
}
}
}

#endif //WALKING_ROBOT_H
