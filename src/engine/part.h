/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#ifndef WALKING_PART_H
#define WALKING_PART_H

#include <exception>
#include "../data/vector.h"

namespace mote
{
namespace walking
{

class Part
{
public:
	virtual void zero() = 0;
};

class Leg
	: public Part
{
public:
	data::Vector3d position;

	double pitch;
	double roll;
	double yaw;

	virtual void zero() override;
};

class Arm
	: public Part
{
public:
	double pitch;
	double roll;
	double elbow;

	double velocityPitch;
	double velocityRoll;
	double velocityElbow;

	virtual void zero() override;
};

class Head
	: public Part
{
public:
	virtual void zero() override;
};

class Humanoid:
	public Part
{
public:
	Head head;
	Arm rightArm;
	Arm leftArm;
	Leg rightLeg;
	Leg leftLeg;
};

}
}

#endif //WALKING_PART_H