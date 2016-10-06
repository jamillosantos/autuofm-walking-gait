/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#ifndef WALKING_PART_H
#define WALKING_PART_H

#include <exception>
#include "../data/vector.h"
#include "../data/angle.h"

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
	data::Angle3d angle;

	virtual void zero() override;
};

class Arm
	: public Part
{
public:
	data::Angle3d angle;
	data::Angle3d velocity;

	virtual void zero() override;
	virtual void zero(double velocity);
};

class Head
	: public Part
{
public:
	virtual void zero() override;
};

class Hip
	: public Part
{
public:
	data::Angle3d angle;
	data::Angle3d velocity;
};

class Knee
	: public Part
{
public:
	double velocity;
	double angle;
};

class Foot
	: public Part
{
public:
	data::Angle2d angle;
	data::Angle2d velocity;
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
	Hip rightHip;
	Hip leftHip;
	Knee rightKnee;
	Knee leftKnee;
	Foot rightFoot;
	Foot leftFoot;
};

}
}

#endif //WALKING_PART_H