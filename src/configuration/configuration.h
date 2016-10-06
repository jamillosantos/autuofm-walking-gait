/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_CONFIGURATION_H
#define WALKING_CONFIGURATION_H

#include "../data/vector.h"
#include "../data/angle.h"

namespace mote
{
namespace walking
{
namespace configuration
{

using namespace mote::walking::data;

class Head
{
public:
	double panSpeed;
	double tiltSpeed;
};

class Fall
{
public:
	double rollThreshold;
	double pitchThreshold;
};

class WalkEngine
{
public:
	double motionResolution;
	double gaitFrequency;
	double doubleSupportSleep;
	double singleSupportSleep;

	Angle3d flyGain;
	Vector3d flySwingGain;
	Angle3d supportGain;
	Vector3d supportSwingGain;
	Vector3d bodySwingGain;
};

class Stabilizer
{
public:
	Angle2d armGain;

	double armElbowGain;

	Angle2d hipGain;
	double kneeGain;

	Angle2d footGain;

	Vector2d comShiftGain;
};

class BodyCom
{
public:
	Vector3d positionOffset;
	Angle3d angleOffset;
};

class Leg
{
public:
	Angle3d hipAngleOffset;
	double kneeOffset;
	Angle2d footOffset;

	Vector3d positionOffset;
	Angle3d angleOffset;
};

class Arm
{
public:
	Angle2d angleOffset;
	double elbowOffset;
};

class Walking
{
public:
	Head head;

	float minVoltage;

	Fall fall;

	double legLength;

	Vector3d velocity; // = Vector3d(-0.07, 0.0, 0.001);

	WalkEngine walkEngine;

	Stabilizer stabilizer;
	Stabilizer gyroStabilizer;

	Vector2d hoppingGaitGain;

	BodyCom com;

	Leg rightLeg;
	Leg leftLeg;

	Arm rightArm;
	Arm leftArm;

	Vector2d imuOffset;
	Vector2d gyroLowpassGain;

	Angle3d kalmanRmRate;

	Vector3d smoothingRatio;
};
}

class Configuration
{
public:
	configuration::Walking walking;
};
}
}

#endif //WALKING_CONFIGURATION_H
