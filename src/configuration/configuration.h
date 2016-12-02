/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_CONFIGURATION_H
#define WALKING_CONFIGURATION_H

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include <json/reader.h>
#include <json/value.h>
#include <boost/optional/optional.hpp>

#include "../data/vector.h"
#include "../data/angle.h"

#include "../consts.h"

namespace mote
{
namespace walking
{
namespace configuration
{

using namespace mote::walking::data;

class ConfigPart
{
public:
	virtual void fromJson(const Json::Value &json) = 0;
};

class Head
	: public ConfigPart
{
public:
	double panSpeed;
	double tiltSpeed;

	virtual void fromJson(const Json::Value &json) override;
};

class Fall
	: public ConfigPart
{
public:
	double rollThreshold;
	double pitchThreshold;

	virtual void fromJson(const Json::Value &json) override;
};

class Engine
	: public ConfigPart
{
public:
	double motionResolution;
	double gaitFrequency;
	unsigned int doubleSupportSleep;
	unsigned int singleSupportSleep;

	Angle3d flyGain;
	Vector3d flySwingGain;
	Angle3d supportGain;
	Vector3d supportSwingGain;
	Vector3d bodySwingGain;

	virtual void fromJson(const Json::Value &json) override;
};

class Stabilizer
	: public ConfigPart
{
public:
	Angle2d armGain;

	double armElbowGain;

	Angle2d hipGain;
	double kneeGain;

	Angle2d footGain;

	Vector2d comShiftGain;

	virtual void fromJson(const Json::Value &json) override;
};

class BodyCom
	: public ConfigPart
{
public:
	Vector3d positionOffset;
	Angle3d angleOffset;

	virtual void fromJson(const Json::Value &json) override;
};

class Leg
	: public ConfigPart
{
public:
	Angle3d hipAngleOffset;
	double kneeOffset;
	Angle2d footOffset;

	Vector3d positionOffset;
	Angle3d angleOffset;

	virtual void fromJson(const Json::Value &json) override;
};

class Arm
	: public ConfigPart
{
public:
	Angle2d angleOffset;
	double elbowOffset;

	Vector3d positionOffset;

	virtual void fromJson(const Json::Value &json) override;
};

class Walking
	: public ConfigPart
{
public:
	Head head;

	float minVoltage;

	Fall fall;

	double legLength;
	double armSwingLength;

	Vector2td velocityOffset; // = Vector3d(-0.07, 0.0, 0.001);

	Engine engine;

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

	virtual void fromJson(const Json::Value &json) override;
};

class Server
	: public configuration::ConfigPart
{
public:
	std::string address;
	unsigned int port;

	virtual void fromJson(const Json::Value &json) override;
};
}

class Configuration
{
public:
	std::string robot;
	configuration::Walking walking;
	std::unique_ptr<configuration::Server> simu;
	std::unique_ptr<configuration::Server> controller;

	void loadFromFile(const std::string &filePath);
};
}
}

#endif //WALKING_CONFIGURATION_H
