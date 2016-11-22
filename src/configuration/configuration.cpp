/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#include <src/consts.h>
#include "configuration.h"

void mote::walking::Configuration::loadFromFile(const std::string &filePath)
{
	VERBOSE("Configuration::loadFromFile: " << filePath);
	boost::filesystem::path filePathPath(filePath);
	if (boost::filesystem::exists(filePathPath))
	{
		std::ifstream jsonStream(filePath);
		if (jsonStream.good())
		{
			Json::Value json;
			Json::Reader reader;
			reader.parse(jsonStream, json, false);
			std::string name;
			for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
			{
				name = it.name();
				if (name == "robot")
					this->robot = it->asString();
				else if (name == "walking")
					this->walking.fromJson(*it);
				else if (name == "simu")
				{
					this->simu.reset(new configuration::Server());
					this->simu->fromJson(*it);
				}
				else
				{
					// TODO: Throw an exception!
				}
			}
		}
		else
		{
			// TODO: Throw an exception.
		}
	}
	else
	{
		// TODO: Throw an exception.
	}
}

void mote::walking::configuration::Walking::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "head")
			this->head.fromJson(*it);
		else if (name == "minVoltage")
			this->minVoltage = it->asFloat();
		else if (name == "fall")
			this->fall.fromJson(*it);
		else if (name == "legLength")
			this->legLength = it->asDouble();
		else if (name == "velocityOffset")
			this->velocityOffset.fromJson(*it);
		else if (name == "engine")
			this->engine.fromJson(*it);
		else if (name == "stabilizer")
			this->stabilizer.fromJson(*it);
		else if (name == "gyroStabilizer")
			this->gyroStabilizer.fromJson(*it);
		else if (name == "hoppingGaitGain")
			this->hoppingGaitGain.fromJson(*it);
		else if (name == "com")
			this->com.fromJson(*it);
		else if (name == "rightLeg")
			this->rightLeg.fromJson(*it);
		else if (name == "leftLeg")
			this->leftLeg.fromJson(*it);
		else if (name == "rightArm")
			this->rightArm.fromJson(*it);
		else if (name == "leftArm")
			this->leftArm.fromJson(*it);
		else if (name == "imuOffset")
			this->imuOffset.fromJson(*it);
		else if (name == "gyroLowpassGain")
			this->gyroLowpassGain.fromJson(*it);
		else if (name == "kalmanRmRate")
			this->kalmanRmRate.fromJson(*it);
		else if (name == "smoothingRatio")
			this->smoothingRatio.fromJson(*it);
		else
		{
			// TODO: Throw an exception.
		}

	}
}

void mote::walking::configuration::Head::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "panSpeed")
			this->panSpeed = it->asDouble();
		else if (name == "tiltSpeed")
			this->tiltSpeed = it->asDouble();
		else
		{
			// TODO: Throw an exception.
		}
	}
}

void mote::walking::configuration::Fall::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "pitchThreshold")
			this->pitchThreshold = it->asDouble();
		else if (name == "rollThreshold")
			this->rollThreshold = it->asDouble();
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Engine::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "motionResolution")
			this->motionResolution = it->asDouble();
		else if (name == "gaitFrequency")
			this->gaitFrequency = it->asDouble();
		else if (name == "doubleSupportSleep")
			this->doubleSupportSleep = it->asDouble();
		else if (name == "singleSupportSleep")
			this->singleSupportSleep = it->asDouble();
		else if (name == "flyGain")
			this->flyGain.fromJson(*it);
		else if (name == "flySwingGain")
			this->flySwingGain.fromJson(*it);
		else if (name == "supportGain")
			this->supportGain.fromJson(*it);
		else if (name == "supportSwingGain")
			this->supportSwingGain.fromJson(*it);
		else if (name == "bodySwingGain")
			this->bodySwingGain.fromJson(*it);
		else
		{
			// TODO: Throw an exception.
		}
	}
}

void mote::walking::configuration::Stabilizer::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "armGain")
			this->armGain.fromJson(*it);
		else if (name == "armElbowGain")
			this->armElbowGain = it->asDouble();
		else if (name == "hipGain")
			this->hipGain.fromJson(*it);
		else if (name == "kneeGain")
			this->kneeGain = it->asDouble();
		else if (name == "footGain")
			this->footGain.fromJson(*it);
		else if (name == "comShiftGain")
			this->comShiftGain.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::BodyCom::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (name == "positionOffset")
			this->positionOffset.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Leg::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (name == "positionOffset")
			this->positionOffset.fromJson(*it);
		if (name == "hipAngleOffset")
			this->hipAngleOffset.fromJson(*it);
		else if (name == "kneeOffset")
			this->kneeOffset = it->asDouble();
		else if (name == "footOffset")
			this->footOffset.fromJson(*it);
		else if (name == "positionOffset")
			this->positionOffset.fromJson(*it);
		else if (name == "angleOffset")
			this->angleOffset.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Arm::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (name == "elbowOffset")
			this->elbowOffset = it->asDouble();
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Server::fromJson(const Json::Value &json)
{
	std::string name;
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		name = it.name();
		if (name == "address")
			this->address = it->asString();
		else if (name == "port")
			this->port = it->asUInt();
		else
		{
			// TODO: Throw an exception
		}
	}
}
