/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#include "configuration.h"

void mote::walking::Configuration::loadFromFile(const std::string filePath)
{
	if (boost::filesystem::exists(filePath))
	{
		if (boost::filesystem::is_regular(filePath))
		{
			std::ifstream jsonStream(filePath);
			if (jsonStream.good())
			{
				Json::Value json;
				Json::Reader reader;
				reader.parse(jsonStream, json, false);
				Json::Value &walking = json["walking"];
				if (!walking.isNull())
				{
					this->walking.fromJson(walking);
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
	else
	{
		// TODO: Throw an exception.
	}
}

void mote::walking::configuration::Walking::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "head")
			this->head.fromJson(*it);
		else if (it.name() == "minVoltage")
			this->minVoltage = it->asFloat();
		else if (it.name() == "fall")
			this->fall.fromJson(*it);
		else if (it.name() == "legLength")
			this->legLength = it->asDouble();
		else if (it.name() == "velocityOffset")
			this->velocityOffset.fromJson(*it);
		else if (it.name() == "engine")
			this->engine.fromJson(*it);
		else if (it.name() == "stabilizer")
			this->stabilizer.fromJson(*it);
		else if (it.name() == "gyroStabilizer")
			this->gyroStabilizer.fromJson(*it);
		else if (it.name() == "hoppingGaitGain")
			this->hoppingGaitGain.fromJson(*it);
		else if (it.name() == "com")
			this->com.fromJson(*it);
		else if (it.name() == "rightLeg")
			this->rightLeg.fromJson(*it);
		else if (it.name() == "leftLeg")
			this->leftLeg.fromJson(*it);
		else if (it.name() == "rightArm")
			this->rightArm.fromJson(*it);
		else if (it.name() == "leftArm")
			this->leftArm.fromJson(*it);
		else if (it.name() == "imuOffset")
			this->imuOffset.fromJson(*it);
		else if (it.name() == "gyroLowpassGain")
			this->gyroLowpassGain.fromJson(*it);
		else if (it.name() == "kalmanRmRate")
			this->kalmanRmRate.fromJson(*it);
		else if (it.name() == "smoothingRatio")
			this->smoothingRatio.fromJson(*it);
		else
		{
			// TODO: Throw an exception.
		}

	}
}

void mote::walking::configuration::Head::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "panSpeed")
			this->panSpeed = it->asDouble();
		else if (it.name() == "tiltSpeed")
			this->tiltSpeed = it->asDouble();
		else
		{
			// TODO: Throw an exception.
		}
	}
}

void mote::walking::configuration::Fall::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "pitchThreshold")
			this->pitchThreshold = it->asDouble();
		else if (it.name() == "rollThreshold")
			this->rollThreshold = it->asDouble();
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Engine::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "motionResolution")
			this->motionResolution = it->asDouble();
		else if (it.name() == "gaitFrequency")
			this->gaitFrequency = it->asDouble();
		else if (it.name() == "doubleSupportSleep")
			this->doubleSupportSleep = it->asDouble();
		else if (it.name() == "singleSupportSleep")
			this->singleSupportSleep = it->asDouble();
		else if (it.name() == "flyGain")
			this->flyGain.fromJson(*it);
		else if (it.name() == "flySwingGain")
			this->flySwingGain.fromJson(*it);
		else if (it.name() == "supportGain")
			this->supportGain.fromJson(*it);
		else if (it.name() == "supportSwingGain")
			this->supportSwingGain.fromJson(*it);
		else if (it.name() == "bodySwingGain")
			this->bodySwingGain.fromJson(*it);
		else
		{
			// TODO: Throw an exception.
		}
	}
}

void mote::walking::configuration::Stabilizer::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "armGain")
			this->armGain.fromJson(*it);
		else if (it.name() == "armElbowGain")
			this->armElbowGain = it->asDouble();
		else if (it.name() == "hipGain")
			this->hipGain.fromJson(*it);
		else if (it.name() == "kneeGain")
			this->kneeGain = it->asDouble();
		else if (it.name() == "footGain")
			this->footGain.fromJson(*it);
		else if (it.name() == "comShiftGain")
			this->comShiftGain.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::BodyCom::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (it.name() == "positionOffset")
			this->positionOffset.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Leg::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (it.name() == "positionOffset")
			this->positionOffset.fromJson(*it);
		if (it.name() == "hipAngleOffset")
			this->hipAngleOffset.fromJson(*it);
		else if (it.name() == "kneeOffset")
			this->kneeOffset = it->asDouble();
		else if (it.name() == "footOffset")
			this->footOffset.fromJson(*it);
		else if (it.name() == "positionOffset")
			this->positionOffset.fromJson(*it);
		else if (it.name() == "angleOffset")
			this->angleOffset.fromJson(*it);
		else
		{
			// TODO: Throw an exception
		}
	}
}

void mote::walking::configuration::Arm::fromJson(const Json::Value &json)
{
	for (Json::ValueConstIterator it = json.begin(); it != json.end(); ++it)
	{
		if (it.name() == "angleOffset")
			this->angleOffset.fromJson(*it);
		else if (it.name() == "elbowOffset")
			this->elbowOffset = it->asDouble();
		else
		{
			// TODO: Throw an exception
		}
	}
}
