/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#include <src/consts.h>
#include "simumotorupdater.h"
#include "simuservo.h"

mote::walking::motors::SimuMotorUpdater::SimuMotorUpdater(mote::walking::networking::UDPClient &client,
	mote::walking::HumanoidPart &humanoidPart)
	: MotorUpdater(humanoidPart), _client(client), _robot(humanoidPart)
{ }

void mote::walking::motors::SimuMotorUpdater::updateState()
{
	this->_robot.update();
}

void mote::walking::motors::SimuMotorUpdater::flushChanges()
{
	Json::Value json;
	json["command"] = "update";
	Json::Value &params = json["params"];
	SimuServo* simuServo;
	for (Servo* servo : this->_robot.servos())
	{
		simuServo = static_cast<SimuServo*>(servo);
		if (simuServo)
		{
			Json::Value &jsonServo = params[simuServo->name];
			if (simuServo->angle.is_dirty())
				jsonServo["Angle"] = (*simuServo->angle.toWrite()) * RAD2DEG;
			if (simuServo->speed.is_dirty())
				jsonServo["Speed"] = (*simuServo->speed.toWrite()) * (1023);
		}
	}
	this->_client.send(json);
}

void mote::walking::motors::SimuMotorUpdater::init()
{
	this->_robot.init();
}
