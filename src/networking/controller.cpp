/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#include "controller.h"

mote::walking::networking::Controller::Controller(boost::asio::io_service &io_service, unsigned int port,
	mote::walking::Robot &_robot)
	: UDPServer(io_service, port), _robot(_robot)
{ }

void mote::walking::networking::Controller::processCommand(Json::Value &jsonCommand)
{
	if (jsonCommand["command"].asString() == "omniwalk")
	{
		Json::Value &params = jsonCommand["params"];
		std::string name;
		for (Json::ValueConstIterator it = params.begin(); it != params.end(); ++it)
		{
			name = it.name();
			if (name == "x")
				this->_robot.velocity.x = it->asDouble();
			else if (name == "y")
				this->_robot.velocity.y = it->asDouble();
			else if (name == "theta")
				this->_robot.velocity.theta = it->asDouble();
			else
			{
				ERROR("Unknown property " << name << ".");
			}
		}
		VERBOSE("Velocity updated to " << this->_robot.velocity.x << ", " << this->_robot.velocity.y << ", " << this->_robot.velocity.theta);
	}
}
