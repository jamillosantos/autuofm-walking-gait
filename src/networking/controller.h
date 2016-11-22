/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H

#include "../engine/robot.h"
#include "server.h"

namespace mote
{
namespace walking
{
namespace networking
{
class Controller
	: public UDPServer
{
private:
	Robot &_robot;
public:
	Controller(boost::asio::io_service &io_service, unsigned int port, Robot &_robot);

protected:
	virtual void processCommand(Json::Value &jsonCommand) override;
};
}
}
}


#endif //WALKING_CONTROLLER_H
