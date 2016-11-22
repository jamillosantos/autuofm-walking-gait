/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_NETWORKING_SERVER_H
#define WALKING_NETWORKING_SERVER_H

#include <iostream>

#include <json/value.h>
#include <boost/asio/ip/udp.hpp>

#include "../consts.h"
#include "../data/json.h"

namespace mote
{
namespace walking
{
namespace networking
{

class ServiceException
	: public std::exception
{
private:
	std::string _msg;
public:
	ServiceException(std::string msg);
#ifdef LOOSER_THROW
	virtual ~ServiceException() throw() {};
#endif
	virtual std::string msg();
};
class InvalidMemberName
	: public ServiceException
{
public:
	std::string memberName;
	InvalidMemberName(std::string memberName);
#ifdef LOOSER_THROW
	virtual ~InvalidMemberName() throw() {};
#endif
};
class InvalidJSON
	: public ServiceException
{
public:
	InvalidJSON(std::string &json);
#ifdef LOOSER_THROW
	virtual ~InvalidJSON() throw() {};
#endif
};


class UDPServer
{
private:
	boost::asio::ip::udp::socket _socket;
	boost::asio::ip::udp::endpoint _remoteEP;
	std::unique_ptr<std::thread> _thread;
	volatile bool _running;
public:
	UDPServer(boost::asio::io_service &io_service, unsigned int port);

	void start();
	virtual void run();
	void stop();

protected:

	virtual int send(const char *msg, size_t len);

	virtual int send(std::string &msg);

	virtual int send(Json::Value &json);

	virtual void processCommand(std::string &command);

	virtual void processCommand(Json::Value &jsonCommand) = 0;

	virtual void handleInvalidJSON(std::string &command, mote::walking::networking::InvalidJSON &e);
};
}
}
}


#endif //WALKING_SERVER_H
