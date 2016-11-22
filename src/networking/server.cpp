/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#include <thread>
#include "server.h"

using boost::asio::ip::udp;

mote::walking::networking::ServiceException::ServiceException(std::string msg)
	: _msg(msg)
{ }
std::string mote::walking::networking::ServiceException::msg()
{
	return this->_msg;
}
mote::walking::networking::InvalidMemberName::InvalidMemberName(std::string memberName)
	: ServiceException("Invalid member name '" + memberName + '"'), memberName(memberName)
{ }
mote::walking::networking::InvalidJSON::InvalidJSON(std::string &json)
	: ServiceException("Invalid JSON: " + json)
{ }

mote::walking::networking::UDPServer::UDPServer(boost::asio::io_service &io_service, unsigned int port)
	: _socket(io_service, udp::endpoint(udp::v4(), port)), _running(false)
{ }

void mote::walking::networking::UDPServer::start()
{
	if (!this->_thread)
		this->_thread.reset(new std::thread(&UDPServer::run, this));
}

void mote::walking::networking::UDPServer::stop()
{
	VERBOSE("Stopping UDPServer...");
	if (this->_thread)
	{
		this->_running = false;
		this->_socket.cancel();
		this->_thread->join();
	}
}

int mote::walking::networking::UDPServer::send(const char *msg, size_t len)
{
	std::string message(msg, len);
	return this->send(message);
}

int mote::walking::networking::UDPServer::send(std::string &msg)
{
	const boost::asio::const_buffers_1 &bf = boost::asio::buffer(msg, msg.size());
	int
		sent = 0,
		remaining = msg.size(),
		ret;

	VERBOSE("Command sent: " << msg);
	while (remaining > 0)
	{
		ret = this->_socket.send_to(bf, this->_remoteEP);
		remaining -= ret;
		sent += ret;
	}
	return sent;
}

int mote::walking::networking::UDPServer::send(Json::Value &json)
{
	std::string response;
	json::serialize(json, response);
	return this->send(response);
}

void mote::walking::networking::UDPServer::processCommand(std::string &command)
{
	Json::Value json;
	if (json::unserialize(command, json))
	{
		this->processCommand(json);
	}
	else
	{
		ERROR("Invalid json input: " << command);
		throw mote::walking::networking::InvalidJSON(command);
	}
}

void mote::walking::networking::UDPServer::handleInvalidJSON(std::string &command, mote::walking::networking::InvalidJSON &e)
{
	ERROR("Cannot parse: " << command);
	std::string message("{\"success\":false,\"message\":\"Error parsing request. It is not a valid JSON.\"}");
	this->send(message);
}

void mote::walking::networking::UDPServer::run()
{
	char data[4096];
	size_t maxLength = sizeof(data);

	boost::system::error_code ec;

	this->_running = true;
	while (this->_running)
	{
		size_t len = this->_socket.receive_from(boost::asio::buffer(data, maxLength), this->_remoteEP, 0, ec);
		if (ec)
		{
			VERBOSE("[" << ec.category().name() << "] " << ec.message());
			if (ec == boost::system::errc::interrupted)
			{
				this->_running = false;
			}
		}
		else
		{
			std::string command(data, len);
			VERBOSE("Received command (" << len << "): " << command);
			try
			{
				this->processCommand(command);
			}
			catch (mote::walking::networking::InvalidJSON &e)
			{
				this->handleInvalidJSON(command, e);
			}
		}
	}
}
