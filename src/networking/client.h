/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 18, 2016
 */

#ifndef WALKING_NETWORKING_CLIENT_H
#define WALKING_NETWORKING_CLIENT_H

#include <string>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/bind.hpp>
#include <json/value.h>
#include <mutex>

#include <src/data/json.h>

using boost::asio::ip::udp;
using boost::asio::deadline_timer;

namespace mote
{
namespace walking
{
namespace networking
{
class UDPClient;

class UDPClientLocker
{
private:
	UDPClient &client;
public:
	UDPClientLocker(UDPClient &client);

	~UDPClientLocker();
};

class UDPClient
{

	friend class UDPClientLocker;

private:
	std::string address;
	unsigned int port;

	boost::posix_time::time_duration _defaultTimeout;

	boost::asio::io_service io_service;
	udp::socket socket;
	deadline_timer deadlineTimer;

	udp::resolver resolver;
	udp::resolver::query query;
	udp::resolver::iterator udpIterator;

protected:
	std::mutex transactionMutex;

	virtual int send(std::string const &msg);

	virtual bool send(Json::Value &json);

	virtual std::size_t read(char *buffer, unsigned int bufferLength, boost::posix_time::time_duration timeout,
							 boost::system::error_code &ec);

	virtual bool read(Json::Value &json, boost::posix_time::time_duration timeout, boost::system::error_code &ec);

	virtual bool read(Json::Value &json);

	virtual void checkDeadline();

	static void handleReceive(
		const boost::system::error_code &ec, std::size_t length, boost::system::error_code *out_ec,
		std::size_t *out_length
	);

public:
	UDPClient(std::string address, unsigned int port);

	const boost::posix_time::time_duration &defaultTimeout() const;

	UDPClient &defaultTimeout(boost::posix_time::time_duration duration);
};
}
}
}

#endif //WALKING_NETWORKING_CLIENT_H
