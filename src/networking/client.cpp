/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 18, 2016
 */

#include "client.h"

using boost::asio::ip::udp;

mote::walking::networking::UDPClient::UDPClient(std::string address, unsigned int port)
	: address(address), port(port), _defaultTimeout(boost::posix_time::milliseconds(1000)), io_service(),
	  socket(io_service, udp::endpoint(udp::v4(), 0)), deadlineTimer(io_service), resolver(io_service),
	  query(udp::v4(), address, std::to_string(port)), udpIterator(resolver.resolve(query))
{
	this->deadlineTimer.expires_at(boost::posix_time::pos_infin);
	this->checkDeadline();
}

int mote::walking::networking::UDPClient::send(std::string const &msg)
{
	const boost::asio::const_buffers_1 &bf = boost::asio::buffer(msg, msg.size());
	int
		sent = 0,
		remaining = msg.size(),
		ret;

	while (remaining > 0)
	{
		ret = this->socket.send_to(bf, *this->udpIterator);
		remaining -= ret;
		sent += ret;
	}
	return sent;
}

bool mote::walking::networking::UDPClient::send(Json::Value &json)
{
	std::string msg;
	json::serialize(json, msg);
	int ret = this->send(msg);
	return (ret > 0);
}

std::size_t mote::walking::networking::UDPClient::read(
	char *buffer, unsigned int bufferLength, boost::posix_time::time_duration timeout, boost::system::error_code &ec
)
{
	this->deadlineTimer.expires_from_now(timeout);

	ec = boost::asio::error::would_block;
	std::size_t length = 0;

	this->socket.async_receive(boost::asio::buffer(buffer, bufferLength),
		boost::bind(&UDPClient::handleReceive, boost::placeholders::_1, boost::placeholders::_2, &ec, &length));

	// Block until the asynchronous operation has completed.
	do
	{
		this->io_service.run_one();
	}
	while (ec == boost::asio::error::would_block);
}

bool mote::walking::networking::UDPClient::read(
	Json::Value &json, boost::posix_time::time_duration timeout, boost::system::error_code &ec
)
{
	char buffer[4096];
	unsigned int len = this->read(buffer, sizeof(buffer), timeout, ec);
	// VERBOSE(ec.category().name());
	if (!ec)
	{
		// VERBOSE("OK!");
		return json::unserialize(buffer, len, json);
	}
	else
	{
		// VERBOSE("EC Error: " << ec.message());
		return false;
	}
}

void mote::walking::networking::UDPClient::checkDeadline()
{
	if (this->deadlineTimer.expires_at() <= deadline_timer::traits_type::now())
	{
		this->socket.cancel();
		this->deadlineTimer.expires_at(boost::posix_time::pos_infin);
	}
	this->deadlineTimer.async_wait(boost::bind(&UDPClient::checkDeadline, this));
}

void mote::walking::networking::UDPClient::handleReceive(
	const boost::system::error_code &ec, std::size_t length, boost::system::error_code *out_ec, std::size_t *out_length
)
{
	*out_ec = ec;
	*out_length = length;
}

bool mote::walking::networking::UDPClient::read(Json::Value &json)
{
	boost::system::error_code ec;
	return this->read(json, this->_defaultTimeout, ec);
}

mote::walking::networking::UDPClientLocker::UDPClientLocker(UDPClient &client)
	: client(client)
{
	client.transactionMutex.lock();
}

mote::walking::networking::UDPClientLocker::~UDPClientLocker()
{
	client.transactionMutex.unlock();
}

const boost::posix_time::time_duration &mote::walking::networking::UDPClient::defaultTimeout() const
{
	return this->_defaultTimeout;
}

mote::walking::networking::UDPClient &
mote::walking::networking::UDPClient::defaultTimeout(boost::posix_time::time_duration duration)
{
	this->_defaultTimeout = duration;
	return *this;
}
