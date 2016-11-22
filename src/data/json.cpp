/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#include "json.h"

void mote::json::serialize(const Json::Value &json, std::string &msg)
{
	Json::FastWriter writer;
	msg = writer.write(json);
}

bool mote::json::unserialize(char *buffer, unsigned int len, Json::Value &value)
{
	Json::Reader reader;
	return reader.parse(buffer, &buffer[len], value, false);
}
