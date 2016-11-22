/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 21, 2016
 */

#ifndef WALKING_DATA_JSON_H
#define WALKING_DATA_JSON_H

#include <json/json.h>

namespace mote
{
class json
{
public:
	static void serialize(const Json::Value &json, std::string &msg);

	static bool unserialize(char buffer[4096], unsigned int len, Json::Value &value);
};
}


#endif //WALKING_DATA_JSON_H
