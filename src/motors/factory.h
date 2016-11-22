/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#ifndef WALKING_FACTORY_H
#define WALKING_FACTORY_H

#include "servo.h"

namespace mote
{
namespace walking
{
namespace motors
{
class Factory
{
public:
	virtual Servo* create() = 0;
};
}
}
}


#endif //WALKING_FACTORY_H
