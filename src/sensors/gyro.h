/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_GYRO_H
#define WALKING_GYRO_H

#include "../data/angle.h"

namespace mote
{
namespace walking
{
namespace sensors
{
class Gyro
{
public:
	mote::walking::data::Angle3d data;
};
}
}
}


#endif //WALKING_GYRO_H
