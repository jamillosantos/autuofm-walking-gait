/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 28, 2016
 */

#ifndef WALKING_SENSORS_MPU_H
#define WALKING_SENSORS_MPU_H

#include "../data/vector.h"

namespace mote
{
namespace walking
{
namespace sensors
{
class MPU
{
public:
	data::Vector3d position;
};
}
}
}


#endif //WALKING_SENSORS_MPU_H
