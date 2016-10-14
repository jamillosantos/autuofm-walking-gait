/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_SENSORS_IMU_H
#define WALKING_SENSORS_IMU_H

#include "mpu.h"
#include "gyro.h"

namespace mote
{
namespace walking
{
namespace sensors
{
class IMU
{
public:
	MPU mpu;
	Gyro gyro;

	IMU();
};
}
}
}

#endif //WALKING_SENSORS_IMU_H
