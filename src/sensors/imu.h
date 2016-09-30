/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#ifndef WALKING_IMU_H
#define WALKING_IMU_H

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
	MPU &mpu;
	Gyro &gyro;

	IMU(MPU &mpu, Gyro &gyro);
};
}
}
}

#endif //WALKING_IMU_H
