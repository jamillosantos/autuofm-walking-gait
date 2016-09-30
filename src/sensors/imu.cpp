/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 27, 2016
 */

#include "imu.h"

mote::walking::sensors::IMU::IMU(mote::walking::sensors::MPU &mpu, mote::walking::sensors::Gyro &gyro)
	: mpu(mpu), gyro(gyro)
{ }
