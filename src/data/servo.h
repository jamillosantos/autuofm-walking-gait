/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#ifndef WALKING_DATA_SERVO_H
#define WALKING_DATA_SERVO_H

namespace mote
{
namespace walking
{
class Servo
{
public:
	Servo();

	Servo(float angle);

	float angle;
};
}
}


#endif //WALKING_SERVO_H
