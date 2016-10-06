/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include "part.h"

void mote::walking::Leg::zero()
{
	this->position.zero();
	this->angle.zero();
}

void mote::walking::Arm::zero()
{
	this->elbow = this->pitch = this->elbow = 0;
	this->velocityElbow = this->velocityPitch = this->velocityElbow = 0;
}

void mote::walking::Arm::zero(double velocity)
{
	this->elbow = this->pitch = this->elbow = 0;
	this->velocityElbow = this->velocityPitch = this->velocityElbow = velocity;
}

void mote::walking::Head::zero()
{
	// TODO
	throw std::exception();
}
