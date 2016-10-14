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
	this->angle.zero();
	this->velocity.zero();

	this->elbow = this->velocityElbow = 0;
}

void mote::walking::Arm::zero(double velocity)
{
	this->elbow = 0;
	this->angle.zero();
	this->velocityElbow = velocity;
	this->velocity.set(velocity);
}

void mote::walking::Head::zero()
{
	this->position.pan = this->position.tilt = this->velocity.pan = this->velocity.tilt = 0;
}

void mote::walking::Hip::zero()
{
	this->velocity.zero();
	this->angle.zero();
}

void mote::walking::Knee::zero()
{
	this->angle = this->velocity = 0;
}

void mote::walking::Foot::zero()
{
	this->angle.zero();
	this->velocity.zero();
}

void mote::walking::HumanoidPart::zero()
{
	this->head.zero();
	this->rightArm.zero();
	this->leftArm.zero();
	this->rightLeg.zero();
	this->leftLeg.zero();
	this->rightHip.zero();
	this->leftHip.zero();
	this->rightKnee.zero();
	this->leftKnee.zero();
	this->rightFoot.zero();
	this->leftFoot.zero();
}
