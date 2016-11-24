/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include <iostream>
#include "part.h"
#include "../consts.h"

mote::walking::Leg::Leg()
{ }

mote::walking::Leg::Leg(const mote::walking::Leg& leg)
	: position(leg.position), angle(leg.angle)
{ }

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

void mote::walking::HumanoidPart::dump()
{
	/*
	std::cout << "-- Head" << std::endl;
	std::cout << "     Pan  : " << this->head.position.pan << std::endl;
	std::cout << "     Tilt : " << this->head.position.tilt << std::endl;
	std::cout << "-- RightArm" << std::endl;
	std::cout << "     Pitch: " << this->rightArm.angle.pitch << "(" << this->rightArm.velocity.pitch << ")" << std::endl;
	std::cout << "     Roll : " << this->rightArm.angle.roll << "(" << this->rightArm.velocity.roll << ")" << std::endl;
	std::cout << "     Elbow: " << this->rightArm.elbow << "(" << this->rightArm.velocityElbow << ")" << std::endl;
	std::cout << "-- LeftArm" << std::endl;
	std::cout << "     Pitch: " << this->leftArm.angle.pitch << "(" << this->leftArm.velocity.pitch << ")" << std::endl;
	std::cout << "     Roll : " << this->leftArm.angle.roll << "(" << this->leftArm.velocity.roll << ")" << std::endl;
	std::cout << "     Elbow: " << this->leftArm.elbow << "(" << this->leftArm.velocityElbow << ")" << std::endl;
	*/
	VERBOSE("-- RightHip");
	VERBOSE("     Pitch: " << this->rightHip.angle.pitch << "(" << this->rightHip.velocity.pitch << ")");
	VERBOSE("     Pitch: " << (this->rightHip.angle.pitch * RAD2DEG));
	/*
	std::cout << "     Roll : " << this->rightHip.angle.roll << "(" << this->rightHip.velocity.roll << ")" << std::endl;
	std::cout << "     Yaw  : " << this->rightHip.angle.yaw << "(" << this->rightHip.velocity.yaw << ")" << std::endl;
	std::cout << "-- LeftHip" << std::endl;
	std::cout << "     Pitch: " << this->leftHip.angle.pitch << "(" << this->leftHip.velocity.pitch << ")" << std::endl;
	std::cout << "     Roll : " << this->leftHip.angle.roll << "(" << this->leftHip.velocity.roll << ")" << std::endl;
	std::cout << "     Yaw  : " << this->leftHip.angle.yaw << "(" << this->leftHip.velocity.yaw << ")" << std::endl;
	*/
	std::cout << "-- RightLeg" << std::endl;
	/*
	std::cout << "     Pitch: " << this->rightLeg.angle.pitch << std::endl;
	std::cout << "     Roll : " << this->rightLeg.angle.roll << std::endl;
	std::cout << "     Yaw  : " << this->rightLeg.angle.yaw << std::endl;
	*/
	std::cout << "     Pos  : (" << this->rightLeg.position.x << ", " << this->rightLeg.position.y << ", " << this->rightLeg.position.z << ")" << std::endl;
	std::cout << "-- LeftLeg" << std::endl;
	/*
	std::cout << "     Pitch: " << this->leftLeg.angle.pitch << std::endl;
	std::cout << "     Roll : " << this->leftLeg.angle.roll << std::endl;
	std::cout << "     Yaw  : " << this->leftLeg.angle.yaw << std::endl;
	*/
	std::cout << "     Pos  : (" << this->leftLeg.position.x << ", " << this->leftLeg.position.y << ", " << this->leftLeg.position.z << ")" << std::endl;
	/*
	std::cout << "-- RightKnee" << std::endl;
	std::cout << "     Angle: " << this->rightKnee.angle << std::endl;
	std::cout << "     Vel  : " << this->rightKnee.velocity << std::endl;
	std::cout << "-- LeftKnee" << std::endl;
	std::cout << "     Angle: " << this->leftKnee.angle << std::endl;
	std::cout << "     Vel  : " << this->leftKnee.velocity << std::endl;
	std::cout << "-- RightFoot" << std::endl;
	std::cout << "     Pitch: " << this->rightFoot.angle.pitch << "(" << this->rightFoot.velocity.pitch << ")" << std::endl;
	std::cout << "     Roll : " << this->rightFoot.angle.roll << "(" << this->rightFoot.velocity.roll << ")" << std::endl;
	std::cout << "-- LeftFoot" << std::endl;
	std::cout << "     Pitch: " << this->leftFoot.angle.pitch << "(" << this->leftFoot.velocity.pitch << ")" << std::endl;
	std::cout << "     Roll : " << this->leftFoot.angle.roll << "(" << this->leftFoot.velocity.roll << ")" << std::endl;
	*/
}
