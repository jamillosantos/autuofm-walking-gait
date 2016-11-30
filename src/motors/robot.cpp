/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#include "robot.h"


mote::walking::motors::robot::Neck::Neck(mote::walking::motors::Servo *lateral, mote::walking::motors::Servo *transversal)
	: lateral(lateral), transversal(transversal)
{ }

mote::walking::motors::robot::Neck::Neck()
{ }

mote::walking::motors::robot::Arm::Arm(Servo* shoulderLateral, Servo* shoulderFrontal, Servo* elbow)
	: shoulderLateral(shoulderLateral), shoulderFrontal(shoulderFrontal), elbow(elbow)
{ }

mote::walking::motors::robot::Arm::Arm()
{ }

mote::walking::motors::robot::Hip::Hip(mote::walking::motors::Servo *lateral, mote::walking::motors::Servo *frontal,
	mote::walking::motors::Servo *transversal)
	: lateral(lateral), frontal(frontal), transversal(transversal)
{ }

mote::walking::motors::robot::Hip::Hip()
{ }

mote::walking::motors::robot::Leg::Leg(mote::walking::motors::Servo *kneeLateral)
	: kneeLateral(kneeLateral)
{ }

mote::walking::motors::robot::Leg::Leg()
{ }

mote::walking::motors::robot::Foot::Foot(mote::walking::motors::Servo *lateral, mote::walking::motors::Servo *frontal)
	: lateral(lateral), frontal(frontal)
{ }

mote::walking::motors::robot::Foot::Foot()
{ }

mote::walking::motors::Robot::Robot(mote::walking::HumanoidPart &humanoidPart)
	: _humanoidPart(humanoidPart)
{ }

mote::walking::motors::Servo *mote::walking::motors::Robot::add(Servo* servo)
{
	this->_servos.push_back(servo);
	return servo;
}

void mote::walking::motors::Robot::update()
{
	this->rightLegPosition = this->_humanoidPart.rightLegPosition;
	this->leftLegPosition = this->_humanoidPart.leftLegPosition;

	this->neck.lateral->angle = this->_humanoidPart.head.position.tilt;
	this->neck.transversal->angle = this->_humanoidPart.head.position.pan;

	this->rightArm.elbow->angle = this->_humanoidPart.rightArm.elbow;
	this->rightArm.elbow->speed = this->_humanoidPart.rightArm.velocityElbow;
	this->rightArm.shoulderLateral->angle = this->_humanoidPart.rightArm.angle.pitch;
	this->rightArm.shoulderFrontal->angle = this->_humanoidPart.rightArm.angle.roll;
	this->rightArm.shoulderLateral->speed = this->_humanoidPart.rightArm.velocity.pitch;
	this->rightArm.shoulderFrontal->speed = this->_humanoidPart.rightArm.velocity.roll;

	this->leftArm.elbow->angle = this->_humanoidPart.leftArm.elbow;
	this->leftArm.elbow->speed = this->_humanoidPart.leftArm.velocityElbow;
	this->leftArm.shoulderLateral->angle = this->_humanoidPart.leftArm.angle.pitch;
	this->leftArm.shoulderFrontal->angle = this->_humanoidPart.leftArm.angle.roll;
	this->leftArm.shoulderLateral->speed = this->_humanoidPart.leftArm.velocity.pitch;
	this->leftArm.shoulderFrontal->speed = this->_humanoidPart.leftArm.velocity.roll;

	this->rightHip.lateral->angle = this->_humanoidPart.rightHip.angle.pitch;
	this->rightHip.frontal->angle = this->_humanoidPart.rightHip.angle.roll;
	this->rightHip.transversal->angle = this->_humanoidPart.rightHip.angle.yaw;
	this->rightHip.lateral->speed = this->_humanoidPart.rightHip.velocity.pitch;
	this->rightHip.frontal->speed = this->_humanoidPart.rightHip.velocity.roll;
	this->rightHip.transversal->speed = this->_humanoidPart.rightHip.velocity.yaw;

	this->leftHip.lateral->angle = this->_humanoidPart.leftHip.angle.pitch;
	this->leftHip.frontal->angle = this->_humanoidPart.leftHip.angle.roll;
	this->leftHip.transversal->angle = this->_humanoidPart.leftHip.angle.yaw;
	this->leftHip.lateral->speed = this->_humanoidPart.leftHip.velocity.pitch;
	this->leftHip.frontal->speed = this->_humanoidPart.leftHip.velocity.roll;
	this->leftHip.transversal->speed = this->_humanoidPart.leftHip.velocity.yaw;

	this->rightLeg.kneeLateral->angle = this->_humanoidPart.rightKnee.angle;
	this->rightLeg.kneeLateral->speed = this->_humanoidPart.rightKnee.velocity;

	this->leftLeg.kneeLateral->angle = this->_humanoidPart.leftKnee.angle;
	this->leftLeg.kneeLateral->speed = this->_humanoidPart.leftKnee.velocity;

	this->rightFoot.lateral->angle = this->_humanoidPart.rightFoot.angle.pitch;
	this->rightFoot.frontal->angle = this->_humanoidPart.rightFoot.angle.roll;
	this->rightFoot.lateral->speed = this->_humanoidPart.rightFoot.velocity.pitch;
	this->rightFoot.frontal->speed = this->_humanoidPart.rightFoot.velocity.roll;

	this->leftFoot.lateral->angle = this->_humanoidPart.leftFoot.angle.pitch;
	this->leftFoot.frontal->angle = this->_humanoidPart.leftFoot.angle.roll;
	this->leftFoot.lateral->speed = this->_humanoidPart.leftFoot.velocity.pitch;
	this->leftFoot.frontal->speed = this->_humanoidPart.leftFoot.velocity.roll;
}

std::vector<mote::walking::motors::Servo*>& mote::walking::motors::Robot::servos()
{
	return this->_servos;
}

void mote::walking::motors::Robot::clear()
{
	this->_servos.clear();
}
