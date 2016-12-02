/**
 * @author J. Santos <jamillo@gmail.com>
 * @date November 22, 2016
 */

#include "simurobot.h"
#include "simuservo.h"

mote::walking::motors::SimuRobot::SimuRobot(mote::walking::HumanoidPart &humanoidPart)
	: Robot(humanoidPart)
{ }

void mote::walking::motors::SimuRobot::init()
{
	this->rightHip.transversal.reset(this->add(new SimuServo("RightHipTransversal", true)));
	this->rightHip.frontal.reset(this->add(new SimuServo("RightHipFrontal")));
	this->rightHip.lateral.reset(this->add(new SimuServo("RightHipLateral", true)));
	this->rightLeg.kneeLateral.reset(this->add(new SimuServo("RightKneeLateral", true)));
	this->rightFoot.lateral.reset(this->add(new SimuServo("RightFootLateral", true)));
	this->rightFoot.frontal.reset(this->add(new SimuServo("RightFootFrontal")));

	this->leftHip.transversal.reset(this->add(new SimuServo("LeftHipTransversal")));
	this->leftHip.frontal.reset(this->add(new SimuServo("LeftHipFrontal", true)));
	this->leftHip.lateral.reset(this->add(new SimuServo("LeftHipLateral")));
	this->leftLeg.kneeLateral.reset(this->add(new SimuServo("LeftKneeLateral")));
	this->leftFoot.lateral.reset(this->add(new SimuServo("LeftFootLateral")));
	this->leftFoot.frontal.reset(this->add(new SimuServo("LeftFootFrontal", true)));

	this->neck.lateral.reset(new SimuServo("NeckLateral"));
	this->neck.transversal.reset(new SimuServo("NeckTransversal"));

	this->rightArm.elbow.reset(this->add(new SimuServo("RightElbowLateral", true)));
	this->rightArm.shoulderLateral.reset(this->add(new SimuServo("RightShoulderLateral", true)));
	this->rightArm.shoulderFrontal.reset(this->add(new SimuServo("RightShoulderFrontal", true)));

	this->leftArm.elbow.reset(this->add(new SimuServo("LeftElbowLateral")));
	this->leftArm.shoulderLateral.reset(this->add(new SimuServo("LeftShoulderLateral")));
	this->leftArm.shoulderFrontal.reset(this->add(new SimuServo("LeftShoulderFrontal")));
}
