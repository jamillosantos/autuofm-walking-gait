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
	this->neck.lateral.reset(new SimuServo("NeckLateral"));
	this->neck.transversal.reset(new SimuServo("NeckTransversal"));

	this->rightArm.elbow.reset(new SimuServo("RightElbow"));
	this->rightArm.shoulderLateral.reset(this->add(new SimuServo("RightShoulderLateral")));
	this->rightArm.shoulderFrontal.reset(this->add(new SimuServo("RightShoulderFrontal")));

	this->leftArm.elbow.reset(new SimuServo("LeftElbow", true));
	this->leftArm.shoulderLateral.reset(this->add(new SimuServo("LeftShoulderLateral", true)));
	this->leftArm.shoulderFrontal.reset(this->add(new SimuServo("LeftShoulderFrontal", true)));

	this->rightHip.lateral.reset(this->add(new SimuServo("RightHipLateral", true)));
	this->rightHip.frontal.reset(this->add(new SimuServo("RightHipFrontal")));
	this->rightHip.transversal.reset(this->add(new SimuServo("RightHipTransversal")));

	this->leftHip.lateral.reset(this->add(new SimuServo("LeftHipLateral", false)));
	this->leftHip.frontal.reset(this->add(new SimuServo("LeftHipFrontal", true)));
	this->leftHip.transversal.reset(this->add(new SimuServo("LeftHipTransversal", true)));

	this->rightLeg.kneeLateral.reset(this->add(new SimuServo("RightKneeLateral", true)));
	this->leftLeg.kneeLateral.reset(this->add(new SimuServo("LeftKneeLateral", false)));

	this->rightFoot.lateral.reset(this->add(new SimuServo("RightFootLateral", true)));
	this->rightFoot.frontal.reset(this->add(new SimuServo("RightFootFrontal", true)));

	this->leftFoot.lateral.reset(this->add(new SimuServo("LeftFootLateral")));
	this->leftFoot.frontal.reset(this->add(new SimuServo("LeftFootFrontal")));
}
