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
	this->rightArm.shoulderLateral.reset(new SimuServo("RightShoulderLateral"));
	this->rightArm.shoulderFrontal.reset(new SimuServo("RightShoulderFrontal"));

	this->leftArm.elbow.reset(new SimuServo("LeftElbow"));
	this->leftArm.shoulderLateral.reset(new SimuServo("LeftShoulderLateral"));
	this->leftArm.shoulderFrontal.reset(new SimuServo("LeftShoulderFrontal"));

	this->rightHip.lateral.reset(this->add(new SimuServo("RightHipLateral")));
	this->rightHip.frontal.reset(this->add(new SimuServo("RightHipFrontal")));
	this->rightHip.transversal.reset(this->add(new SimuServo("RightHipTransversal")));

	this->leftHip.lateral.reset(this->add(new SimuServo("LeftHipLateral")));
	this->leftHip.frontal.reset(this->add(new SimuServo("LeftHipFrontal")));
	this->leftHip.transversal.reset(this->add(new SimuServo("LeftHipTransversal")));

	this->rightLeg.kneeLateral.reset(this->add(new SimuServo("RightKneeLateral")));
	this->leftLeg.kneeLateral.reset(this->add(new SimuServo("LeftKneeLateral")));

	this->rightFoot.lateral.reset(this->add(new SimuServo("RightFootLateral")));
	this->rightFoot.frontal.reset(this->add(new SimuServo("RightFootFrontal")));

	this->leftFoot.lateral.reset(this->add(new SimuServo("LeftFootLateral")));
	this->leftFoot.frontal.reset(this->add(new SimuServo("LeftFootFrontal")));
}
