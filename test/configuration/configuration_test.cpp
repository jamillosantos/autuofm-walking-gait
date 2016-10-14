/**
 * @author J. Santos <jamillo@gmail.com>
 * @date October 13, 2016
 */

#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>

#include "../../src/configuration/configuration.h"
#include "../resources.h"

GTEST_TEST(configuration_configuration, loadFromFile)
{
	mote::walking::Configuration configuration;
	configuration.loadFromFile((Resources::resources() / "config.json").string());

	ASSERT_EQ(1000, configuration.walking.head.panSpeed);
	ASSERT_EQ(1000, configuration.walking.head.tiltSpeed);

	ASSERT_EQ(100, configuration.walking.minVoltage);

	ASSERT_EQ(471, configuration.walking.legLength);

	ASSERT_EQ(-0.07, configuration.walking.velocityOffset.x);
	ASSERT_EQ(0.0, configuration.walking.velocityOffset.y);
	ASSERT_EQ(0.001, configuration.walking.velocityOffset.theta);

	ASSERT_EQ(0.13, configuration.walking.engine.motionResolution);
	ASSERT_EQ(0.1, configuration.walking.engine.gaitFrequency);
	ASSERT_EQ(40, configuration.walking.engine.doubleSupportSleep);
	ASSERT_EQ(0, configuration.walking.engine.singleSupportSleep);
	ASSERT_EQ(0.005, configuration.walking.engine.flyGain.roll);
	ASSERT_EQ(0.8, configuration.walking.engine.flyGain.pitch);
	ASSERT_EQ(0, configuration.walking.engine.flyGain.yaw);
	ASSERT_EQ(0, configuration.walking.engine.flySwingGain.x);
	ASSERT_EQ(0.05, configuration.walking.engine.flySwingGain.y);
	ASSERT_EQ(0.8, configuration.walking.engine.flySwingGain.z);
	ASSERT_EQ(0.12, configuration.walking.engine.supportGain.roll);
	ASSERT_EQ(0, configuration.walking.engine.supportGain.pitch);
	ASSERT_EQ(0, configuration.walking.engine.supportGain.yaw);
	ASSERT_EQ(0, configuration.walking.engine.supportSwingGain.x);
	ASSERT_EQ(0, configuration.walking.engine.supportSwingGain.y);
	ASSERT_EQ(-0.03, configuration.walking.engine.supportSwingGain.z);
	ASSERT_EQ(-1.0, configuration.walking.engine.bodySwingGain.x);
	ASSERT_EQ(0.1, configuration.walking.engine.bodySwingGain.y);
	ASSERT_EQ(0, configuration.walking.engine.bodySwingGain.z);

	ASSERT_EQ(-3.5, configuration.walking.stabilizer.armGain.pitch);
	ASSERT_EQ(2.2, configuration.walking.stabilizer.armGain.roll);
	ASSERT_EQ(0, configuration.walking.stabilizer.armElbowGain);
	ASSERT_EQ(0, configuration.walking.stabilizer.hipGain.roll);
	ASSERT_EQ(-0.05, configuration.walking.stabilizer.hipGain.pitch);
	ASSERT_EQ(0, configuration.walking.stabilizer.kneeGain);
	ASSERT_EQ(-0.05, configuration.walking.stabilizer.footGain.pitch);
	ASSERT_EQ(0, configuration.walking.stabilizer.footGain.roll);
	ASSERT_EQ(2, configuration.walking.stabilizer.comShiftGain.x);
	ASSERT_EQ(40, configuration.walking.stabilizer.comShiftGain.y);

	ASSERT_EQ(-0.000001, configuration.walking.gyroStabilizer.armGain.pitch);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.armGain.roll);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.armElbowGain);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.hipGain.roll);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.hipGain.pitch);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.kneeGain);
	ASSERT_EQ(-0.000002, configuration.walking.gyroStabilizer.footGain.pitch);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.footGain.roll);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.comShiftGain.x);
	ASSERT_EQ(0, configuration.walking.gyroStabilizer.comShiftGain.y);

	ASSERT_EQ(0, configuration.walking.hoppingGaitGain.x);
	ASSERT_EQ(0, configuration.walking.hoppingGaitGain.y);

	ASSERT_EQ(-5, configuration.walking.com.positionOffset.x);
	ASSERT_EQ(35, configuration.walking.com.positionOffset.y);
	ASSERT_EQ(110, configuration.walking.com.positionOffset.z);

	ASSERT_EQ(0, configuration.walking.com.angleOffset.roll);
	ASSERT_EQ(0.032, configuration.walking.com.angleOffset.pitch);
	ASSERT_EQ(0, configuration.walking.com.angleOffset.yaw);

	ASSERT_EQ(0.1, configuration.walking.leftLeg.hipAngleOffset.pitch);
	ASSERT_EQ(0, configuration.walking.leftLeg.hipAngleOffset.yaw);
	ASSERT_EQ(0, configuration.walking.leftLeg.hipAngleOffset.roll);
	ASSERT_EQ(0, configuration.walking.leftLeg.kneeOffset);
	ASSERT_EQ(0, configuration.walking.leftLeg.footOffset.pitch);
	ASSERT_EQ(0, configuration.walking.leftLeg.footOffset.roll);

	ASSERT_EQ(0.1, configuration.walking.rightLeg.hipAngleOffset.pitch);
	ASSERT_EQ(0, configuration.walking.rightLeg.hipAngleOffset.yaw);
	ASSERT_EQ(0, configuration.walking.rightLeg.hipAngleOffset.roll);
	ASSERT_EQ(0, configuration.walking.rightLeg.kneeOffset);
	ASSERT_EQ(0, configuration.walking.rightLeg.footOffset.pitch);
	ASSERT_EQ(0, configuration.walking.rightLeg.footOffset.roll);

	ASSERT_EQ(0, configuration.walking.leftLeg.positionOffset.x);
	ASSERT_EQ(10, configuration.walking.leftLeg.positionOffset.y);
	ASSERT_EQ(-3, configuration.walking.leftLeg.positionOffset.z);
	ASSERT_EQ(0, configuration.walking.leftLeg.angleOffset.roll);
	ASSERT_EQ(0, configuration.walking.leftLeg.angleOffset.pitch);
	ASSERT_EQ(0, configuration.walking.leftLeg.angleOffset.yaw);

	ASSERT_EQ(-4, configuration.walking.rightLeg.positionOffset.x);
	ASSERT_EQ(10, configuration.walking.rightLeg.positionOffset.y);
	ASSERT_EQ(0, configuration.walking.rightLeg.positionOffset.z);
	ASSERT_EQ(0, configuration.walking.rightLeg.angleOffset.roll);
	ASSERT_EQ(0, configuration.walking.rightLeg.angleOffset.pitch);
	ASSERT_EQ(0, configuration.walking.rightLeg.angleOffset.yaw);

	ASSERT_EQ(-0.2, configuration.walking.rightArm.angleOffset.roll);
	ASSERT_EQ(-1.9, configuration.walking.rightArm.angleOffset.pitch);
	ASSERT_EQ(1.1, configuration.walking.rightArm.elbowOffset);

	ASSERT_EQ(-0.2, configuration.walking.leftArm.angleOffset.roll);
	ASSERT_EQ(-1.9, configuration.walking.leftArm.angleOffset.pitch);
	ASSERT_EQ(1.1, configuration.walking.leftArm.elbowOffset);

	ASSERT_EQ(0.055, configuration.walking.imuOffset.x);
	ASSERT_EQ(0.07, configuration.walking.imuOffset.y);

	ASSERT_EQ(0.8, configuration.walking.gyroLowpassGain.x);
	ASSERT_EQ(0.7, configuration.walking.gyroLowpassGain.y);

	ASSERT_EQ(40, configuration.walking.kalmanRmRate.roll);
	ASSERT_EQ(100, configuration.walking.kalmanRmRate.pitch);
	ASSERT_EQ(50, configuration.walking.kalmanRmRate.yaw);

	ASSERT_EQ(0, configuration.walking.smoothingRatio.x);
	ASSERT_EQ(0, configuration.walking.smoothingRatio.y);
	ASSERT_EQ(0, configuration.walking.smoothingRatio.z);
}
