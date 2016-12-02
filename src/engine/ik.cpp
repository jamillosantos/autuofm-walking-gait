/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include <algorithm>
#include "ik.h"

mote::walking::HumanoidIK::HumanoidIK(mote::walking::HumanoidPart &humanoid,
	mote::walking::HumanoidPart &humanoidPublished, mote::walking::Configuration &configuration, sensors::IMU &imu)
	: humanoid(humanoid), humanoidPublished(humanoidPublished), configuration(configuration), imu(imu)
{ }

void mote::walking::HumanoidIK::update()
{
	/*
	lock (Dxl_Pos)
	{
		lock (Dxl_Spd)
		{
			//set motors speed
			Dxl_Spd[R_L_Hip_Yaw] = IK_Spd[R_L_Hip_Yaw];
			Dxl_Spd[R_L_Hip_Roll] = IK_Spd[R_L_Hip_Roll];
			Dxl_Spd[R_L_Hip_Pitch] = IK_Spd[R_L_Hip_Pitch];
			Dxl_Spd[R_L_Knee] = IK_Spd[R_L_Knee];
			Dxl_Spd[R_L_Foot_Pitch] = IK_Spd[R_L_Foot_Pitch];
			Dxl_Spd[R_L_Foot_Roll] = IK_Spd[R_L_Foot_Roll];

			Dxl_Spd[L_L_Hip_Yaw] = IK_Spd[L_L_Hip_Yaw];
			Dxl_Spd[L_L_Hip_Roll] = IK_Spd[L_L_Hip_Roll];
			Dxl_Spd[L_L_Hip_Pitch] = IK_Spd[L_L_Hip_Pitch];
			Dxl_Spd[L_L_Knee] = IK_Spd[L_L_Knee];
			Dxl_Spd[L_L_Foot_Pitch] = IK_Spd[L_L_Foot_Pitch];
			Dxl_Spd[L_L_Foot_Roll] = IK_Spd[L_L_Foot_Roll];

			Dxl_Spd[R_A_Elbow] = IK_Spd[R_A_Elbow];
			Dxl_Spd[R_A_Roll] = IK_Spd[R_A_Roll];
			Dxl_Spd[R_A_Pitch] = IK_Spd[R_A_Pitch];

			Dxl_Spd[L_A_Elbow] = IK_Spd[L_A_Elbow];
			Dxl_Spd[L_A_Roll] = IK_Spd[L_A_Roll];
			Dxl_Spd[L_A_Pitch] = IK_Spd[L_A_Pitch];
	*/

	const configuration::Walking& conf = this->configuration.walking;

	//update IK value for offset
	// IK[R_L_X_Position] += WEP[P_Right_Leg_X_Offset] + WEP[P_COM_X_offset];
	this->humanoid.rightLeg.position.x += conf.rightLeg.positionOffset.x + conf.com.positionOffset.x;
	// IK[R_L_Y_Position] += WEP[P_Right_Leg_Y_Offset] + WEP[P_COM_Y_offset];
	this->humanoid.rightLeg.position.y += conf.rightLeg.positionOffset.y + conf.com.positionOffset.y;
	// IK[R_L_Z_Position] += WEP[P_Right_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	this->humanoid.rightLeg.position.z += conf.rightLeg.positionOffset.z + conf.com.positionOffset.z;
	// IK[R_L_R_Angle] += WEP[P_Right_Leg_Roll_Offset] + WEP[P_COM_Roll_offset];
	this->humanoid.rightLeg.angle.roll += conf.rightLeg.angleOffset.roll + conf.com.angleOffset.roll;
	// IK[R_L_P_Angle] += WEP[P_Right_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	this->humanoid.rightLeg.angle.pitch += conf.rightLeg.angleOffset.pitch + conf.com.angleOffset.pitch;
	// IK[R_L_Y_Angle] += WEP[P_Right_Leg_Yaw_Offset] + WEP[P_COM_Yaw_offset];
	this->humanoid.rightLeg.angle.yaw += conf.rightLeg.angleOffset.yaw + conf.com.angleOffset.yaw;

	// IK[L_L_X_Position] += WEP[P_Left_Leg_X_Offset] + WEP[P_COM_X_offset];
	this->humanoid.leftLeg.position.x += conf.leftLeg.positionOffset.x + conf.com.positionOffset.x;
	// IK[L_L_Y_Position] += WEP[P_Left_Leg_Y_Offset] + WEP[P_COM_Y_offset];
	this->humanoid.leftLeg.position.y += conf.leftLeg.positionOffset.y + conf.com.positionOffset.y;
	// IK[L_L_Z_Position] += WEP[P_Left_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	this->humanoid.leftLeg.position.z += conf.leftLeg.positionOffset.z + conf.com.positionOffset.z;
	// IK[L_L_R_Angle] += WEP[P_Left_Leg_Roll_Offset] + WEP[P_COM_Roll_offset];
	this->humanoid.leftLeg.angle.roll += conf.leftLeg.angleOffset.roll + conf.com.angleOffset.roll;
	// IK[L_L_P_Angle] += WEP[P_Left_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	this->humanoid.leftLeg.angle.pitch += conf.leftLeg.angleOffset.pitch + conf.com.angleOffset.pitch;
	// IK[L_L_Y_Angle] += WEP[P_Left_Leg_Yaw_Offset] + WEP[P_COM_Yaw_offset];
	this->humanoid.leftLeg.angle.yaw += conf.leftLeg.angleOffset.yaw + conf.com.angleOffset.yaw;

	// IK[R_H_X_Position] += WEP[P_Right_Arm_X_Offset];
	this->humanoid.rightArm.position.x += conf.rightArm.positionOffset.x;
	// IK[R_H_Y_Position] += WEP[P_Right_Arm_Y_Offset];
	this->humanoid.rightArm.position.y += conf.rightArm.positionOffset.y;
	// IK[R_H_Z_Position] += WEP[P_Right_Arm_Z_Offset];
	this->humanoid.rightArm.position.z += conf.rightArm.positionOffset.z;

	// IK[L_H_X_Position] += WEP[P_Left_Arm_X_Offset];
	this->humanoid.leftArm.position.x += conf.leftArm.positionOffset.x;
	// IK[L_H_Y_Position] += WEP[P_Left_Arm_Y_Offset];
	this->humanoid.leftArm.position.y += conf.leftArm.positionOffset.y;
	// IK[L_H_Z_Position] += WEP[P_Left_Arm_Z_Offset];
	this->humanoid.leftArm.position.z += conf.leftArm.positionOffset.z;

	//calculate absolute Z for legs
	// IK[R_L_Z_Position] = WEP[P_Leg_Length] - IK[R_L_Z_Position];
	this->humanoid.rightLeg.position.z = conf.legLength - this->humanoid.rightLeg.position.z;
	// IK[L_L_Z_Position] = WEP[P_Leg_Length] - IK[L_L_Z_Position]; //???????
	this->humanoid.leftLeg.position.z = conf.legLength - this->humanoid.leftLeg.position.z;

	this->humanoidPublished.rightLeg.position = this->humanoid.rightLeg.position;
	this->humanoidPublished.leftLeg.position = this->humanoid.leftLeg.position;

	this->humanoidPublished.rightArm.position = this->humanoid.rightArm.position;
	this->humanoidPublished.leftArm.position = this->humanoid.leftArm.position;

	//internal position value (degree)
	// double[] T_Pos = new double[18];
	//initialize for center first
	// for (int i = 0; i <= 17; i++)
	// {
	// 	T_Pos[i] = 0;
	// }

	//calculate the inverse kinematic
	//set legs yaw

	// T_Pos[R_L_Hip_Yaw] = IK[R_L_Y_Angle] + WEP[P_Right_Leg_Hip_Yaw_Offset];
	this->humanoidPublished.rightHip.angle.yaw = this->humanoid.rightHip.angle.yaw + conf.rightLeg.hipAngleOffset.yaw;
	// T_Pos[L_L_Hip_Yaw] = IK[L_L_Y_Angle] + WEP[P_Left_Leg_Hip_Yaw_Offset];
	this->humanoidPublished.leftHip.angle.yaw = this->humanoid.leftHip.angle.yaw + conf.leftLeg.hipAngleOffset.yaw;

	// double Tmp_angle = 0.0;
	double tmpAngle = 0.0;
	// double dfoot, alpha, beta;
	double dfoot, alpha, beta;

	// double Tmp_R_X = (IK[R_L_X_Position] * Math.Cos(IK[R_L_Y_Angle]) + IK[R_L_Y_Position] * Math.Sin(IK[R_L_Y_Angle]));
	double tmpRX = (this->humanoid.rightLeg.position.x * std::cos(this->humanoid.rightLeg.angle.yaw)) + (this->humanoid.rightLeg.position.y * std::sin(this->humanoid.rightLeg.angle.yaw));
	// double Tmp_R_Y = (IK[R_L_X_Position] * Math.Sin(IK[R_L_Y_Angle]) + IK[R_L_Y_Position] * Math.Cos(IK[R_L_Y_Angle]));
	double tmpRY = (this->humanoid.rightLeg.position.x * std::sin(this->humanoid.rightLeg.angle.yaw) + this->humanoid.rightLeg.position.y * std::cos(this->humanoid.rightLeg.angle.yaw));
	// double Tmp_R_Z = IK[R_L_Z_Position];
	double tmpRZ = this->humanoid.rightLeg.position.z;
	// Tmp_angle = Math.Atan2(Tmp_R_Y, Tmp_R_Z);
	tmpAngle = std::atan2(tmpRY, tmpRZ);
	// T_Pos[R_L_Hip_Roll] = Tmp_angle + IK[R_L_R_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset] -(MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	this->humanoidPublished.rightHip.angle.roll = tmpAngle + (this->humanoid.rightLeg.angle.roll / 2.0) + conf.rightLeg.hipAngleOffset.roll
							   - (this->imu.mpu.position.y * conf.stabilizer.hipGain.roll) - (this->imu.gyro.data.y * conf.gyroStabilizer.hipGain.roll);
	// T_Pos[R_L_Foot_Roll] = -Tmp_angle + IK[R_L_R_Angle] / 4.0 + WEP[P_Right_Leg_Foot_Roll_Offset] -(MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);
	this->humanoidPublished.rightFoot.angle.roll = -tmpAngle + (this->humanoid.rightLeg.angle.roll / 4.0) + conf.rightLeg.footOffset.roll
								- (this->imu.mpu.position.y * conf.rightLeg.footOffset.roll) - (this->imu.gyro.data.y * conf.gyroStabilizer.footGain.roll);

	// double Tmp_L_X = (IK[L_L_X_Position] * Math.Cos(IK[L_L_Y_Angle]) + IK[L_L_Y_Position] * Math.Sin(IK[L_L_Y_Angle]));
	double tmpLX = (this->humanoid.leftLeg.position.x * std::cos(this->humanoid.leftLeg.angle.yaw) + this->humanoid.leftLeg.position.y * std::sin(this->humanoid.leftLeg.angle.yaw));
	// double Tmp_L_Y = (IK[L_L_X_Position] * Math.Sin(IK[L_L_Y_Angle]) + IK[L_L_Y_Position] * Math.Cos(IK[L_L_Y_Angle]));
	double tmpLY = (this->humanoid.leftLeg.position.x * std::sin(this->humanoid.leftLeg.angle.yaw) + this->humanoid.leftLeg.position.y * std::cos(this->humanoid.leftLeg.angle.yaw));
	// double Tmp_L_Z = IK[L_L_Z_Position];
	double tmpLZ = this->humanoid.leftLeg.position.z;
	// Tmp_angle = Math.Atan2(Tmp_L_Y, Tmp_L_Z);
	tmpAngle = std::atan2(tmpLY, tmpLZ);
	// T_Pos[L_L_Hip_Roll] = Tmp_angle + IK[L_L_R_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset] +(MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	this->humanoidPublished.leftHip.angle.roll = tmpAngle + (this->humanoid.leftLeg.angle.roll / 2.0) + conf.leftLeg.hipAngleOffset.roll
							  +(this->imu.mpu.position.y * conf.stabilizer.hipGain.roll) + (this->imu.gyro.data.y * conf.gyroStabilizer.hipGain.roll);
	// T_Pos[L_L_Foot_Roll] = -Tmp_angle + IK[L_L_R_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] +(MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);
	this->humanoidPublished.leftFoot.angle.roll = -tmpAngle + (this->humanoid.leftLeg.angle.roll / 4.0) + conf.leftLeg.footOffset.roll
							   +(this->imu.mpu.position.y * conf.stabilizer.footGain.roll) + (this->imu.gyro.data.y * conf.gyroStabilizer.footGain.roll);


	// Tmp_R_Z = Math.Sqrt((Tmp_R_Y * Tmp_R_Y) + (Tmp_R_Z * Tmp_R_Z));
	tmpRZ = std::sqrt((tmpRY * tmpRY) + (tmpRZ * tmpRZ));
	// dfoot = Math.Sqrt((Tmp_R_Z * Tmp_R_Z) + (Tmp_R_X * Tmp_R_X));
	dfoot = std::sqrt((tmpRZ * tmpRZ) + (tmpRX * tmpRX));
	// alpha = Math.Atan2(Tmp_R_X, Tmp_R_Z);
	alpha = std::atan2(tmpRX, tmpRZ);
	// beta =Math.Acos(dfoot / WEP[P_Leg_Length]);
	beta = std::acos(dfoot / conf.legLength);
	// T_Pos[R_L_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] +(MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	this->humanoidPublished.rightHip.angle.pitch = (alpha + beta) + conf.rightLeg.hipAngleOffset.pitch +
								(this->imu.mpu.position.x * conf.stabilizer.hipGain.pitch) +
								(this->imu.gyro.data.x * conf.gyroStabilizer.hipGain.pitch);
	// T_Pos[R_L_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] +(MPU_X * WEP[P_Stablizer_Knee_Gain]) + (this->imu.gyro.data.x * WEP[P_Gyro_Stablizer_Knee_Gain]);
	this->humanoidPublished.rightKnee.angle =
		(-2.0 * beta) + conf.rightLeg.kneeOffset + (this->imu.mpu.position.x * conf.stabilizer.kneeGain) +
		(this->imu.gyro.data.x * conf.gyroStabilizer.kneeGain);
	// T_Pos[R_L_Foot_Pitch] = (-alpha + beta) + IK[R_L_P_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] +(MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
	this->humanoidPublished.rightFoot.angle.pitch =
		(-alpha + beta) + this->humanoid.rightLeg.angle.pitch + conf.rightLeg.footOffset.pitch +
		(this->imu.mpu.position.x * conf.stabilizer.footGain.pitch) +
		((this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x) * conf.gyroStabilizer.footGain.pitch);

	// Tmp_R_Z = Math.Sqrt((Tmp_L_Y * Tmp_L_Y) + (Tmp_L_Z * Tmp_L_Z));
	tmpRZ = std::sqrt((tmpLY * tmpLY) + (tmpLZ * tmpLZ));
	// dfoot = Math.Sqrt((Tmp_L_Z * Tmp_L_Z) + (Tmp_L_X * Tmp_L_X));
	dfoot = std::sqrt((tmpLZ * tmpLZ) + (tmpLX * tmpLX));
	// alpha = Math.Atan2(Tmp_L_X, Tmp_L_Z);
	alpha = std::atan2(tmpLX, tmpLZ);
	// beta = Math.Acos(dfoot / WEP[P_Leg_Length]);
	beta = std::acos(dfoot / conf.legLength);
	// T_Pos[L_L_Hip_Pitch] = (alpha + beta) + WEP[P_Left_Leg_Hip_Pitch_Offset] +(MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	this->humanoidPublished.leftHip.angle.pitch = (alpha + beta) + conf.leftLeg.hipAngleOffset.pitch + (this->imu.mpu.position.x * conf.stabilizer.hipGain.pitch) + (this->imu.gyro.data.x * conf.gyroStabilizer.hipGain.pitch);
	// T_Pos[L_L_Knee] = (-2.0 * beta) + WEP[P_Left_Leg_Knee_Offset] +(MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	this->humanoidPublished.leftKnee.angle = (-2.0 * beta) + conf.leftLeg.kneeOffset
						  + (this->imu.mpu.position.x * conf.stabilizer.kneeGain)
						  + (this->imu.gyro.data.x * conf.gyroStabilizer.kneeGain);
	// T_Pos[L_L_Foot_Pitch] = (-alpha + beta) + IK[L_L_P_Angle] + WEP[P_Left_Leg_Foot_Pitch_Offset] +(MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
	this->humanoidPublished.leftFoot.angle.pitch = (-alpha + beta) + this->humanoid.leftLeg.angle.pitch + conf.leftLeg.footOffset.pitch
								+ (this->imu.mpu.position.x * conf.stabilizer.footGain.pitch)
								+ ((this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x)
								   * conf.gyroStabilizer.footGain.pitch);

	// T_Pos[R_A_Pitch] = IK[R_H_Z_Position] + WEP[P_R_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoidPublished.rightArm.angle.pitch = this->humanoid.rightArm.position.z + conf.rightArm.angleOffset.pitch +
								(this->imu.mpu.position.x * conf.stabilizer.armGain.pitch) +
								(this->imu.gyro.data.x * conf.gyroStabilizer.armGain.pitch);
	// T_Pos[R_A_Roll] = IK[R_H_Y_Position] + WEP[P_R_Arm_Roll_offset] + (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Arm_Roll_Gain]);
	this->humanoidPublished.rightArm.angle.roll = this->humanoid.rightArm.position.y + conf.rightArm.angleOffset.roll + (this->imu.mpu.position.x * conf.stabilizer.armGain.roll) + (this->imu.gyro.data.x * conf.gyroStabilizer.armGain.roll);
	// T_Pos[R_A_Elbow] = IK[R_H_X_Position] + WEP[P_R_Arm_Elbow_offset]; // +(MPU_X * WEP[P_Stablizer_Arm_Elbow_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]);
	this->humanoidPublished.rightArm.elbow = this->humanoid.rightArm.position.x + conf.rightArm.elbowOffset; // +(MPU_X * WEP[P_Stablizer_Arm_Elbow_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]);


	// T_Pos[L_A_Pitch] = IK[L_H_Z_Position] + WEP[P_L_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoidPublished.leftArm.angle.pitch = this->humanoid.leftArm.position.z + conf.leftArm.angleOffset.pitch +
							   (this->imu.mpu.position.x * conf.stabilizer.armGain.pitch) +
							   (this->imu.gyro.data.x * conf.gyroStabilizer.armGain.pitch);
	// T_Pos[L_A_Roll] = IK[L_H_Y_Position] + WEP[P_L_Arm_Roll_offset] - (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Arm_Roll_Gain]);
	this->humanoidPublished.leftArm.angle.roll = this->humanoid.leftArm.position.y + conf.leftArm.angleOffset.roll -
							  (this->imu.mpu.position.x * conf.stabilizer.armGain.roll) -
							  (this->imu.gyro.data.x * conf.gyroStabilizer.armGain.roll);
	// T_Pos[L_A_Elbow] = IK[L_H_X_Position] + WEP[P_L_Arm_Elbow_offset]; //(MPU_X * WEP[P_Stablizer_Arm_Elbow_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]);
	this->humanoidPublished.leftArm.elbow = this->humanoid.leftArm.position.x + conf.leftArm.elbowOffset; //(MPU_X * WEP[P_Stablizer_Arm_Elbow_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Arm_Elbow_Gain]);

	/*
	//set motors positions
	Dxl_Pos[R_L_Hip_Yaw] = 2048 + (Joint_Dir[R_L_Hip_Yaw] * (int)(T_Pos[R_L_Hip_Yaw] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_L_Hip_Roll] = 2048 + (Joint_Dir[R_L_Hip_Roll] * (int)(T_Pos[R_L_Hip_Roll] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_L_Hip_Pitch] = 2048 + (Joint_Dir[R_L_Hip_Pitch] * (int)(T_Pos[R_L_Hip_Pitch] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_L_Knee] = 2048 + (Joint_Dir[R_L_Knee] * (int)(T_Pos[R_L_Knee] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_L_Foot_Pitch] = 2048 + (Joint_Dir[R_L_Foot_Pitch] * (int)(T_Pos[R_L_Foot_Pitch] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_L_Foot_Roll] = 2048 + (Joint_Dir[R_L_Foot_Roll] * (int)(T_Pos[R_L_Foot_Roll] * RAD2DEG * DEG2DXL));

	Dxl_Pos[L_L_Hip_Yaw] = 2048 + (Joint_Dir[L_L_Hip_Yaw] * (int)(T_Pos[L_L_Hip_Yaw] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_L_Hip_Roll] = 2048 + (Joint_Dir[L_L_Hip_Roll] * (int)(T_Pos[L_L_Hip_Roll] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_L_Hip_Pitch] = 2048 + (Joint_Dir[L_L_Hip_Pitch] * (int)(T_Pos[L_L_Hip_Pitch] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_L_Knee] = 2048 + (Joint_Dir[L_L_Knee] * (int)(T_Pos[L_L_Knee] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_L_Foot_Pitch] = 2048 + (Joint_Dir[L_L_Foot_Pitch] * (int)(T_Pos[L_L_Foot_Pitch] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_L_Foot_Roll] = 2048 + (Joint_Dir[L_L_Foot_Roll] * (int)(T_Pos[L_L_Foot_Roll] * RAD2DEG * DEG2DXL));

	Dxl_Pos[R_A_Elbow] = 2048 + (Joint_Dir[R_A_Elbow] * (int)(T_Pos[R_A_Elbow] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_A_Roll] = 2048 + (Joint_Dir[R_A_Roll] * (int)(T_Pos[R_A_Roll] * RAD2DEG * DEG2DXL));
	Dxl_Pos[R_A_Pitch] = 2048 + (Joint_Dir[R_A_Pitch] * (int)(T_Pos[R_A_Pitch] * RAD2DEG * DEG2DXL));

	Dxl_Pos[L_A_Elbow] = 2048 + (Joint_Dir[L_A_Elbow] * (int)(T_Pos[L_A_Elbow] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_A_Roll] = 2048 + (Joint_Dir[L_A_Roll] * (int)(T_Pos[L_A_Roll] * RAD2DEG * DEG2DXL));
	Dxl_Pos[L_A_Pitch] = 2048 + (Joint_Dir[L_A_Pitch] * (int)(T_Pos[L_A_Pitch] * RAD2DEG * DEG2DXL));

		}
	}
	*/
}
