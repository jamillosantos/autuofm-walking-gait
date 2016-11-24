/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include <algorithm>
#include "ik.h"

mote::walking::HumanoidIK::HumanoidIK(mote::walking::HumanoidPart &humanoid, mote::walking::Configuration &configuration,
	sensors::IMU &imu)
	: humanoid(humanoid), configuration(configuration), imu(imu)
{ }

void mote::walking::HumanoidIK::update(double _R_Leg_Speed, double _L_Leg_Speed, Leg rightLeg, Leg leftLeg,
	Arm rightArm, Arm leftArm)
{
	/**
	 * Right leg
	 */
	// _R_Leg_Ik[X_Position] += WEP[P_Right_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	rightLeg.position.x +=
		this->configuration.walking.rightLeg.positionOffset.x + this->configuration.walking.com.positionOffset.x
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.comShiftGain.x)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.comShiftGain.x);
	// _R_Leg_Ik[Y_Position] += WEP[P_Right_Leg_Y_Offset] + WEP[P_COM_Y_offset] - (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	rightLeg.position.y +=
		this->configuration.walking.rightLeg.positionOffset.y + this->configuration.walking.com.positionOffset.y
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.comShiftGain.y)
		- (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.comShiftGain.y);
	// _R_Leg_Ik[Z_Position] += WEP[P_Right_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	rightLeg.position.z +=
		this->configuration.walking.rightLeg.positionOffset.z + this->configuration.walking.com.positionOffset.z;

	// _R_Leg_Ik[Roll_Angle] += WEP[P_Right_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	rightLeg.angle.roll +=
		this->configuration.walking.rightLeg.angleOffset.roll + this->configuration.walking.com.angleOffset.roll;
	// _R_Leg_Ik[Pitch_Angle]+= WEP[P_Right_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	rightLeg.angle.pitch +=
		this->configuration.walking.rightLeg.angleOffset.pitch + this->configuration.walking.com.angleOffset.pitch;
	// _R_Leg_Ik[Yaw_Angle]  += WEP[P_Right_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];
	rightLeg.angle.yaw +=
		this->configuration.walking.rightLeg.angleOffset.yaw + this->configuration.walking.com.angleOffset.yaw;

	/**
	 * Left leg
	 */
	//_L_Leg_Ik[X_Position] += WEP[P_Left_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	leftLeg.position.x +=
		this->configuration.walking.leftLeg.positionOffset.x + this->configuration.walking.com.positionOffset.x
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.comShiftGain.x)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.comShiftGain.x);
	// _L_Leg_Ik[Y_Position] += WEP[P_Left_Leg_Y_Offset] + WEP[P_COM_Y_offset] + (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	leftLeg.position.y +=
		this->configuration.walking.leftLeg.positionOffset.y + this->configuration.walking.com.positionOffset.y
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.comShiftGain.y)
		+ (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.comShiftGain.y);
	// _L_Leg_Ik[Z_Position] += WEP[P_Left_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	leftLeg.position.z +=
		this->configuration.walking.leftLeg.positionOffset.z + this->configuration.walking.com.positionOffset.z;

	// _L_Leg_Ik[Roll_Angle] += WEP[P_Left_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	leftLeg.angle.roll +=
		this->configuration.walking.leftLeg.angleOffset.roll + this->configuration.walking.com.angleOffset.roll;
	// _L_Leg_Ik[Pitch_Angle] += WEP[P_Left_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	leftLeg.angle.pitch +=
		this->configuration.walking.leftLeg.angleOffset.pitch + this->configuration.walking.com.angleOffset.pitch;
	// _L_Leg_Ik[Yaw_Angle]  += WEP[P_Left_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];
	leftLeg.angle.yaw +=
		this->configuration.walking.leftLeg.angleOffset.yaw + this->configuration.walking.com.angleOffset.yaw;

	//calculate absolute Z
	// _R_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _R_Leg_Ik[Z_Position];
	rightLeg.position.z = this->configuration.walking.legLength - rightLeg.position.z;
	// _L_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _L_Leg_Ik[Z_Position];
	leftLeg.position.z = this->configuration.walking.legLength - leftLeg.position.z;

	/**
	 * Right arm joints update
	 */

	// Speed[Id_Right_Arm_Pitch] = _R_Arm[I_A_Vp];
	this->humanoid.rightArm.velocity.pitch = rightArm.velocity.pitch;
	// Speed[Id_Right_Arm_Roll] = _R_Arm[I_A_Vr];
	this->humanoid.rightArm.velocity.roll = rightArm.velocity.roll;
	// Speed[Id_Right_Arm_Elbow]= _R_Arm[I_A_Ve];
	this->humanoid.rightArm.velocityElbow = rightArm.velocityElbow;

	// Angle[Id_Right_Arm_Pitch]= _R_Arm[I_A_Pitch] + WEP[P_R_Arm_Pitch_offset]
	// 	+ (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain])
	// 	+ ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoid.rightArm.angle.pitch = rightArm.angle.pitch + this->configuration.walking.rightArm.angleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.armGain.pitch)
		+ ((this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x) * this->configuration.walking.gyroStabilizer.armGain.pitch);
	// Angle[Id_Right_Arm_Roll] = _R_Arm[I_A_Roll]  + WEP[P_R_Arm_Roll_offset]  + (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	this->humanoid.rightArm.angle.roll = rightArm.angle.roll + this->configuration.walking.rightArm.angleOffset.roll
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.armGain.roll);

	// if (Angle[Id_Right_Arm_Roll]<-1.85)
	//	Angle[Id_Right_Arm_Roll]=-1.85;
	this->humanoid.rightArm.angle.roll = std::max(-1.85, this->humanoid.rightArm.angle.roll);
	//Angle[Id_Right_Arm_Elbow]= _R_Arm[I_A_Elbow] + WEP[P_R_Arm_Elbow_offset];
	this->humanoid.rightArm.elbow = rightArm.elbow + this->configuration.walking.rightArm.elbowOffset;

	/**
	 * Left arm joints update
	 */

	// Speed[Id_Left_Arm_Pitch] = _L_Arm[I_A_Vp];
	this->humanoid.leftArm.velocity.pitch = leftArm.velocity.pitch;
	// Speed[Id_Left_Arm_Roll] = _L_Arm[I_A_Vr];
	this->humanoid.leftArm.velocity.roll = leftArm.velocity.roll;
	// Speed[Id_Left_Arm_Elbow] = _L_Arm[I_A_Ve];
	this->humanoid.leftArm.velocityElbow = leftArm.velocityElbow;

	// Angle[Id_Left_Arm_Pitch] =
	// 	_L_Arm[I_A_Pitch] + WEP[P_L_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) +
	// 	((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoid.leftArm.angle.pitch = leftArm.angle.pitch + this->configuration.walking.leftArm.angleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.armGain.pitch)
		+ (
			(this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x)
			+ this->configuration.walking.gyroStabilizer.armGain.pitch
		);

	// Angle[Id_Left_Arm_Roll] = _L_Arm[I_A_Roll] + WEP[P_L_Arm_Roll_offset] - (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	this->humanoid.leftArm.angle.roll = leftArm.angle.roll + this->configuration.walking.leftArm.angleOffset.roll
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.armGain.roll);
	// if (Angle[Id_Left_Arm_Roll] < -1.85)
	// 	Angle[Id_Left_Arm_Roll] = -1.85;
	this->humanoid.leftArm.angle.roll = std::max(-1.85, this->humanoid.leftArm.angle.roll);
	// Angle[Id_Left_Arm_Elbow] = _L_Arm[I_A_Elbow] + WEP[P_L_Arm_Elbow_offset];
	this->humanoid.leftArm.elbow = leftArm.elbow + this->configuration.walking.leftArm.elbowOffset;

	/**
	 * Set right leg speeds
	 */
	// Speed[Id_Right_Hip_Yaw] = _R_Leg_Speed;
	this->humanoid.rightHip.velocity.yaw = _R_Leg_Speed;
	// Speed[Id_Right_Hip_Roll] = _R_Leg_Speed;
	this->humanoid.rightHip.velocity.roll = _R_Leg_Speed;
	// Speed[Id_Right_Hip_Pitch] = _R_Leg_Speed / 2.0;
	this->humanoid.rightHip.velocity.pitch = _R_Leg_Speed / 2.0;
	// Speed[Id_Right_Knee] = _R_Leg_Speed;
	this->humanoid.rightKnee.velocity = _R_Leg_Speed;
	// Speed[Id_Right_Foot_Pitch] = _R_Leg_Speed / 2.0;
	this->humanoid.rightFoot.velocity.pitch = _R_Leg_Speed / 2.0;
	// Speed[Id_Right_Foot_Roll] = _R_Leg_Speed;
	this->humanoid.rightFoot.velocity.roll = _R_Leg_Speed;

	/**
	 * Set left leg speeds
	 */
	// Speed[Id_Left_Hip_Yaw] = _L_Leg_Speed;
	this->humanoid.leftHip.velocity.yaw = _L_Leg_Speed;
	// Speed[Id_Left_Hip_Roll] = _L_Leg_Speed;
	this->humanoid.leftHip.velocity.roll = _L_Leg_Speed;
	// Speed[Id_Left_Hip_Pitch] = _L_Leg_Speed / 2.0;
	this->humanoid.leftHip.velocity.pitch = _L_Leg_Speed / 2.0;
	// Speed[Id_Left_Knee] = _L_Leg_Speed;
	this->humanoid.leftKnee.velocity = _L_Leg_Speed;
	// Speed[Id_Left_Foot_Pitch] = _L_Leg_Speed / 2.0;
	this->humanoid.leftFoot.velocity.pitch = _L_Leg_Speed / 2.0;
	// Speed[Id_Left_Foot_Roll] = _L_Leg_Speed;
	this->humanoid.leftFoot.velocity.roll = _L_Leg_Speed;

	/**
	 * Set legs yaw
	 */
	// Angle[Id_Right_Hip_Yaw] = _R_Leg_Ik[Yaw_Angle] + WEP[P_Right_Leg_Hip_Yaw_Offset];
	this->humanoid.rightHip.angle.yaw = rightLeg.angle.yaw + this->configuration.walking.rightLeg.hipAngleOffset.yaw;
	// Angle[Id_Left_Hip_Yaw] = _L_Leg_Ik[Yaw_Angle] + WEP[P_Left_Leg_Hip_Yaw_Offset];
	this->humanoid.leftHip.angle.yaw = leftLeg.angle.yaw + this->configuration.walking.leftLeg.hipAngleOffset.yaw;

	double tmpAngle = 0.0;
	double dfoot, alpha, beta;

	// double tmpRightX = (_R_Leg_Ik[X_Position] * cos(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * sin(_R_Leg_Ik[Yaw_Angle]));
	double
		tmpRightX =
			(rightLeg.position.x * std::cos(rightLeg.angle.yaw)) + (rightLeg.position.y * std::sin(rightLeg.angle.yaw)),
	// double tmpRightY = (_R_Leg_Ik[X_Position] * sin(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * cos(_R_Leg_Ik[Yaw_Angle]));
		tmpRightY =
			(rightLeg.position.x * std::sin(rightLeg.angle.yaw))
			+ (rightLeg.position.y * std::cos(rightLeg.angle.yaw)),
	// double Tmp_R_Z= _R_Leg_Ik[Z_Position];
		tmpRightZ = rightLeg.position.z;
	// Tmp_angle= atan2(Tmp_R_Y, Tmp_R_Z);
	tmpAngle = std::atan2(tmpRightY, tmpRightZ);
	// Angle[Id_Right_Hip_Roll]  = tmpAngle + _R_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset]  - (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	this->humanoid.rightHip.angle.roll =
		tmpAngle + (rightLeg.angle.roll / 2.0) + this->configuration.walking.rightLeg.hipAngleOffset.roll
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.hipGain.roll)
		- (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.hipGain.roll);
	// Angle[Id_Right_Foot_Roll] =-tmpAngle + _R_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Right_Leg_Foot_Roll_Offset] - (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);
	this->humanoid.rightFoot.angle.roll =
		-tmpAngle + (rightLeg.angle.roll / 4.0) + this->configuration.walking.rightLeg.footOffset.roll
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.footGain.roll)
		- (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.footGain.roll);

	// double tmpLeftX = (_L_Leg_Ik[X_Position] * cos(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * sin(_L_Leg_Ik[Yaw_Angle]));
	double
		tmpLeftX =
			(leftLeg.position.x * std::cos(leftLeg.angle.yaw))
			+ (leftLeg.position.y * std::sin(leftLeg.angle.yaw)),
	// double tmpLeftY = (_L_Leg_Ik[X_Position] * sin(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * cos(_L_Leg_Ik[Yaw_Angle]));
		tmpLeftY =
			(leftLeg.position.x * std::sin(leftLeg.angle.yaw))
			+ (leftLeg.position.y * std::cos(leftLeg.angle.yaw)),
	// double tmpLeftZ =  _L_Leg_Ik[Z_Position];
		tmpLeftZ =  leftLeg.position.z;
	tmpAngle = atan2(tmpLeftY, tmpLeftZ);
	// Angle[Id_Left_Hip_Roll]  = tmpAngle  + _L_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset]  + (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	this->humanoid.leftHip.angle.roll =
		tmpAngle + (leftLeg.angle.roll / 2.0) + this->configuration.walking.leftLeg.hipAngleOffset.roll
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.hipGain.roll)
		+ (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.hipGain.roll);
	// Angle[Id_Left_Foot_Roll] =-tmpAngle  + _L_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] + (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);
	this->humanoid.leftFoot.angle.roll =
		-tmpAngle + (leftLeg.angle.roll / 4.0) + this->configuration.walking.leftLeg.footOffset.roll
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.footGain.roll)
		+ (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.footGain.roll);

	// Tmp_R_Z= sqrt((Tmp_R_Y * Tmp_R_Y) + (Tmp_R_Z * Tmp_R_Z));
	tmpRightZ = std::sqrt((tmpRightY * tmpRightY) + (tmpRightZ * tmpRightZ));
	// dfoot= sqrt((Tmp_R_Z * Tmp_R_Z) + (Tmp_R_X * Tmp_R_X));
	dfoot = std::sqrt((tmpRightZ * tmpRightZ) + (tmpRightX * tmpRightX));
	// alpha= atan2(Tmp_R_X, Tmp_R_Z);
	alpha = std::atan2(tmpRightX, tmpRightZ);
	// beta = acos(dfoot / WEP[P_Leg_Length]);
	beta = std::acos(dfoot / this->configuration.walking.legLength);
	// Angle[Id_Right_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	this->humanoid.rightHip.angle.pitch =
		(alpha + beta) + this->configuration.walking.rightLeg.hipAngleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.hipGain.pitch)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.hipGain.pitch);
	// Angle[Id_Right_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	this->humanoid.rightKnee.angle =
		(-2.0 * beta) + this->configuration.walking.rightLeg.kneeOffset
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.kneeGain)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.kneeGain);
	// Angle[Id_Right_Foot_Pitch] = (-alpha + beta) + _R_Leg_Ik[Pitch_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
	this->humanoid.rightFoot.angle.pitch =
		(-alpha + beta) + rightLeg.angle.pitch + this->configuration.walking.rightLeg.footOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.footGain.pitch)
		+ (
			(this->imu.gyro.data.y*this->imu.gyro.data.y*this->imu.gyro.data.y)
			* this->configuration.walking.gyroStabilizer.footGain.pitch
		);

	tmpRightZ = std::sqrt((tmpLeftY * tmpLeftY) + (tmpLeftZ * tmpLeftZ));
	dfoot = std::sqrt((tmpLeftZ * tmpLeftZ) + (tmpLeftX * tmpLeftX));
	alpha = std::atan2(tmpLeftX, tmpLeftZ);
	// beta = std::acos(dfoot / WEP[P_Leg_Length]);
	beta = std::acos(dfoot / this->configuration.walking.legLength);
	// Angle[Id_Left_Hip_Pitch]  = (alpha + beta) + WEP[P_Left_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	this->humanoid.leftHip.angle.pitch =
		(alpha + beta) + this->configuration.walking.leftLeg.hipAngleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.hipGain.pitch)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.hipGain.pitch);
	// Angle[Id_Left_Knee]  = (-2.0 * beta) + WEP[P_Left_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	this->humanoid.leftKnee.angle =
		(-2.0 * beta) + this->configuration.walking.leftLeg.kneeOffset
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.kneeGain)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.kneeGain);
	// Angle[Id_Left_Foot_Pitch]  = (-alpha + beta) + _L_Leg_Ik[Pitch_Angle] + WEP[P_Left_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
	this->humanoid.leftFoot.angle.pitch =
		(-alpha + beta) + leftLeg.angle.pitch + this->configuration.walking.leftLeg.footOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.footGain.pitch)
		+ (
			(this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x)
			* this->configuration.walking.gyroStabilizer.footGain.pitch
		);
}
