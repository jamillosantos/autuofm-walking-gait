/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include <algorithm>
#include "ik.h"
#include "robot.h"

mote::walking::HumanoidIK::HumanoidIK(mote::walking::Humanoid &humanoid, mote::walking::Configuration &configuration,
	sensors::IMU &imu)
	: humanoid(humanoid), configuration(configuration), imu(imu)
{ }

void mote::walking::HumanoidIK::update(double _R_Leg_Speed, double _L_Leg_Speed, Leg rightLeg, Leg leftLeg,
	Arm rightArm, Arm leftArm)
{
	//add offset to inverse kinematic

	/**
	 * Right leg
	 */
	// _R_Leg_Ik[X_Position] += WEP[P_Right_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	this->humanoid.rightLeg.position.x +=
		this->configuration.walking.rightLeg.positionOffset.x + this->configuration.walking.com.positionOffset.x
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.comShiftGain.x)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.comShiftGain.x);
	// _R_Leg_Ik[Y_Position] += WEP[P_Right_Leg_Y_Offset] + WEP[P_COM_Y_offset] - (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	this->humanoid.rightLeg.position.y +=
		this->configuration.walking.rightLeg.positionOffset.y + this->configuration.walking.com.positionOffset.y
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.comShiftGain.y)
		- (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.comShiftGain.y);
	// _R_Leg_Ik[Z_Position] += WEP[P_Right_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	this->humanoid.rightLeg.position.z +=
		this->configuration.walking.rightLeg.positionOffset.z + this->configuration.walking.com.positionOffset.z;

	// _R_Leg_Ik[Roll_Angle] += WEP[P_Right_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	this->humanoid.rightLeg.angle.roll +=
		this->configuration.walking.rightLeg.angleOffset.roll + this->configuration.walking.com.angleOffset.roll;
	// _R_Leg_Ik[Pitch_Angle]+= WEP[P_Right_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	this->humanoid.rightLeg.angle.pitch +=
		this->configuration.walking.rightLeg.angleOffset.pitch + this->configuration.walking.com.angleOffset.pitch;
	// _R_Leg_Ik[Yaw_Angle]  += WEP[P_Right_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];
	this->humanoid.rightLeg.angle.yaw +=
		this->configuration.walking.rightLeg.angleOffset.yaw + this->configuration.walking.com.angleOffset.yaw;

	/**
	 * Left leg
	 */
	//_L_Leg_Ik[X_Position] += WEP[P_Left_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	this->humanoid.leftLeg.position.x +=
		this->configuration.walking.leftLeg.positionOffset.x + this->configuration.walking.com.positionOffset.x
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.comShiftGain.x)
		+ (this->imu.gyro.data.x * this->configuration.walking.gyroStabilizer.comShiftGain.x);
	// _L_Leg_Ik[Y_Position] += WEP[P_Left_Leg_Y_Offset] + WEP[P_COM_Y_offset] + (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	this->humanoid.leftLeg.position.y +=
		this->configuration.walking.leftLeg.positionOffset.y + this->configuration.walking.com.positionOffset.y
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.comShiftGain.y)
		+ (this->imu.gyro.data.y * this->configuration.walking.gyroStabilizer.comShiftGain.y);
	// _L_Leg_Ik[Z_Position] += WEP[P_Left_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	this->humanoid.leftLeg.position.z +=
		this->configuration.walking.leftLeg.positionOffset.z + this->configuration.walking.com.positionOffset.z;

	// _L_Leg_Ik[Roll_Angle] += WEP[P_Left_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	this->humanoid.leftLeg.angle.roll +=
		this->configuration.walking.leftLeg.angleOffset.roll + this->configuration.walking.com.angleOffset.roll;
	// _L_Leg_Ik[Pitch_Angle] += WEP[P_Left_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	this->humanoid.leftLeg.angle.pitch +=
		this->configuration.walking.leftLeg.angleOffset.pitch + this->configuration.walking.com.angleOffset.pitch;
	// _L_Leg_Ik[Yaw_Angle]  += WEP[P_Left_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];
	this->humanoid.leftLeg.angle.yaw +=
		this->configuration.walking.leftLeg.angleOffset.yaw + this->configuration.walking.com.angleOffset.yaw;

	//calculate absolute Z
	// _R_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _R_Leg_Ik[Z_Position];
	this->humanoid.rightLeg.position.z = this->configuration.walking.legLength + this->humanoid.rightLeg.position.z;
	// _L_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _L_Leg_Ik[Z_Position];
	this->humanoid.leftLeg.position.z = this->configuration.walking.legLength + this->humanoid.leftLeg.position.z;

	/**
	 * Right arm joints update
	 */

	// Speed[Id_Right_Arm_Pitch] = _R_Arm[I_A_Vp];
	this->humanoid.rightArm.velocityPitch = rightArm.velocityPitch;
	// Speed[Id_Right_Arm_Roll] = _R_Arm[I_A_Vr];
	this->humanoid.rightArm.velocityRoll = rightArm.velocityRoll;
	// Speed[Id_Right_Arm_Elbow]= _R_Arm[I_A_Ve];
	this->humanoid.rightArm.velocityElbow = rightArm.velocityElbow;

	// Angle[Id_Right_Arm_Pitch]= _R_Arm[I_A_Pitch] + WEP[P_R_Arm_Pitch_offset]
	// 	+ (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain])
	// 	+ ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoid.rightArm.pitch = rightArm.pitch + this->configuration.walking.rightArm.angleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.armGain.pitch)
		+ ((this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x) * this->configuration.walking.gyroStabilizer.armGain.pitch);
	// Angle[Id_Right_Arm_Roll] = _R_Arm[I_A_Roll]  + WEP[P_R_Arm_Roll_offset]  + (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	this->humanoid.rightArm.roll = rightArm.roll + this->configuration.walking.rightArm.angleOffset.roll
		+ (this->imu.mpu.position.y * this->configuration.walking.stabilizer.armGain.roll);

	// if (Angle[Id_Right_Arm_Roll]<-1.85)
	//	Angle[Id_Right_Arm_Roll]=-1.85;
	this->humanoid.rightArm.roll = std::max(-1.85, this->humanoid.rightArm.roll);
	//Angle[Id_Right_Arm_Elbow]= _R_Arm[I_A_Elbow] + WEP[P_R_Arm_Elbow_offset];
	this->humanoid.rightArm.elbow = rightArm.elbow + this->configuration.walking.rightArm.elbowOffset;

	/**
	 * Left arm joints update
	 */

	// Speed[Id_Left_Arm_Pitch] = _L_Arm[I_A_Vp];
	this->humanoid.leftArm.velocityPitch = leftArm.velocityPitch;
	// Speed[Id_Left_Arm_Roll] = _L_Arm[I_A_Vr];
	this->humanoid.leftArm.velocityRoll = leftArm.velocityRoll;
	// Speed[Id_Left_Arm_Elbow] = _L_Arm[I_A_Ve];
	this->humanoid.leftArm.velocityElbow = leftArm.velocityElbow;

	// Angle[Id_Left_Arm_Pitch] =
	// 	_L_Arm[I_A_Pitch] + WEP[P_L_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) +
	// 	((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	this->humanoid.leftArm.pitch = leftArm.pitch + this->configuration.walking.leftArm.angleOffset.pitch
		+ (this->imu.mpu.position.x * this->configuration.walking.stabilizer.armGain.pitch)
		+ (
			(this->imu.gyro.data.x * this->imu.gyro.data.x * this->imu.gyro.data.x)
			+ this->configuration.walking.gyroStabilizer.armGain.pitch
		);

	// Angle[Id_Left_Arm_Roll] = _L_Arm[I_A_Roll] + WEP[P_L_Arm_Roll_offset] - (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	this->humanoid.leftArm.roll = leftArm.roll + this->configuration.walking.leftArm.angleOffset.roll
		- (this->imu.mpu.position.y * this->configuration.walking.stabilizer.armGain.roll);
	// if (Angle[Id_Left_Arm_Roll] < -1.85)
	// 	Angle[Id_Left_Arm_Roll] = -1.85;
	this->humanoid.leftArm.roll = std::max(-1.85, this->humanoid.leftArm.roll);
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
			(rightLeg.position.x * std::cos(rightLeg.angle.yaw))
			+ (rightLeg.position.y * std::sin(rightLeg.angle.yaw)),
	// double tmpRightY = (_R_Leg_Ik[X_Position] * sin(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * cos(_R_Leg_Ik[Yaw_Angle]));
		tmpRightY =
			(rightLeg.position.x * std::sin(rightLeg.angle.yaw))
			+ (rightLeg.position.y * std::cos(rightLeg.angle.yaw)),
	// double Tmp_R_Z= _R_Leg_Ik[Z_Position];
		tmpRightZ = rightLeg.position.z;
	tmpAngle = atan2(tmpRightY, tmpRightZ);
	// Angle[Id_Right_Hip_Roll]  = tmpAngle + _R_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset]  - (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	this->humanoid.leftHip.angle.roll =
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
		tmpLeftX = (_L_Leg_Ik[X_Position] * cos(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * sin(_L_Leg_Ik[Yaw_Angle]));
	// double tmpLeftY = (_L_Leg_Ik[X_Position] * sin(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * cos(_L_Leg_Ik[Yaw_Angle]));
	// double tmpLeftZ =  _L_Leg_Ik[Z_Position];
	tmpAngle= atan2(tmpLeftY, tmpLeftZ);
	Angle[Id_Left_Hip_Roll]  = tmpAngle  + _L_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset]  + (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Left_Foot_Roll] =-tmpAngle  + _L_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] + (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	tmpRightZ= sqrt((tmpRightY * tmpRightY) + (tmpRightZ * tmpRightZ));
	dfoot= sqrt((tmpRightZ * tmpRightZ) + (tmpRightX * tmpRightX));
	alpha= atan2(tmpRightX, tmpRightZ);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Right_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Right_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Right_Foot_Pitch] = (-alpha + beta) + _R_Leg_Ik[Pitch_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);

	tmpRightZ= sqrt((tmpLeftY * tmpLeftY) + (tmpLeftZ * tmpLeftZ));
	dfoot= sqrt((tmpLeftZ * tmpLeftZ) + (tmpLeftX * tmpLeftX));
	alpha= atan2(tmpLeftX, tmpLeftZ);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Left_Hip_Pitch]  = (alpha + beta) + WEP[P_Left_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Left_Knee]  = (-2.0 * beta) + WEP[P_Left_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Left_Foot_Pitch]  = (-alpha + beta) + _L_Leg_Ik[Pitch_Angle] + WEP[P_Left_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
	//double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	//double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	//double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
	//double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

	//add offset to inverse kinematic
	_R_Leg_Ik[X_Position] += WEP[P_Right_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	_R_Leg_Ik[Y_Position] += WEP[P_Right_Leg_Y_Offset] + WEP[P_COM_Y_offset] - (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	_R_Leg_Ik[Z_Position] += WEP[P_Right_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	_R_Leg_Ik[Roll_Angle] += WEP[P_Right_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	_R_Leg_Ik[Pitch_Angle]+= WEP[P_Right_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	_R_Leg_Ik[Yaw_Angle]  += WEP[P_Right_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];

	_L_Leg_Ik[X_Position] += WEP[P_Left_Leg_X_Offset] + WEP[P_COM_X_offset] + (MPU_X * WEP[P_Stablizer_COM_X_Shift_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_COM_X_Shift_Gain]);
	_L_Leg_Ik[Y_Position] += WEP[P_Left_Leg_Y_Offset] + WEP[P_COM_Y_offset] + (MPU_Y * WEP[P_Stablizer_COM_Y_Shift_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_COM_Y_Shift_Gain]);
	_L_Leg_Ik[Z_Position] += WEP[P_Left_Leg_Z_Offset] + WEP[P_COM_Z_offset];
	_L_Leg_Ik[Roll_Angle] += WEP[P_Left_Leg_Roll_Offset]  + WEP[P_COM_Roll_offset];
	_L_Leg_Ik[Pitch_Angle]+= WEP[P_Left_Leg_Pitch_Offset] + WEP[P_COM_Pitch_offset];
	_L_Leg_Ik[Yaw_Angle]  += WEP[P_Left_Leg_Yaw_Offset]   + WEP[P_COM_Yaw_offset];

	//calculate absolute Z
	_R_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _R_Leg_Ik[Z_Position];
	_L_Leg_Ik[Z_Position]= WEP[P_Leg_Length] - _L_Leg_Ik[Z_Position];

	//right arm joints update
	Speed[Id_Right_Arm_Pitch]= _R_Arm[I_A_Vp];
	Speed[Id_Right_Arm_Roll] = _R_Arm[I_A_Vr];
	Speed[Id_Right_Arm_Elbow]= _R_Arm[I_A_Ve];
	Angle[Id_Right_Arm_Pitch]= _R_Arm[I_A_Pitch] + WEP[P_R_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) + ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	Angle[Id_Right_Arm_Roll] = _R_Arm[I_A_Roll]  + WEP[P_R_Arm_Roll_offset]  + (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	if (Angle[Id_Right_Arm_Roll]<-1.85) Angle[Id_Right_Arm_Roll]=-1.85;
	Angle[Id_Right_Arm_Elbow]= _R_Arm[I_A_Elbow] + WEP[P_R_Arm_Elbow_offset];

	//left arm joints update
	Speed[Id_Left_Arm_Pitch]= _L_Arm[I_A_Vp];
	Speed[Id_Left_Arm_Roll] = _L_Arm[I_A_Vr];
	Speed[Id_Left_Arm_Elbow]= _L_Arm[I_A_Ve];
	Angle[Id_Left_Arm_Pitch]= _L_Arm[I_A_Pitch] + WEP[P_L_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) + ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	Angle[Id_Left_Arm_Roll] = _L_Arm[I_A_Roll]  + WEP[P_L_Arm_Roll_offset]  - (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	if (Angle[Id_Left_Arm_Roll]<-1.85) Angle[Id_Left_Arm_Roll]=-1.85;
	Angle[Id_Left_Arm_Elbow]= _L_Arm[I_A_Elbow] + WEP[P_L_Arm_Elbow_offset];

	//set right leg speeds
	Speed[Id_Right_Hip_Yaw]    = _R_Leg_Speed;
	Speed[Id_Right_Hip_Roll]   = _R_Leg_Speed;
	Speed[Id_Right_Hip_Pitch]  = _R_Leg_Speed/2.0;
	Speed[Id_Right_Knee]       = _R_Leg_Speed;
	Speed[Id_Right_Foot_Pitch] = _R_Leg_Speed/2.0;
	Speed[Id_Right_Foot_Roll]  = _R_Leg_Speed;

	//set left leg speeds
	Speed[Id_Left_Hip_Yaw]    = _L_Leg_Speed;
	Speed[Id_Left_Hip_Roll]   = _L_Leg_Speed;
	Speed[Id_Left_Hip_Pitch]  = _L_Leg_Speed/2.0;
	Speed[Id_Left_Knee]       = _L_Leg_Speed;
	Speed[Id_Left_Foot_Pitch] = _L_Leg_Speed/2.0;
	Speed[Id_Left_Foot_Roll]  = _L_Leg_Speed;

	//set legs yaw
	Angle[Id_Right_Hip_Yaw]= _R_Leg_Ik[Yaw_Angle] + WEP[P_Right_Leg_Hip_Yaw_Offset];
	Angle[Id_Left_Hip_Yaw] = _L_Leg_Ik[Yaw_Angle] + WEP[P_Left_Leg_Hip_Yaw_Offset];

	double Tmp_angle=0.0;
	double dfoot,alpha,beta;

	double Tmp_R_X= (_R_Leg_Ik[X_Position] * cos(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * sin(_R_Leg_Ik[Yaw_Angle]));
	double Tmp_R_Y= (_R_Leg_Ik[X_Position] * sin(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * cos(_R_Leg_Ik[Yaw_Angle]));
	double Tmp_R_Z= _R_Leg_Ik[Z_Position];
	tmpAngle= atan2(tmpRightY, tmpRightZ);
	Angle[Id_Right_Hip_Roll]  = tmpAngle + _R_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset]  - (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Right_Foot_Roll] =-tmpAngle + _R_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Right_Leg_Foot_Roll_Offset] - (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	double Tmp_L_X= (_L_Leg_Ik[X_Position] * cos(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * sin(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Y= (_L_Leg_Ik[X_Position] * sin(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * cos(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Z=  _L_Leg_Ik[Z_Position];
	tmpAngle= atan2(tmpLeftY, tmpLeftZ);
	Angle[Id_Left_Hip_Roll]  = tmpAngle  + _L_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset]  + (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Left_Foot_Roll] =-tmpAngle  + _L_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] + (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	tmpRightZ= sqrt((tmpRightY * tmpRightY) + (tmpRightZ * tmpRightZ));
	dfoot= sqrt((tmpRightZ * tmpRightZ) + (tmpRightX * tmpRightX));
	alpha= atan2(tmpRightX, tmpRightZ);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Right_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Right_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Right_Foot_Pitch] = (-alpha + beta) + _R_Leg_Ik[Pitch_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);

	tmpRightZ= sqrt((tmpLeftY * tmpLeftY) + (tmpLeftZ * tmpLeftZ));
	dfoot= sqrt((tmpLeftZ * tmpLeftZ) + (tmpLeftX * tmpLeftX));
	alpha= atan2(tmpLeftX, tmpLeftZ);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Left_Hip_Pitch]  = (alpha + beta) + WEP[P_Left_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Left_Knee]  = (-2.0 * beta) + WEP[P_Left_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Left_Foot_Pitch]  = (-alpha + beta) + _L_Leg_Ik[Pitch_Angle] + WEP[P_Left_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
}
