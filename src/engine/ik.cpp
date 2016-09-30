/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 23, 2016
 */

#include "ik.h"

mote::walking::HumanoidIK::HumanoidIK(mote::walking::Humanoid &humanoid, mote::walking::Configuration &configuration,
	sensors::IMU &imu)
	: humanoid(humanoid), configuration(configuration), imu(imu)
{ }

void mote::walking::HumanoidIK::update(double _R_Leg_Speed, double _L_Leg_Speed, double *_R_Leg_Ik, double *_L_Leg_Ik,
	double *_R_Arm, double *_L_Arm)
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

	//right arm joints update
	Speed[Id_Right_Arm_Pitch]= _R_Arm[I_A_Vp];
	Speed[Id_Right_Arm_Roll] = _R_Arm[I_A_Vr];
	Speed[Id_Right_Arm_Elbow]= _R_Arm[I_A_Ve];
	Angle[Id_Right_Arm_Pitch]= _R_Arm[I_A_Pitch] + WEP[P_R_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) + ((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	Angle[Id_Right_Arm_Roll] = _R_Arm[I_A_Roll]  + WEP[P_R_Arm_Roll_offset]  + (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	if (Angle[Id_Right_Arm_Roll]<-1.85)
		Angle[Id_Right_Arm_Roll]=-1.85;
	Angle[Id_Right_Arm_Elbow]= _R_Arm[I_A_Elbow] + WEP[P_R_Arm_Elbow_offset];

	//left arm joints update
	Speed[Id_Left_Arm_Pitch] = _L_Arm[I_A_Vp];
	Speed[Id_Left_Arm_Roll] = _L_Arm[I_A_Vr];
	Speed[Id_Left_Arm_Elbow] = _L_Arm[I_A_Ve];
	Angle[Id_Left_Arm_Pitch] =
		_L_Arm[I_A_Pitch] + WEP[P_L_Arm_Pitch_offset] + (MPU_X * WEP[P_Stablizer_Arm_Pitch_Gain]) +
		((Gyro_X * Gyro_X * Gyro_X) * WEP[P_Gyro_Stablizer_Arm_Pitch_Gain]);
	Angle[Id_Left_Arm_Roll] = _L_Arm[I_A_Roll] + WEP[P_L_Arm_Roll_offset] - (MPU_Y * WEP[P_Stablizer_Arm_Roll_Gain]);
	if (Angle[Id_Left_Arm_Roll] < -1.85) Angle[Id_Left_Arm_Roll] = -1.85;
	Angle[Id_Left_Arm_Elbow] = _L_Arm[I_A_Elbow] + WEP[P_L_Arm_Elbow_offset];

	//set right leg speeds
	Speed[Id_Right_Hip_Yaw] = _R_Leg_Speed;
	Speed[Id_Right_Hip_Roll] = _R_Leg_Speed;
	Speed[Id_Right_Hip_Pitch] = _R_Leg_Speed / 2.0;
	Speed[Id_Right_Knee] = _R_Leg_Speed;
	Speed[Id_Right_Foot_Pitch] = _R_Leg_Speed / 2.0;
	Speed[Id_Right_Foot_Roll] = _R_Leg_Speed;

	//set left leg speeds
	Speed[Id_Left_Hip_Yaw] = _L_Leg_Speed;
	Speed[Id_Left_Hip_Roll] = _L_Leg_Speed;
	Speed[Id_Left_Hip_Pitch] = _L_Leg_Speed / 2.0;
	Speed[Id_Left_Knee] = _L_Leg_Speed;
	Speed[Id_Left_Foot_Pitch] = _L_Leg_Speed / 2.0;
	Speed[Id_Left_Foot_Roll] = _L_Leg_Speed;

	//set legs yaw
	Angle[Id_Right_Hip_Yaw] = _R_Leg_Ik[Yaw_Angle] + WEP[P_Right_Leg_Hip_Yaw_Offset];
	Angle[Id_Left_Hip_Yaw] = _L_Leg_Ik[Yaw_Angle] + WEP[P_Left_Leg_Hip_Yaw_Offset];

	double Tmp_angle=0.0;
	double dfoot,alpha,beta;

	double Tmp_R_X= (_R_Leg_Ik[X_Position] * cos(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * sin(_R_Leg_Ik[Yaw_Angle]));
	double Tmp_R_Y= (_R_Leg_Ik[X_Position] * sin(_R_Leg_Ik[Yaw_Angle]) + _R_Leg_Ik[Y_Position] * cos(_R_Leg_Ik[Yaw_Angle]));
	double Tmp_R_Z= _R_Leg_Ik[Z_Position];
	Tmp_angle= atan2(Tmp_R_Y, Tmp_R_Z);
	Angle[Id_Right_Hip_Roll]  = Tmp_angle + _R_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset]  - (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Right_Foot_Roll] =-Tmp_angle + _R_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Right_Leg_Foot_Roll_Offset] - (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	double Tmp_L_X= (_L_Leg_Ik[X_Position] * cos(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * sin(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Y= (_L_Leg_Ik[X_Position] * sin(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * cos(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Z=  _L_Leg_Ik[Z_Position];
	Tmp_angle= atan2(Tmp_L_Y, Tmp_L_Z);
	Angle[Id_Left_Hip_Roll]  = Tmp_angle  + _L_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset]  + (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Left_Foot_Roll] =-Tmp_angle  + _L_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] + (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	Tmp_R_Z= sqrt((Tmp_R_Y * Tmp_R_Y) + (Tmp_R_Z * Tmp_R_Z));
	dfoot= sqrt((Tmp_R_Z * Tmp_R_Z) + (Tmp_R_X * Tmp_R_X));
	alpha= atan2(Tmp_R_X, Tmp_R_Z);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Right_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Right_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Right_Foot_Pitch] = (-alpha + beta) + _R_Leg_Ik[Pitch_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);

	Tmp_R_Z= sqrt((Tmp_L_Y * Tmp_L_Y) + (Tmp_L_Z * Tmp_L_Z));
	dfoot= sqrt((Tmp_L_Z * Tmp_L_Z) + (Tmp_L_X * Tmp_L_X));
	alpha= atan2(Tmp_L_X, Tmp_L_Z);
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
	Tmp_angle= atan2(Tmp_R_Y, Tmp_R_Z);
	Angle[Id_Right_Hip_Roll]  = Tmp_angle + _R_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Right_Leg_Hip_Roll_Offset]  - (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  - (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Right_Foot_Roll] =-Tmp_angle + _R_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Right_Leg_Foot_Roll_Offset] - (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) - (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	double Tmp_L_X= (_L_Leg_Ik[X_Position] * cos(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * sin(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Y= (_L_Leg_Ik[X_Position] * sin(_L_Leg_Ik[Yaw_Angle]) + _L_Leg_Ik[Y_Position] * cos(_L_Leg_Ik[Yaw_Angle]));
	double Tmp_L_Z=  _L_Leg_Ik[Z_Position];
	Tmp_angle= atan2(Tmp_L_Y, Tmp_L_Z);
	Angle[Id_Left_Hip_Roll]  = Tmp_angle  + _L_Leg_Ik[Roll_Angle] / 2.0 + WEP[P_Left_Leg_Hip_Roll_Offset]  + (MPU_Y * WEP[P_Stablizer_Hip_Roll_Gain])  + (Gyro_Y * WEP[P_Gyro_Stablizer_Hip_Roll_Gain]);
	Angle[Id_Left_Foot_Roll] =-Tmp_angle  + _L_Leg_Ik[Roll_Angle] / 4.0 + WEP[P_Left_Leg_Foot_Roll_Offset] + (MPU_Y * WEP[P_Stablizer_Foot_Roll_Gain]) + (Gyro_Y * WEP[P_Gyro_Stablizer_Foot_Roll_Gain]);

	Tmp_R_Z= sqrt((Tmp_R_Y * Tmp_R_Y) + (Tmp_R_Z * Tmp_R_Z));
	dfoot= sqrt((Tmp_R_Z * Tmp_R_Z) + (Tmp_R_X * Tmp_R_X));
	alpha= atan2(Tmp_R_X, Tmp_R_Z);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Right_Hip_Pitch] = (alpha + beta) + WEP[P_Right_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Right_Knee] = (-2.0 * beta) + WEP[P_Right_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Right_Foot_Pitch] = (-alpha + beta) + _R_Leg_Ik[Pitch_Angle] + WEP[P_Right_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);

	Tmp_R_Z= sqrt((Tmp_L_Y * Tmp_L_Y) + (Tmp_L_Z * Tmp_L_Z));
	dfoot= sqrt((Tmp_L_Z * Tmp_L_Z) + (Tmp_L_X * Tmp_L_X));
	alpha= atan2(Tmp_L_X, Tmp_L_Z);
	beta= acos(dfoot / WEP[P_Leg_Length]);
	Angle[Id_Left_Hip_Pitch]  = (alpha + beta) + WEP[P_Left_Leg_Hip_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Hip_Pitch_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Hip_Pitch_Gain]);
	Angle[Id_Left_Knee]  = (-2.0 * beta) + WEP[P_Left_Leg_Knee_Offset] + (MPU_X * WEP[P_Stablizer_Knee_Gain]) + (Gyro_X * WEP[P_Gyro_Stablizer_Knee_Gain]);
	Angle[Id_Left_Foot_Pitch]  = (-alpha + beta) + _L_Leg_Ik[Pitch_Angle] + WEP[P_Left_Leg_Foot_Pitch_Offset] + (MPU_X * WEP[P_Stablizer_Foot_Pitch_Gain]) + ((Gyro_X*Gyro_X*Gyro_X) * WEP[P_Gyro_Stablizer_Foot_Pitch_Gain]);
}
