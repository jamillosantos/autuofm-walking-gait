/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#include "robot.h"

void mote::walking::Robot::run()
{
	this->_running = true;

	this->init();
	this->velocity.zero();
	// Buzzer(200);

	VERBOSE("Stand init 1!");
	this->standInitT(0.05, 1);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	//stand for first time
	VERBOSE("Stand init 2!");
	this->standInitT(0.05, 300);

	this->Check_Robot_Fall = 1; // Check_Robot_Fall==1;??

	VERBOSE("Starting main loop!");
	//main task loop
	while (this->_running)
	{
		// VERBOSE("Tick!");
		// togglePin(RED_LED_485EXP);

		if ((
				(
					(this->velocity.x != 0)
					|| (this->velocity.y != 0)
					|| (this->velocity.theta != 0)
				)
				&& (this->Motion_Ins == No_Motion)
				&& (this->Internal_Motion_Request == No_Motion)
			)
			// OLD LINE: && (this->System_Voltage >= (int)WEP[P_Min_Voltage_Limit])
			// TODO Uncomment "NEW LINE" (below) after plugging in the system voltage controller
			// NEW LINE: && (this->System_Voltage >= this->configuration.walking.minVoltage)
		) {
			this->standInitT(0.5, 50);

			//start gait
			// WEP[P_Left_Leg_Y_Offset]=-50;
			this->configuration.walking.leftLeg.positionOffset.y  = -50;
			// WEP[P_Right_Leg_Y_Offset]=50;
			this->configuration.walking.rightLeg.positionOffset.y = 50;

			this->standInitT(1.0, 4);  // last one was 2

			// NEW: Probably we will not need this line anymore, since all parameters are loaded when it starts.
			// Set_Walk_Engine_Parameters((byte)Teen_Size_Robot_Num);

			//this->omniGait(WEP[Vx_Offset], WEP[Vy_Offset], WEP[Vt_Offset]); //execute omni-directional start gait
			this->omniGait(
				this->configuration.walking.velocityOffset.x,
				this->configuration.walking.velocityOffset.y,
				this->configuration.walking.velocityOffset.theta
			); //execute omni-directional start gait

			//main gait
			while((
					(
						(this->velocity.x != 0)
						|| (this->velocity.y != 0)
						|| (this->velocity.theta != 0)
					)
					&& (this->Motion_Ins == No_Motion)
					&& (this->Internal_Motion_Request == No_Motion)
				  )
				  // && (this->System_Voltage >= (int)WEP[P_Min_Voltage_Limit])
				  && (this->System_Voltage >= this->configuration.walking.minVoltage)
			)
				// this->omniGait(Vx + WEP[Vx_Offset], Vy + WEP[Vy_Offset], Vt + WEP[Vt_Offset]); // execute omni-directional gait
				this->omniGait(
					this->velocity.x + this->configuration.walking.velocityOffset.x,
					this->velocity.y + this->configuration.walking.velocityOffset.y,
					this->velocity.theta + this->configuration.walking.velocityOffset.theta
				); // execute omni-directional gait

			//finish gate
			// this->omniGait(WEP[Vx_Offset], WEP[Vy_Offset], WEP[Vt_Offset]); //execute omni-directional end gait
			this->omniGait(
				this->configuration.walking.velocityOffset.x,
				this->configuration.walking.velocityOffset.y,
				this->configuration.walking.velocityOffset.theta
			); //execute omni-directional end gait
		}
		else
		{
			//firts run gait generation with vx=vy=vt=0 for semi gait generation to stand position
			if (this->Internal_Motion_Request == No_Motion)
				this->standInit(0.5);

			if (this->Internal_Motion_Request == Stop_Motion)
				this->standInit(0.05);

			//vTaskDelay(20);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));

			//run standup motions
			switch(this->Internal_Motion_Request)
			{ //check for instrcation
				case Stand_Up_Front:
					//run stand up
					// vTaskDelay(1000);
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					this->Check_Robot_Fall = 0;
					this->Actuators_Update = 1;
					this->standInitT(1.0, 10);

					// TODO: Uncomment line below!
					// Motion_Stand_Up_Front((byte)Teen_Size_Robot_Num);

					this->Internal_Motion_Request = No_Motion;
					this->standInitT(1.0, 10);
					this->Check_Robot_Fall = 1;
					break;

				case Stand_Up_Back:
					//run stand up
					// vTaskDelay(1000);
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					Check_Robot_Fall = 0;
					Actuators_Update = 1;
					this->standInitT(1.0, 10);

					// TODO: Uncomment line below!
					// Motion_Stand_Up_Back((byte)Teen_Size_Robot_Num);

					Internal_Motion_Request = No_Motion;
					this->standInitT(1.0, 10);
					Check_Robot_Fall=1;
					break;
			}

			// TODO: Add support for recorded motions.
			/*
			//run the motion
			switch(Motion_Ins)
			{  //check for instrcation
				case (byte)Motion_1:
					//run motion 1
					//initialize head joint
					this->setHead(0,0.5,500,500);
					this->standInitT(1.0, 25);
					Run_R_Kik_Motion((byte)Teen_Size_Robot_Num);
					this->standInitT(1.0, 70);
					Motion_Ins=No_Motion;
					break;

				case (byte)Motion_2:
					//run motion 2
					this->setHead(0,0.5,500,500);
					this->standInitT(1.0, 25);
					Run_L_Kik_Motion((byte)Teen_Size_Robot_Num);
					this->standInitT(1.0, 70);
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_3:
					this->standInitT(1.0, 10);
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_4:
					this->standInitT(1.0, 10);
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_5:
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_6:
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_7:
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_8:
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_9:
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_10:
					Motion_Ins = No_Motion;
					break;
			}//external motion request switch
			*/
		}//main if/else
		// this->humanoid.dump();
	}
}

void mote::walking::Robot::init()
{
	VERBOSE("Initialization started!");
	this->standInit(0.1);  //ititialie robot to stand position

	this->velocity.zero();
	this->velocity.theta = 0;

	Motion_Ins = No_Motion;
	Internal_Motion_Request = No_Motion;
	VERBOSE("ok!");
}

void mote::walking::Robot::standInit(double speed)
{
	Leg
		leftLeg, rightLeg;
	Arm
		leftArm, rightArm;

	/*
	double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
	double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve
	*/

	leftLeg.zero();
	rightLeg.zero();
	leftArm.zero();
	rightArm.zero();

	leftLeg.zero();
	rightLeg.zero();

	leftArm.zero();
	leftArm.velocity.pitch = leftArm.velocity.roll = leftArm.velocityElbow = speed;

	//left arm initialize
	rightArm.zero();
	rightArm.velocity.pitch = rightArm.velocity.roll = rightArm.velocityElbow = speed;

	//update robotis joints
	// this->Update_Ik(_Speed, _Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
	this->ik.update(speed, speed, rightLeg, leftLeg, rightArm, leftArm);
}

//initialize robot to stand
void mote::walking::Robot::standInitT(double speed, int time)
{
	Leg
		L_Leg_Ik,  // x, y, z, roll, pitch, yaw
		R_Leg_Ik;  // x, y, z, roll, pitch, yaw
	Arm
		L_Arm,     // pitch, roll, elbow, vp, vr, ve
		R_Arm;     // pitch, roll, elbow, vp, vr, ve

		//update robotis joints
	for (unsigned int i = 0; i <= time; i++)
	{
		//right leg initialize
		// R_Leg_Ik.position.x[I_X] = 0.0;
		// R_Leg_Ik.position.y[I_Y] = 0.0;
		// R_Leg_Ik.position.z[I_Z] = 0.0;
		// R_Leg_Ik[I_Roll] = 0.0;
		// R_Leg_Ik[I_Pitch] = 0.0;
		// R_Leg_Ik[I_Yaw] = 0.0;
		R_Leg_Ik.zero();

		//left leh initialize
		// L_Leg_Ik[I_X] = 0.0;
		// L_Leg_Ik[I_Y] = 0.0;
		// L_Leg_Ik[I_Z] = 0.0;
		// L_Leg_Ik[I_Roll] = 0.0;
		// L_Leg_Ik[I_Pitch] = 0.0;
		// L_Leg_Ik[I_Yaw] = 0.0;
		L_Leg_Ik.zero();

		//right arm initialize
		// R_Arm[I_A_Pitch] = 0.0;
		// R_Arm[I_A_Roll] = 0.0;
		// R_Arm[I_A_Elbow] = 0.0;
		R_Arm.angle.zero();
		R_Arm.elbow = 0.0;

		// R_Arm[I_A_Vp] = _Speed;
		// R_Arm[I_A_Vr] = _Speed;
		// R_Arm[I_A_Ve] = _Speed;
		R_Arm.velocity.set(speed);
		R_Arm.velocityElbow = speed;

		//left arm initialize
		// L_Arm[I_A_Pitch] = 0.0;
		// L_Arm[I_A_Roll] = 0.0;
		// L_Arm[I_A_Elbow] = 0.0;
		L_Arm.angle.zero();
		L_Arm.elbow = 0.0;
		// L_Arm[I_A_Vp] = _Speed;
		// L_Arm[I_A_Vr] = _Speed;
		// L_Arm[I_A_Ve] = _Speed;
		L_Arm.velocity.set(speed);
		L_Arm.velocityElbow = speed;

		this->ik.update(speed, speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		//vTaskDelay(20);
		std::cerr << ".";
	}
	std::cerr << std::endl;
}

//omni directional gaite generation
void mote::walking::Robot::omniGait(double vx, double vy, double vt)
{
	Leg
		leftLeg,  // x, y, z, roll, pitch, yaw
		rightLeg;  // x, y, z, roll, pitch, yaw
	Arm
		leftArm,     // pitch, roll, elbow, vp, vr, ve
		rightArm;     // pitch, roll, elbow, vp, vr, ve

	double Joint_Speed = 0.3;

	/**
	 * gait generate with for loop form 0 ~ 2 * 3.14
	 */
	// for (double t = 0; t <= TwoPi ; t += WEP[P_Motion_Resolution])
	for (double t = 0; t <= M_2xPIl; t += this->configuration.walking.engine.motionResolution)
	{
		if (vx > 0.3)
			vx = 0.3;
		if (vx < -0.3)
			vx = -0.3;

		if (vy > 0.4)
			vy = 0.4;
		if (vy < -0.4)
			vy = -0.4;

		if (vt > 0.05)
			vt = 0.05;
		if (vt < -0.05)
			vt = -0.05;

		//-----------------------
		if (
			(
				// (t >= Pi-WEP[P_Motion_Resolution])
				(t >= (M_PIl - this->configuration.walking.engine.motionResolution))
				// && (t <= Pi+WEP[P_Motion_Resolution])
				&& (t <= (M_PIl + this->configuration.walking.engine.motionResolution))
			)
			|| (
				//(t >= TwoPi-WEP[P_Motion_Resolution])
				(t >= (M_2xPIl - this->configuration.walking.engine.motionResolution))
				// &&(t <= TwoPi+WEP[P_Motion_Resolution])
				&& (t <= M_2xPIl + this->configuration.walking.engine.motionResolution)
			)
			|| (t == 0.0)
		)
		{
			// vTaskDelay(WEP[P_Double_Support_Sleep]);
			std::this_thread::sleep_for(std::chrono::milliseconds(this->configuration.walking.engine.doubleSupportSleep));
		}

		if (
			(
				// (t >= (Pi/2.0) - WEP[P_Motion_Resolution])
				(t >= (M_PI_2l - this->configuration.walking.engine.motionResolution))
				// && (t <= (Pi/2.0)+WEP[P_Motion_Resolution])
				&& (t <= (M_PI_2l + this->configuration.walking.engine.motionResolution))
			)
			|| (
				// (t >= ((3.0*Pi)/2.0)-WEP[P_Motion_Resolution])
				(t >= (M_32_PIl - this->configuration.walking.engine.motionResolution))
				// && (t <= ((3.0*Pi)/2.0)+WEP[P_Motion_Resolution])
				&& (t <= (M_32_PIl + this->configuration.walking.engine.motionResolution))
			)
		)
		{
			// vTaskDelay(WEP[P_Single_Support_Sleep]);
			std::this_thread::sleep_for(std::chrono::milliseconds(this->configuration.walking.engine.singleSupportSleep));
		}

		/**
		 * Right leg initialization
		 */
		//if t<pi the right leg in fly state and other wise in support state
		int S_X = 1;
		if (vx < 0.0)
			S_X = 0.2;

		/** This code was added by Jamillo for different offsets between forward speed and backward speed.
		 *  It will not be added to this version of the walking gait.
		if (vx > 0)
		{
			WEP[P_Left_Leg_Hip_Pitch_Offset] = WEP[P_Left_Leg_Hip_Pitch_Offset_Original];
			WEP[P_Right_Leg_Hip_Pitch_Offset] = WEP[P_Right_Leg_Hip_Pitch_Offset_Original];
		}
		else if (vx < 0)
		{
			WEP[P_Left_Leg_Hip_Pitch_Offset] = WEP[P_Left_Leg_Hip_Pitch_Offset_Backwards];
			WEP[P_Right_Leg_Hip_Pitch_Offset] = WEP[P_Right_Leg_Hip_Pitch_Offset_Backwards];
		}
		*/

		// R_Leg_Ik[I_X]     = (-(cos(t)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * S_X * 100.0);
		rightLeg.position.x =
			(-(std::cos(t) * (vx * 100.0)))
			+ (vx * this->configuration.walking.engine.bodySwingGain.x * S_X * 100.0);
		// R_Leg_Ik[I_Y]     = (t<=Pi) ? (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) + (cos(t-Pi)*(vy*50.0))
		// 							: (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
		rightLeg.position.y =
			(t <= M_PIl)
				? (std::sin(t) * (this->configuration.walking.engine.bodySwingGain.y * 100.0))
				  + (std::sin(t) * (this->configuration.walking.engine.flySwingGain.y * 100.0))
				  + (std::cos(t - M_PIl) * (vy * 50.0))
				: (std::sin(t) * (this->configuration.walking.engine.bodySwingGain.y * 100.0))
				  + (std::cos(t - M_PIl) * (vy * 50.0)) + (std::sin(t) * this->configuration.walking.engine.supportSwingGain.y);
		// R_Leg_Ik[I_Z]     = (t<=Pi) ? (sin(t)*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : (sin(t)*(WEP[P_Support_Z_Swing_Gain]*100.0));
		rightLeg.position.z =
			(t <= M_PIl)
			? (std::sin(t) * (this->configuration.walking.engine.flySwingGain.z * 100.0))
			: (std::sin(t) * (this->configuration.walking.engine.supportSwingGain.z * 100.0));
		// R_Leg_Ik[I_Roll]  = (t<=Pi) ? (-sin(t)*(WEP[P_Fly_Roll_Gain])) : (-sin(t)*(WEP[P_Support_Roll_Gain]));
		rightLeg.angle.roll =
			(t <= M_PIl)
			? (-std::sin(t) * this->configuration.walking.engine.flyGain.roll)
			: (-std::sin(t) * this->configuration.walking.engine.supportGain.roll);
		// R_Leg_Ik[I_Pitch] = (t<=Pi) ? 0.0 : (sin(t)*WEP[P_Support_Pitch_Gain]);
		rightLeg.angle.pitch =
			(t <= M_PIl)
			? 0.0
			: (std::sin(t) * this->configuration.walking.engine.supportGain.pitch);
		// R_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt));
		rightLeg.angle.yaw = (std::cos(t - M_PIl) * (vt));

		/**
		 * Left leg initialization
		 */
		// L_Leg_Ik[I_X]     = (-(cos(t-Pi)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * S_X * 100.0);
		leftLeg.position.x =
			(-(std::cos(t - M_PIl) * (vx * 100.0)))
			+ (vx * this->configuration.walking.engine.bodySwingGain.x * S_X * 100.0);
		// L_Leg_Ik[I_Y]     = (t>=Pi) ? (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (-sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) +(cos(t-Pi)*(vy*50.0))
		// 							: (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
		leftLeg.position.y =
			(t >= M_PIl)
			? (-std::sin(t) * (this->configuration.walking.engine.bodySwingGain.y * 100.0))
			  + (-std::sin(t) * (this->configuration.walking.engine.flySwingGain.y * 100.0))
			  + (std::cos(t - M_PIl) * (vy * 50.0))
			: (-std::sin(t) * (this->configuration.walking.engine.bodySwingGain.y * 100.0))
			  + (std::cos(t - M_PIl) * (vy * 50.0))
			  + (std::sin(t) * this->configuration.walking.engine.supportSwingGain.y);
		// L_Leg_Ik[I_Z]     = (t>=Pi) ? ((sin(t-Pi))*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : ((sin(t-Pi))*(WEP[P_Support_Z_Swing_Gain]*100.0));
		leftLeg.position.z =
			(t >= M_PIl)
			? ((std::sin(t - M_PIl)) * (this->configuration.walking.engine.flySwingGain.z * 100.0))
			: ((std::sin(t - M_PIl)) * (this->configuration.walking.engine.supportSwingGain.z * 100.0));
		// L_Leg_Ik[I_Roll]  = (t>=Pi) ? (sin(t)*(WEP[P_Fly_Roll_Gain])) : (sin(t)*(WEP[P_Support_Roll_Gain]));
		leftLeg.angle.roll =
			(t >= M_PIl)
			? (std::sin(t) * (this->configuration.walking.engine.flyGain.roll))
			: (std::sin(t) * (this->configuration.walking.engine.supportGain.roll));
		// L_Leg_Ik[I_Pitch] = (t>=Pi) ? 0.0 : (sin(t-Pi)*WEP[P_Support_Pitch_Gain]);
		leftLeg.angle.pitch =
			(t >= M_PIl)
			? 0.0
			: (std::sin(t - M_PIl) * this->configuration.walking.engine.supportGain.pitch);
		// L_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt)); //(cos(t-Pi)*(vt));
		leftLeg.angle.yaw = (std::cos(t - M_PIl) * (vt));

		//right arm initialize
		// R_Arm[I_A_Pitch]   = ((cos(t)) * vx * 1.995); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		rightArm.angle.pitch = ((std::cos(t)) * vx * 1.995); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		// R_Arm[I_A_Roll]    = 0.0;
		rightArm.angle.roll = 0.0;
		// R_Arm[I_A_Elbow]   = 0.0;
		rightArm.elbow = 0.0;
		// R_Arm[I_A_Vp]      = 0.2;
		rightArm.velocity.pitch = 0.2;
		// R_Arm[I_A_Vr]      = 0.05;
		rightArm.velocity.roll = 0.05;
		// R_Arm[I_A_Ve]      = 0.05;
		rightArm.velocityElbow = 0.05;

		//left arm initialize
		// L_Arm[I_A_Pitch]   = ((cos(t-Pi)) * vx * 1.995); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		leftArm.angle.pitch = ((std::cos(t - M_PIl)) * vx * 1.995); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		// L_Arm[I_A_Roll]    = 0.0;
		leftArm.angle.roll = 0.0;
		// L_Arm[I_A_Elbow]   = 0.0;
		leftArm.elbow   = 0.0;
		// L_Arm[I_A_Vp]      = 0.2;
		leftArm.velocity.pitch = 0.2;
		// L_Arm[I_A_Vr]      = 0.05;
		leftArm.velocity.roll = 0.05;
		// L_Arm[I_A_Ve]      = 0.05;
		leftArm.velocityElbow = 0.05;

		//update robotis joints
		// Update_Ik(Joint_Speed, Joint_Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
		this->ik.update(Joint_Speed, Joint_Speed, rightLeg, leftLeg, rightArm, leftArm);
		// vTaskDelay(WEP[P_Gait_Frequency]*100);
		std::this_thread::sleep_for(std::chrono::milliseconds(
			(unsigned int) std::round(this->configuration.walking.engine.gaitFrequency * 100))
		);
	}//main gait timi for ins
}

//get robot state for fall detection
//byte Get_Robot_State(double Roll, double Pitch){
mote::walking::RobotState mote::walking::Robot::getRobotState(double roll, double pitch)
{
//	if(Roll>=WEP[P_Fall_Roll_Thershold]){
//		return Fallen_Left;
//	}
	if(roll >= this->configuration.walking.fall.rollThreshold)
		return mote::walking::RobotState::FallenLeft;
//	if(Roll<=(-WEP[P_Fall_Roll_Thershold])){
//		return Fallen_Right;
//	}
	if (roll <= (-this->configuration.walking.fall.rollThreshold))
		return mote::walking::RobotState::FallenRight;
//	if(Pitch>=WEP[P_Fall_Pitch_Thershold]){
//		return Fallen_Front;
//	}
	if (pitch >= this->configuration.walking.fall.pitchThreshold)
		return mote::walking::RobotState::FallenFront;
//	if(Pitch<=(-WEP[P_Fall_Pitch_Thershold])){
//		return Fallen_Back;
//	}
	if(pitch <= (-this->configuration.walking.fall.pitchThreshold))
		return mote::walking::RobotState::FallenBack;

//	return Normal_Stand;
	return mote::walking::RobotState::NormalStand;
}

//check robot state and run the stand function
// void Robot_State(){
void mote::walking::Robot::checkRobotState()
{
//	switch (Get_Robot_State(MPU_Y, MPU_X)){
	mote::walking::RobotState state = this->getRobotState(this->imu.mpu.position.y, this->imu.mpu.position.x);
	if (state != RobotState::NormalStand)
	{
		// run stand motion from front

		Check_Robot_Fall = 0;

		// TODO: Turn all motors off, but head motors, when falling (uncomment the following code).

		// vTaskSuspendAll();

		// Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
		// Dxl.writeByte(Id_Head_Pan,P_TORQUE_ENABLE,1);
		// Dxl.writeByte(Id_Head_Tilt,P_TORQUE_ENABLE,1);

		switch (state)
		{
			case mote::walking::RobotState::FallenFront:
			case mote::walking::RobotState::FallenRight:
				this->setHead(0, -0.4, 1023, 1023);
				break;
			case mote::walking::RobotState::FallenBack:
			case mote::walking::RobotState::FallenLeft:
				this->setHead(0, 0.4, 1023, 1023);
				break;
		}

		// TODO: Set the position of the head and enable hip. (Uncomment this part).
		/*
		Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
		Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);

		Dxl.writeByte(Id_Right_Hip_Yaw,P_TORQUE_ENABLE,1);
		Dxl.writeByte(Id_Left_Hip_Yaw,P_TORQUE_ENABLE,1);

		Dxl.writeByte(Id_Right_Hip_Roll,P_TORQUE_ENABLE,1);
		Dxl.writeByte(Id_Left_Hip_Roll,P_TORQUE_ENABLE,1);

		xTaskResumeAll();
		*/

		switch (state)
		{
			case mote::walking::RobotState::FallenFront:
			case mote::walking::RobotState::FallenRight:
			case mote::walking::RobotState::FallenLeft:
				Internal_Motion_Request = Stand_Up_Front;
				break;
			case mote::walking::RobotState::FallenBack:
				Internal_Motion_Request = Stand_Up_Back;
				break;
		}
		// vTaskDelay(1500);
		std::this_thread::sleep_for(std::chrono::milliseconds(1500));
		Check_Robot_Fall=1;
	}
}

//void Set_Head(double Pan, double Tilt, double Pan_Speed, double Tilt_Speed){
void mote::walking::Robot::setHead(double pan, double tilt, double panSpeed, double tiltSpeed)
{
	// Head_Pan_Angle = Pan;
	// Head_Tilt_Angle = Tilt;
	this->_humanoid.head.position.pan = pan;
	this->_humanoid.head.position.tilt = tilt;

	//check for head limitation

	// if(Head_Pan_Angle >  (Pi/2.0)) Head_Pan_Angle =  (Pi/2.0);
	// if(Head_Pan_Angle < -(Pi/2.0)) Head_Pan_Angle = -(Pi/2.0);

	// TODO: Put the head limits as configuration.
	if (this->_humanoid.head.position.pan > M_PI_2)
		this->_humanoid.head.position.pan = M_PI_2;
	if (this->_humanoid.head.position.pan < -M_PI_2)
		this->_humanoid.head.position.pan = -M_PI_2;

	//check for head tilt limitation
	// if(Head_Tilt_Angle >  ((Pi/2.0)-0.1)) Head_Tilt_Angle =  (Pi/2.0)-0.1;
	// if(Head_Tilt_Angle < -(Pi/3.0)) Head_Tilt_Angle = -(Pi/3.0);
	if (this->_humanoid.head.position.tilt > (M_PI_2 - 0.1))
		this->_humanoid.head.position.tilt = (M_PI_2 - 0.1);
	if (this->_humanoid.head.position.tilt < -(M_PI / 3.0))
		this->_humanoid.head.position.tilt = -M_PI / 3.0;

	this->_humanoid.head.velocity.pan = panSpeed;
	this->_humanoid.head.velocity.tilt = tiltSpeed;
}

mote::walking::Robot::Robot(mote::walking::Configuration &configuration, mote::walking::sensors::IMU &imu, HumanoidPart &humanoid)
	: configuration(configuration), imu(imu), ik(_humanoid, configuration, imu), _humanoid(humanoid)
{ }

void mote::walking::Robot::start()
{
	if (!this->_thread)
		this->_thread.reset(new std::thread(&mote::walking::Robot::run, this));
}

void mote::walking::Robot::stop()
{
	if (this->_thread)
	{
		this->_running = false;
		this->_thread->join();
		this->_thread.reset();
	}
}
