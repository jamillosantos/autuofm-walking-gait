/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#include "robot.h"

void mote::walking::Robot::run()
{
	this->_running = true;

	this->init();
	std::this_thread::sleep_for(std::chrono::seconds(1));

	this->Check_Robot_Fall = 1; // Check_Robot_Fall==1;??

	const configuration::Walking conf = this->configuration.walking;

	// double vx = Vx;
	double vx = this->velocity.x;
	// double vy = Vy;
	double vy = this->velocity.y;
	// double vt = Vt;
	double vt = this->velocity.theta;

	VERBOSE("Starting main loop!");
	//main task loop
	// while (true)
	while (this->_running)
	{
		VERBOSE("");
		// if (((Vx != 0) || (Vy != 0) || (Vt != 0)) && (Motion_Ins == 0))
		if (((this->velocity.x != 0) || (this->velocity.y != 0) || (this->velocity.theta != 0)) && (this->Motion_Ins == No_Motion))
		{
			// Stand_Init_T(50);
			this->standInitT(50);

			//start gait
			// Omni_Gait(WEP[Vx_Offset], WEP[Vy_Offset], WEP[Vt_Offset]); //execute omni-directional start gait
			this->omniGait(conf.velocityOffset.x, conf.velocityOffset.y, conf.velocityOffset.theta); //execute omni-directional start gait

			//main gait
			// while (((Vx != 0) || (Vy != 0) || (Vt != 0)) && (Motion_Ins == 0))
			while (((this->velocity.x != 0) || (this->velocity.y != 0) || (this->velocity.theta != 0)) &&
				   (this->Motion_Ins == No_Motion))
			{
				//increase the speed
				// if (Vx > vx) vx += 0.05;
				if (this->velocity.x > vx) vx += 0.05;
				// if (Vx < vx) vx -= 0.05;
				if (this->velocity.x < vx) vx -= 0.05;

				// if (Vy > vy) vy += 0.05;
				if (this->velocity.y > vy) vy += 0.05;
				// if (Vy < vy) vy -= 0.05;
				if (this->velocity.y < vy) vy -= 0.05;

				// if (Vt > vt) vt += 0.05;
				if (this->velocity.theta > vt) vt += 0.05;
				// if (Vt < vt) vt -= 0.05;
				if (this->velocity.theta < vt) vt -= 0.05;


				//check the max and min
				if (vx > 1.0) vx = 1.0;
				if (vx < -1.0) vx = -1.0;

				if (vy > 1.0) vy = 1.0;
				if (vy < -1.0) vy = -1.0;

				if (vt > 1.0) vt = 1.0;
				if (vt < -1.0) vt = -1.0;

				// Omni_Gait(vx + WEP[Vx_Offset], vy + WEP[Vy_Offset], Vt + WEP[Vt_Offset]); //execute omni-directional gait
				this->omniGait(vx + conf.velocityOffset.x, vy + conf.velocityOffset.y, vt + conf.velocityOffset.theta); //execute omni-directional gait
			}
		}
		else
		{
			this->standInitT(50);
			/*
			// TODO: insert motion run here!!!!!!!
			if(Motion_Ins==1)
			{
				Run_Static_Motion_1(1000);
				Stand_Init_T(50);
			}
			*/
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

void mote::walking::Robot::init()
{
	this->velocity.zero();
	this->velocity.theta = 0;
	// this->velocity.x = 0.1;

	this->Motion_Ins = No_Motion;
	this->Internal_Motion_Request = No_Motion;
}

//initialize robot to stand
void mote::walking::Robot::standInitT(double speed)
{
	this->_humanoid.rightHip.velocity.yaw = speed;
	this->_humanoid.rightHip.velocity.roll = speed;
	this->_humanoid.rightHip.velocity.pitch = speed;

	this->_humanoid.rightKnee.velocity = speed;

	this->_humanoid.rightFoot.velocity.pitch = speed;
	this->_humanoid.rightFoot.velocity.roll = speed;

	this->_humanoid.leftHip.velocity.yaw = speed;
	this->_humanoid.leftHip.velocity.roll = speed;
	this->_humanoid.leftHip.velocity.pitch = speed;

	this->_humanoid.leftKnee.velocity = speed;

	this->_humanoid.leftFoot.velocity.pitch = speed;
	this->_humanoid.leftFoot.velocity.roll = speed;

	this->_humanoid.rightArm.velocityElbow = speed;
	this->_humanoid.rightArm.velocity.roll = speed;
	this->_humanoid.rightArm.velocity.pitch = speed;

	this->_humanoid.leftArm.velocityElbow = speed;
	this->_humanoid.leftArm.velocity.roll = speed;
	this->_humanoid.leftArm.velocity.pitch = speed;

	this->_humanoid.rightLeg.position.zero();
	this->_humanoid.rightLeg.angle.zero();
	this->_humanoid.leftLeg.position.zero();
	this->_humanoid.leftLeg.angle.zero();

	this->_humanoid.rightArm.position.zero();
	this->_humanoid.leftArm.position.zero();

	// this->_humanoid.rightLeg.position.x = 110;
	// this->_humanoid.rightLeg.position.y = 110;
	// this->_humanoid.rightLeg.position.z = 110;

	this->ik.update();
}

//omni directional gaite generation
void mote::walking::Robot::omniGait(double vx, double vy, double vt)
{
	VERBOSE("Begin cycle");
	// for (double t = 0; t <= (Math.PI * 2); t += WEP[P_Motion_Resolution])
	for (double t = 0; t <= M_2xPIl; t += this->configuration.walking.engine.motionResolution)
	{
		// Walk_Thr_Cnt++;
		this->walkThreadCount++;

		// if (((t >= Math.PI - WEP[P_Motion_Resolution]) && (t <= Math.PI + WEP[P_Motion_Resolution])) || ((t >= (Math.PI * 2) - WEP[P_Motion_Resolution]) && (t <= (Math.PI * 2) + WEP[P_Motion_Resolution])) || (t == 0.0))
		if (
			(
				(t >= M_PIl - this->configuration.walking.engine.motionResolution)
				&& (t <= M_PIl + this->configuration.walking.engine.motionResolution)
			)
			||
			(
				(t >= (M_2xPIl) - this->configuration.walking.engine.motionResolution)
				&& (t <= (M_2xPIl) + this->configuration.walking.engine.motionResolution)
			)
			|| (t == 0.0)
		)
		{
			// for (long i = 0; i <= ((long)(WEP[P_Double_Support_Sleep] * 1000)); i++)
			for (long i = 0; i <= ((long) (this->configuration.walking.engine.doubleSupportSleep * 1000)); i++)
			{
				// Update_Trajectory(vx, vy, vt, t);
				this->updateTrajectory(vx, vy, vt, t);
				// Update_Ik();
				this->ik.update();
				// NOP(100)
				this->nop(100);
			}
		}

		// if (((t >= (Math.PI / 2.0) - WEP[P_Motion_Resolution]) && (t <= (Math.PI / 2.0) + WEP[P_Motion_Resolution])) || ((t >= ((3.0 * Math.PI) / 2.0) - WEP[P_Motion_Resolution]) && (t <= ((3.0 * Math.PI) / 2.0) + WEP[P_Motion_Resolution])))
		if (
			(
				(t >= (M_PI_2l) - this->configuration.walking.engine.motionResolution)
				&& (t <= (M_PI_2l) + this->configuration.walking.engine.motionResolution)
			)
			||
			(
				(t >= ((3.0 * M_PIl) / 2.0) - this->configuration.walking.engine.motionResolution)
				&& (t <= ((3.0 * M_PIl) / 2.0) + this->configuration.walking.engine.motionResolution)
			)
		)
		{
			// for (long i = 0; i <= ((long)(WEP[P_Single_Support_Sleep] * 1000)); i++)
			for (long i = 0; i <= ((long) (this->configuration.walking.engine.singleSupportSleep * 1000)); i++)
			{
				// Update_Trajectory(vx, vy, vt, t);
				this->updateTrajectory(vx, vy, vt, t);
				// Update_Ik();
				this->ik.update();
				// NOP(100);
				this->nop(100);
			}
		}

		// Update_Trajectory(vx, vy, vt, t);
		this->updateTrajectory(vx, vy, vt, t);

		// for (long i = 0; i <= ((long)(WEP[P_Gait_Frequency] * 1000)); i++)
		for (long i = 0; i <= ((long) (this->configuration.walking.engine.gaitFrequency * 1000)); i++)
		{
			// Update_Trajectory(vx,vy,vt,t);
			this->updateTrajectory(vx, vy, vt, t);
			// Update_Ik();
			this->ik.update();
			// NOP(100);
			this->nop(100);
		}
	} //omni for 0 to Pi
	VERBOSE("End cycle");
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

mote::walking::Robot::Robot(mote::walking::Configuration &configuration, mote::walking::sensors::IMU &imu,
	mote::walking::HumanoidPart &humanoidPublished)
	: configuration(configuration), imu(imu), ik(_humanoid, humanoidPublished, configuration, imu),
	  _publicHumanoidPart(humanoidPublished)
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

void mote::walking::Robot::updateTrajectory(double vx, double vy, double vt, double t)
{
	/*
	lock (IK_Spd)
	{
	 */
	//set speed
	// IK_Spd[R_L_Hip_Yaw] = 500 / 2;
	this->_humanoid.rightHip.velocity.yaw = 500.0 / 2.0;
	// IK_Spd[R_L_Hip_Roll] = 1000 / 2;
	this->_humanoid.rightHip.velocity.roll = 1000.0 / 2.0;
	// IK_Spd[R_L_Hip_Pitch] = 500 / 2;
	this->_humanoid.rightHip.velocity.pitch = 500.0 / 2.0;
	// IK_Spd[R_L_Knee] = 1000 / 2;
	this->_humanoid.rightKnee.velocity = 1000.0 / 2.0;
	// IK_Spd[R_L_Foot_Pitch] = 500 / 2;
	this->_humanoid.rightFoot.velocity.pitch = 500.0 / 2.0;
	// IK_Spd[R_L_Foot_Roll] = 1000 / 2;
	this->_humanoid.rightFoot.velocity.roll = 1000.0 / 2.0;

	// IK_Spd[L_L_Hip_Yaw] = 500 / 2;
	this->_humanoid.leftHip.velocity.yaw = 500.0 / 2.0;
	// IK_Spd[L_L_Hip_Roll] = 1000 / 2;
	this->_humanoid.leftHip.velocity.roll = 1000.0 / 2.0;
	// IK_Spd[L_L_Hip_Pitch] = 500 / 2;
	this->_humanoid.leftHip.velocity.pitch = 500.0 / 2.0;
	// IK_Spd[L_L_Knee] = 1000 / 2;
	this->_humanoid.leftKnee.velocity = 1000.0 / 2.0;
	// IK_Spd[L_L_Foot_Pitch] = 500 / 2;
	this->_humanoid.leftFoot.velocity.pitch = 500.0 / 2.0;
	// IK_Spd[L_L_Foot_Roll] = 1000 / 2;
	this->_humanoid.leftFoot.velocity.roll = 1000.0 / 2.0;

	// IK_Spd[R_A_Elbow] = 1000;
	this->_humanoid.rightArm.velocityElbow = 1000;
	// IK_Spd[R_A_Roll] = 500;
	this->_humanoid.rightArm.velocity.roll = 500;
	// IK_Spd[R_A_Pitch] = 1000;
	this->_humanoid.rightArm.velocity.pitch = 1000;

	// IK_Spd[L_A_Elbow] = 1000;
	this->_humanoid.leftArm.velocityElbow = 1000;
	// IK_Spd[L_A_Roll] = 500;
	this->_humanoid.leftArm.velocity.roll = 500;
	// IK_Spd[L_A_Pitch] = 1000;
	this->_humanoid.leftArm.velocity.pitch = 1000;
	// }

	// lock (IK)
	// {
	double Y_Time_Shift = 10000; //for shift the Y time
	double Roll_Time_Shift = 10000;

	// IK[R_L_X_Position] = (-(Math.Cos(t) * (vx * 500.0))) + (vx * WEP[P_Body_X_Swing_Gain] * 500.0);
	this->_humanoid.rightLeg.position.x = (-(std::cos(t) * (vx * 500.0))) + (vx * this->configuration.walking.engine.bodySwingGain.x * 500.0);
	// IK[R_L_Y_Position] = (t <= Math.PI) ? (Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Body_Y_Swing_Gain] * 200.0)) + (Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Fly_Y_Swing_Gain] * 100.0)) + (Math.Cos(t - Math.PI) * (vy * 150.0))
	// 									: (Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Body_Y_Swing_Gain] * 200.0)) + (Math.Cos(t - Math.PI) * (vy * 50.0)) + (Math.Sin(t + (Math.PI / Y_Time_Shift)) * WEP[P_Support_Y_Swing_Gain]);
	this->_humanoid.rightLeg.position.y =
		(t <= M_PIl) ?
			(std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.bodySwingGain.y * 200.0)) + (std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.flySwingGain.y * 100.0)) + (std::cos(t - M_PIl) * (vy * 150.0))
			:
			(std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.bodySwingGain.y * 200.0)) + (std::cos(t - M_PIl) * (vy * 50.0)) + (std::sin(t + (M_PIl / Y_Time_Shift)) * this->configuration.walking.engine.supportSwingGain.y);

	//IK[R_L_Z_Position] = Math.Exp(-Math.Pow((t - 1.6) / 0.4, 2)) * (WEP[P_Fly_Z_Swing_Gain] * 200.0);
	this->_humanoid.rightLeg.position.z = std::exp(-std::pow((t - 1.6) / 0.4, 2)) * (this->configuration.walking.engine.flySwingGain.z * 200.0);
	// IK[R_L_R_Angle] = (t <= Math.PI) ? (-Math.Sin(t) * (WEP[P_Fly_Roll_Gain])) : (-Math.Sin(t + (Math.PI / Roll_Time_Shift)) * (WEP[P_Support_Roll_Gain]));
	this->_humanoid.rightLeg.angle.roll = (t <= M_PIl) ? (-std::sin(t) * (this->configuration.walking.engine.flyGain.roll)) : (-std::sin(t + (M_PIl / Roll_Time_Shift)) * (this->configuration.walking.engine.supportGain.roll));
	// IK[R_L_P_Angle] = (t <= Math.PI) ? 0.0 : (Math.Sin(t) * WEP[P_Support_Pitch_Gain]);
	this->_humanoid.rightLeg.angle.pitch = (t <= M_PIl) ? 0.0 : (std::sin(t) * this->configuration.walking.engine.supportGain.pitch);
	// IK[R_L_Y_Angle] = (Math.Cos(t - Math.PI) * (vt));
	this->_humanoid.rightLeg.angle.yaw = (std::cos(t - M_PIl) * (vt));


	// IK[L_L_X_Position] = (-(Math.Cos(t - Math.PI) * (vx * 500.0))) + (vx * WEP[P_Body_X_Swing_Gain] * 500.0);
	this->_humanoid.leftLeg.position.x = (-(std::cos(t - M_PIl) * (vx * 500.0))) + (vx * this->configuration.walking.engine.bodySwingGain.x * 500.0);
	// IK[L_L_Y_Position] = (t >= Math.PI) ? (-Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Body_Y_Swing_Gain] * 200.0)) + (-Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Fly_Y_Swing_Gain] * 100.0)) + (Math.Cos(t - Math.PI) * (vy * 150.0))
	// 									: (-Math.Sin(t + (Math.PI / Y_Time_Shift)) * (WEP[P_Body_Y_Swing_Gain] * 200.0)) + (Math.Cos(t - Math.PI) * (vy * 50.0)) + (Math.Sin(t + (Math.PI / Y_Time_Shift)) * WEP[P_Support_Y_Swing_Gain]);
	this->_humanoid.leftLeg.position.y =
		(t >= M_PIl) ?
			(-std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.bodySwingGain.y * 200.0)) + (-std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.flySwingGain.y * 100.0)) + (std::cos(t - M_PIl) * (vy * 150.0))
			:
			(-std::sin(t + (M_PIl / Y_Time_Shift)) * (this->configuration.walking.engine.bodySwingGain.y * 200.0)) + (std::cos(t - M_PIl) * (vy * 50.0)) + (std::sin(t + (M_PIl / Y_Time_Shift)) * this->configuration.walking.engine.supportSwingGain.y);
	// IK[L_L_Z_Position] = Math.Exp(-Math.Pow((t - (1.6 + Math.PI)) / 0.4, 2)) * (WEP[P_Fly_Z_Swing_Gain] * 200.0);
	this->_humanoid.leftLeg.position.z = std::exp(-std::pow((t - (1.6 + M_PIl)) / 0.4, 2)) * (this->configuration.walking.engine.flySwingGain.z * 200.0);
	// IK[L_L_R_Angle] = (t >= Math.PI) ? (Math.Sin(t) * (WEP[P_Fly_Roll_Gain])) : (Math.Sin(t + (Math.PI / Roll_Time_Shift)) * (WEP[P_Support_Roll_Gain]));
	this->_humanoid.leftLeg.angle.roll =
		(t >= M_PIl) ?
			(std::sin(t) * (this->configuration.walking.engine.flyGain.roll))
			:
			(std::sin(t + (M_PIl / Roll_Time_Shift)) * (this->configuration.walking.engine.supportGain.roll));
	// IK[L_L_P_Angle] = (t >= Math.PI) ? 0.0 : (Math.Sin(t - Math.PI) * WEP[P_Support_Pitch_Gain]);
	this->_humanoid.leftLeg.angle.pitch = (t >= M_PIl) ? 0.0 : (std::sin(t - M_PIl) * this->configuration.walking.engine.supportGain.pitch);
	// IK[L_L_Y_Angle] = (Math.Cos(t - Math.PI) * (vt));
	this->_humanoid.leftLeg.angle.yaw = (std::cos(t - M_PIl) * (vt));

	//calculate odometery
	// Odometer_X = (short)(Math.Abs(IK[R_L_X_Position] - IK[L_L_X_Position])*vx);
	this->odometer.x = (short)(std::abs(this->_humanoid.rightLeg.position.x - this->_humanoid.leftLeg.position.x)*vx);
	// Odometer_Y = (short)(Math.Abs(IK[R_L_Y_Position] + IK[L_L_Y_Position])*vy);
	this->odometer.y = (short)(std::abs(this->_humanoid.rightLeg.position.y + this->_humanoid.leftLeg.position.y)*vy);
	// Odometer_Z = (short)(Math.Abs(IK[R_L_Y_Angle]*314 + IK[L_L_Y_Angle]*314)*vt);
	this->odometer.z = (short)(std::abs(this->_humanoid.rightLeg.angle.yaw*314 + this->_humanoid.leftLeg.angle.yaw*314)*vt);

	//create hand motion
	// IK[R_H_X_Position] = 0;
	this->_humanoid.rightArm.position.x = 0;
	// IK[R_H_Y_Position] = 0;
	this->_humanoid.rightArm.position.y = 0;
	// IK[R_H_Z_Position] = ((Math.Cos(t - Math.PI)) * vx * 1.65);
	this->_humanoid.rightArm.position.z = ((std::cos(t - M_PIl)) * vx * this->configuration.walking.armSwingLength);

	// IK[L_H_X_Position] = 0;
	this->_humanoid.leftArm.position.x = 0;
	// IK[L_H_Y_Position] = 0;
	this->_humanoid.leftArm.position.y = 0;
	// IK[L_H_Z_Position] = ((Math.Cos(t)) * vx * 1.65);
	this->_humanoid.leftArm.position.z = ((std::cos(t)) * vx * this->configuration.walking.armSwingLength);

	// } //lock
}

void mote::walking::Robot::nop(int times)
{
	// TODO: Solve this problem.
	std::this_thread::sleep_for(std::chrono::milliseconds(times/100));
}
