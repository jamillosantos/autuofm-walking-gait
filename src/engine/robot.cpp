/**
 * @author J. Santos <jamillo@gmail.com>
 * @date September 20, 2016
 */

#include "robot.h"
#include "part.h"

void mote::walking::Robot::run()
{
	this->init();
	this->_velocity.zero();
	this->_velocityTheta = 0;
	// Buzzer(200);

	this->Stand_Init_T(0.05, 1);
	std::sleep(1);
	//stand for first time
	this->Stand_Init_T(0.05, 300);

	Check_Robot_Fall=1; // Check_Robot_Fall==1;??
	//main task loop
	for( ;; )
	{
		// togglePin(RED_LED_485EXP);

		if ((
				(
					(this->_velocity.x != 0)
					|| (this->_velocity.y != 0)
					|| (this->_velocityTheta != 0)
				)
				&& (Motion_Ins == No_Motion)
				&& (Internal_Motion_Request == No_Motion)
			)
			&& (System_Voltage >= (int)WEP[P_Min_Voltage_Limit])
		) {
			this->Stand_Init_T(0.5,50);

			//start gait
			WEP[P_Left_Leg_Y_Offset]=-50;
			WEP[P_Right_Leg_Y_Offset]=50;

			this->Stand_Init_T(1.0,4);  // last one was 2
			Set_Walk_Engine_Parameters((byte)Teen_Size_Robot_Num);
			this->Omni_Gait(WEP[Vx_Offset],WEP[Vy_Offset],WEP[Vt_Offset]); //execute omni-directional start gait

			//main gait
			while((
					(
						(this->_velocity.x != 0)
						|| (this->_velocity.y != 0)
						|| (this->_velocityTheta != 0)
					)
					&& (Motion_Ins == No_Motion)
					&& (Internal_Motion_Request == No_Motion)
				  )
				  && (System_Voltage >= (int)WEP[P_Min_Voltage_Limit])
			)
				this->Omni_Gait(Vx+WEP[Vx_Offset],Vy+WEP[Vy_Offset],Vt+WEP[Vt_Offset]); // execute omni-directional gait

			//finish gate
			this->Omni_Gait(WEP[Vx_Offset],WEP[Vy_Offset],WEP[Vt_Offset]); //execute omni-directional end gait
		}
		else
		{
			//firts run gait generation with vx=vy=vt=0 for semi gait generation to stand position
			if (Internal_Motion_Request == No_Motion)
				this->standInit(0.5);

			if (Internal_Motion_Request == Stop_Motion)
				this->standInit(0.05);

			vTaskDelay(20);

			//run standup motions
			switch(Internal_Motion_Request)
			{ //check for instrcation
				case Stand_Up_Front:
					//run stand up
					// vTaskDelay(1000);
					std::sleep(1);

					Check_Robot_Fall = 0;
					Actuators_Update = 1;
					this->Stand_Init_T(1.0, 10);
					//Internal_Motion_Request=No_Motion;
					Motion_Stand_Up_Front((byte)Teen_Size_Robot_Num);
					Internal_Motion_Request = No_Motion;
					this->Stand_Init_T(1.0, 10);
					Check_Robot_Fall = 1;
					break;

				case (byte)Stand_Up_Back:
					//run stand up
					// vTaskDelay(1000);
					std::sleep(1);

					Check_Robot_Fall = 0;
					Actuators_Update = 1;
					this->Stand_Init_T(1.0, 10);
					//Internal_Motion_Request=No_Motion;
					Motion_Stand_Up_Back((byte)Teen_Size_Robot_Num);
					Internal_Motion_Request = No_Motion;
					this->Stand_Init_T(1.0, 10);
					Check_Robot_Fall=1;
					break;
			}

			//run the motion
			switch(Motion_Ins)
			{  //check for instrcation
				case (byte)Motion_1:
					//run motion 1
					//initialize head joint
					Set_Head(0,0.5,500,500);
					this->Stand_Init_T(1.0, 25);
					Run_R_Kik_Motion((byte)Teen_Size_Robot_Num);
					this->Stand_Init_T(1.0, 70);
					Motion_Ins=No_Motion;
					break;

				case (byte)Motion_2:
					//run motion 2
					Set_Head(0,0.5,500,500);
					this->Stand_Init_T(1.0, 25);
					Run_L_Kik_Motion((byte)Teen_Size_Robot_Num);
					this->Stand_Init_T(1.0, 70);
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_3:
					this->Stand_Init_T(1.0, 10);
					Motion_Ins = No_Motion;
					break;

				case (byte)Motion_4:
					this->Stand_Init_T(1.0, 10);
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
		}//main if/else
	}
}

void mote::walking::Robot::init()
{
	this->standInit(0.1);  //ititialie robot to stand position

	this->_velocity.zero();
	this->_velocityTheta = 0;

	Motion_Ins = No_Motion;
	Internal_Motion_Request = No_Motion;
}

void mote::walking::Robot::standInit(double _Speed)
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
	leftArm.velocityPitch = leftArm.velocityRoll = leftArm.velocityElbow = _Speed;

	//left arm initialize
	rightArm.zero();
	rightArm.velocityPitch = rightArm.velocityRoll = rightArm.velocityElbow = _Speed;

	//update robotis joints
	// this->Update_Ik(_Speed, _Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
	this->Update_Ik(_Speed, _Speed, rightLeg, leftLeg, rightArm, leftArm);
}

//initialize robot to stand
void mote::walking::Robot::Stand_Init_T(double _Speed, int T)
{
		double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
		double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
		double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
		double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

		//update robotis joints
	for (unsigned int Cnt = 0; Cnt <= T; Cnt++)
	{
		//right leg initialize
		R_Leg_Ik[I_X] = 0.0;
		R_Leg_Ik[I_Y] = 0.0;
		R_Leg_Ik[I_Z] = 0.0;
		R_Leg_Ik[I_Roll] = 0.0;
		R_Leg_Ik[I_Pitch] = 0.0;
		R_Leg_Ik[I_Yaw] = 0.0;

		//left leh initialize
		L_Leg_Ik[I_X] = 0.0;
		L_Leg_Ik[I_Y] = 0.0;
		L_Leg_Ik[I_Z] = 0.0;
		L_Leg_Ik[I_Roll] = 0.0;
		L_Leg_Ik[I_Pitch] = 0.0;
		L_Leg_Ik[I_Yaw] = 0.0;

		//right arm initialize
		R_Arm[I_A_Pitch] = 0.0;
		R_Arm[I_A_Roll] = 0.0;
		R_Arm[I_A_Elbow] = 0.0;
		R_Arm[I_A_Vp] = _Speed;
		R_Arm[I_A_Vr] = _Speed;
		R_Arm[I_A_Ve] = _Speed;

		//left arm initialize
		L_Arm[I_A_Pitch] = 0.0;
		L_Arm[I_A_Roll] = 0.0;
		L_Arm[I_A_Elbow] = 0.0;
		L_Arm[I_A_Vp] = _Speed;
		L_Arm[I_A_Vr] = _Speed;
		L_Arm[I_A_Ve] = _Speed;

		this->Update_Ik(_Speed, _Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
		std::this_thread::sleep_for(std::chrono::duration<std::chrono::milliseconds>(20));
		//vTaskDelay(20);
	}
}

//omni directional gaite generation
void mote::walking::Robot::Omni_Gait(double vx, double vy, double vt)
{
	double L_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	double R_Leg_Ik[6];  // x, y, z, roll, pitch, yaw
	double L_Arm[6];     // pitch, roll, elbow, vp, vr, ve
	double R_Arm[6];     // pitch, roll, elbow, vp, vr, ve

	double Joint_Speed = 0.3;

	//gait generate with for loop form  0~3.14
	for (double t = 0; t <= TwoPi ; t += WEP[P_Motion_Resolution])
	{
		if (vx >  0.3)
			vx =  0.3;
		if (vx < -0.3)
			vx = -0.3;

		if (vy >  0.4)
			vy =  0.4;
		if (vy < -0.4)
			vy = -0.4;

		if (vt >  0.05)
			vt =  0.05;
		if (vt < -0.05)
			vt = -0.05;

		//-----------------------
		if ((
				(t >= Pi-WEP[P_Motion_Resolution])
				&& (t <= Pi+WEP[P_Motion_Resolution])
			)
			|| (
				(t >= TwoPi-WEP[P_Motion_Resolution])
				&&(t <= TwoPi+WEP[P_Motion_Resolution])
			)
			|| (t == 0.0)
		)
			vTaskDelay(WEP[P_Double_Support_Sleep]);

		if (
			(
				(t >= (Pi/2.0) - WEP[P_Motion_Resolution])
				&& (t <= (Pi/2.0)+WEP[P_Motion_Resolution])
			)
			|| (
				(t >= ((3.0*Pi)/2.0)-WEP[P_Motion_Resolution])
				&& (t <= ((3.0*Pi)/2.0)+WEP[P_Motion_Resolution])
			)
		)
			vTaskDelay(WEP[P_Single_Support_Sleep]);

		//right leg initialize
		//if t<pi the right leg in fly state and other wise in support state
		int S_X=1;
		if (vx < 0.0)
			S_X=0.2;

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

		R_Leg_Ik[I_X]     = (-(cos(t)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * S_X * 100.0);
		R_Leg_Ik[I_Y]     = (t<=Pi) ? (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) + (cos(t-Pi)*(vy*50.0))
									: (sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
		R_Leg_Ik[I_Z]     = (t<=Pi) ? (sin(t)*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : (sin(t)*(WEP[P_Support_Z_Swing_Gain]*100.0));
		R_Leg_Ik[I_Roll]  = (t<=Pi) ? (-sin(t)*(WEP[P_Fly_Roll_Gain])) : (-sin(t)*(WEP[P_Support_Roll_Gain]));
		R_Leg_Ik[I_Pitch] = (t<=Pi) ? 0.0 : (sin(t)*WEP[P_Support_Pitch_Gain]);
		R_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt)); //(cos(t-Pi)*(vt));

		//left leh initialize
		L_Leg_Ik[I_X]     = (-(cos(t-Pi)*(vx*100.0)))+ (vx * WEP[P_Body_X_Swing_Gain] * S_X * 100.0);
		L_Leg_Ik[I_Y]     = (t>=Pi) ? (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0)) + (-sin(t)*(WEP[P_Fly_Y_Swing_Gain]*100.0)) +(cos(t-Pi)*(vy*50.0))
									: (-sin(t)*(WEP[P_Body_Y_Swing_Gain]*100.0))+(cos(t-Pi)*(vy*50.0))+(sin(t)*WEP[P_Support_Y_Swing_Gain]);
		L_Leg_Ik[I_Z]     = (t>=Pi) ? ((sin(t-Pi))*(WEP[P_Fly_Z_Swing_Gain]*100.0)) : ((sin(t-Pi))*(WEP[P_Support_Z_Swing_Gain]*100.0));
		L_Leg_Ik[I_Roll]  = (t>=Pi) ? (sin(t)*(WEP[P_Fly_Roll_Gain])) : (sin(t)*(WEP[P_Support_Roll_Gain]));
		L_Leg_Ik[I_Pitch] = (t>=Pi) ? 0.0 : (sin(t-Pi)*WEP[P_Support_Pitch_Gain]);
		L_Leg_Ik[I_Yaw]   = (cos(t-Pi)*(vt)); //(cos(t-Pi)*(vt));

		//right arm initialize
		R_Arm[I_A_Pitch]   = ((cos(t)) * vx * 1.995); // + Arm_Hopping_Val; // + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		R_Arm[I_A_Roll]    = 0.0;
		R_Arm[I_A_Elbow]   = 0.0;
		R_Arm[I_A_Vp]      = 0.2;
		R_Arm[I_A_Vr]      = 0.05;
		R_Arm[I_A_Ve]      = 0.05;

		//left arm initialize
		L_Arm[I_A_Pitch]   = ((cos(t-Pi)) * vx * 1.995); // + Arm_Hopping_Val;// + ((MPU_X+Get_E_Param(Addr_IMU_X_Angle_Offset)) * Get_E_Param(Addr_Stablizer_Arm_Pitch_Gain)*2);
		L_Arm[I_A_Roll]    = 0.0;
		L_Arm[I_A_Elbow]   = 0.0;
		L_Arm[I_A_Vp]      = 0.2;
		L_Arm[I_A_Vr]      = 0.05;
		L_Arm[I_A_Ve]      = 0.05;

		//update robotis joints
		Update_Ik(Joint_Speed, Joint_Speed, R_Leg_Ik, L_Leg_Ik, R_Arm, L_Arm);
		vTaskDelay(WEP[P_Gait_Frequency]*100);
	}//main gait timi for ins
}

//get robot state for fall detection
byte Get_Robot_State(double Roll, double Pitch){
	if(Roll>=WEP[P_Fall_Roll_Thershold]){
		return Fallen_Left;
	}

	if(Roll<=(-WEP[P_Fall_Roll_Thershold])){
		return Fallen_Right;
	}

	if(Pitch>=WEP[P_Fall_Pitch_Thershold]){
		return Fallen_Front;
	}

	if(Pitch<=(-WEP[P_Fall_Pitch_Thershold])){
		return Fallen_Back;
	}
	return Normal_Stand;
}

//check robot state and run the stand function
void Robot_State(){
	switch (Get_Robot_State(MPU_Y, MPU_X)){
		case (byte)Fallen_Front:{
			//run stand motion from front
			Check_Robot_Fall=0;

			vTaskSuspendAll();

			Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
			Dxl.writeByte(Id_Head_Pan,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Head_Tilt,P_TORQUE_ENABLE,1);

			Set_Head(0,-0.4,1023,1023);
			Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
			Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);

			Dxl.writeByte(Id_Right_Hip_Yaw,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Yaw,P_TORQUE_ENABLE,1);

			Dxl.writeByte(Id_Right_Hip_Roll,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Roll,P_TORQUE_ENABLE,1);

			xTaskResumeAll();

			Internal_Motion_Request=Stand_Up_Front;
			//Buzzer(200);
			vTaskDelay(1500);
			Check_Robot_Fall=1;
		}
			break;
		case (byte)Fallen_Back:{
			//run stand motion from front
			Check_Robot_Fall=0;

			vTaskSuspendAll();
			Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
			Dxl.writeByte(Id_Head_Pan,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Head_Tilt,P_TORQUE_ENABLE,1);

			Set_Head(0,0.4,1023,1023);
			Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
			Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);

			Dxl.writeByte(Id_Right_Hip_Yaw,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Yaw,P_TORQUE_ENABLE,1);

			Dxl.writeByte(Id_Right_Hip_Roll,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Roll,P_TORQUE_ENABLE,1);

			xTaskResumeAll();

			Internal_Motion_Request=Stand_Up_Back;
			vTaskDelay(1500);
			Check_Robot_Fall=1;
			//Buzzer(200);
		}
			break;
		case (byte)Fallen_Right:{
			//run stand motion from front
			Check_Robot_Fall=0;
			vTaskSuspendAll();
			Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
			Dxl.writeByte(Id_Head_Pan,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Head_Tilt,P_TORQUE_ENABLE,1);

			Set_Head(0,-0.4,1023,1023);
			Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
			Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);

			Dxl.writeByte(Id_Right_Hip_Yaw,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Yaw,P_TORQUE_ENABLE,1);

			Dxl.writeByte(Id_Right_Hip_Roll,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Roll,P_TORQUE_ENABLE,1);
			xTaskResumeAll();

			Internal_Motion_Request=Stand_Up_Front;
			//Buzzer(200);
			vTaskDelay(1500);
			Check_Robot_Fall=1;
		}
			break;
		case (byte)Fallen_Left:{
			//run stand motion from front
			Check_Robot_Fall=0;
			vTaskSuspendAll();
			Dxl.writeByte(BROADCAST_ID,P_TORQUE_ENABLE,0);
			Dxl.writeByte(Id_Head_Pan,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Head_Tilt,P_TORQUE_ENABLE,1);

			Set_Head(0,0.4,1023,1023);
			Dxl.setPosition(Id_Head_Pan,2048+(((int)((Head_Pan_Angle*RAD2DEG)*DEG2DXL))*1),(int)Head_Pan_Speed);
			Dxl.setPosition(Id_Head_Tilt,2048+(((int)(((Head_Tilt_Angle-MPU_X)*RAD2DEG)*DEG2DXL))*1),(int)Head_Tilt_Speed);

			Dxl.writeByte(Id_Right_Hip_Yaw,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Yaw,P_TORQUE_ENABLE,1);

			Dxl.writeByte(Id_Right_Hip_Roll,P_TORQUE_ENABLE,1);
			Dxl.writeByte(Id_Left_Hip_Roll,P_TORQUE_ENABLE,1);
			xTaskResumeAll();

			Internal_Motion_Request=Stand_Up_Front;
			//Buzzer(200);
			vTaskDelay(1500);
			Check_Robot_Fall=1;
		}
			break;
		case (byte)Normal_Stand:
			//Internal_Motion_Request
			break;
	}//switch case
}
