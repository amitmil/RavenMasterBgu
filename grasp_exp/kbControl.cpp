#include <stdio.h>
#include "timer.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <Windows.h>
#include <conio.h>
#include <list>
#include <vector>
#include <Eigen/Geometry>
#include "Eigen/Eigen"
#include "PracticalSocket\PracticalSocket.h"
#include "sigma_comm.h"
#include "ITPteleoperation.h"
#include "..\VS_GUI_SERVER\CommonDS.h"
#include "VisionThread3.h"
#include <iomanip> 
#include "kbControl.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;



extern v_struct ravenData;
extern int TimeToStop;
extern FrameRateCounter FPS;
extern Timer timer;
extern double gRampupRate1;
extern Matrix4d xfs;
extern int footpedal, prevpedal;



//double nullPose[DHD_MAX_DOF] = { 0.08, -0.03, 0.0,  // base  (translations)
//                                 0.0, 0.0, 0.0,  // wrist (rotations)
//                                 0.0 };          // gripper


extern const double PI = 3.1415926535897932384626433832795;

//extern bool ExpOn = false;
extern bool ServoFlag;
//extern float initReachPosX;
//extern float initReachPosY;

//extern bool HitFlag;
//extern float VelX;
//extern float VelY;
//extern float VelPreX;
//extern float VelPreY;

extern stMA2UI_DATA Ma2UIdata;
extern stUI2MA_DATA UI2Madata;

extern Sigma_Comm comm;
extern int g_footpedal;
extern bool SimulationOn;
extern unsigned int servo;


void* keyBoardLoop(void* pUserData)
{
	bool indexing= false;
	bool firstime=true;
	bool initialized = false;
	bool WriteToFile=false;
	int bttn=0;
	int rl2sui;

	Vector3d dPosition[2], ravenPosition[2];
	Vector3d dOrientation[2];
	bool firstraven=false;
	double dGripper[2],Gripper[2];
	dGripper[0] = 0;
	dGripper[1] = 0;
	Gripper[0] = 0;
	Gripper[1] = 0;
	Quaterniond qIncr[2];
	// start haptic simulation
	SimulationOn       = true;
	//	SimulationFinished = false;
	char a;
	bool kbFlag = false;
	while ( !TimeToStop && !UI2Madata.flagControlMode)
	{// 	ENABLE ME   && comm.Check_Flag(BASIC_PROGRAM) ) {
		dOrientation[0].setZero();		
		dOrientation[1].setZero();
		dPosition[0].setZero();
		dPosition[1].setZero();
		dGripper[0] = 0;
		dGripper[1] = 0;
		Quaterniond qIncr[2];
		qIncr[0].setIdentity();
		qIncr[1].setIdentity();

		servo++;
		//if(servo%100 == 1) cout<<servo<<endl;
		prevpedal= footpedal;
		footpedal = comm.Check_Flag(FPEDAL_RIGHT) ? TRUE : FALSE;
		Gripper[1] = 1000*ravenData.grasp[1];
		Gripper[0] = 1000*ravenData.grasp[1];
		bool ravenUpdate = false;
		bool gripChange = false;
		
		if( _kbhit() )	
		{
			if (!kbFlag)
			{
				kbFlag = true;
				a = _getch();
			}
			switch(a) {
			case X_P:    // key up
				dPosition[0].x() = STEP_CART;
				//cout <<  "x+" << endl;
				break;
			case X_N:    // key down
				dPosition[0].x() = -STEP_CART;
				//cout <<  "x-" << endl;
				break;
			case Y_P:    // key right
				dPosition[0].y() = STEP_CART;
				//cout <<  "y+" << endl;
				break;
			case Y_N:    // key left
				dPosition[0].y() = -STEP_CART;
				//cout <<  "y-" << endl;
				break;
			case Z_P:    // key right
				dPosition[0].z() = STEP_CART;
				//cout <<  "z+" << endl;
				break;
			case Z_N:    // key left
				dPosition[0].z() = (-STEP_CART);
				//cout <<  "z-" << endl;
				break;
			case GRIP_CLOSE:
				dGripper[0] = (-STEP_GRIP);
				//cout <<  "grip-" << endl;
				gripChange = true;
				break;
			case GRIP_OPEN:
				dGripper[0] = STEP_GRIP;
				//cout <<  dGripper[0] << endl;
				gripChange = true;
				break;
			case YAW_P:
				dOrientation[0].z() = STEP_ORI;
				//cout <<  "yaw+" << endl;
				break;
			case YAW_N:
				dOrientation[0].z() = -STEP_ORI;
				//cout <<  "yaw-" << endl;
				break;
			case ROLL_P:
				dOrientation[0].x()= STEP_ORI;
				//cout <<  "roll+" << endl;
				break;
			case ROLL_N:
				dOrientation[0].x() = -STEP_ORI;
				//cout <<  "roll-" << endl;
				break;
			case PITCH_P:
				dOrientation[0].y() = STEP_ORI;
				//cout <<  "pitch+" << endl;
				break;
			case PITCH_N:
				dOrientation[0].y() = -STEP_ORI;
				//cout <<  "pitch-" << endl;
				break;
			
		//}
		}
		}
		else
			kbFlag = false;
		//Roll pitch and yaw in Radians
		qIncr[0] = AngleAxisd(dOrientation[0].x(), Vector3d::UnitX()) // ROLL
			* AngleAxisd(dOrientation[0].y(),  Vector3d::UnitY()) // PITCH
			* AngleAxisd(dOrientation[0].z(), Vector3d::UnitZ()); // YAW
		dGripper[1] = dGripper[0];
		qIncr[1] = qIncr[0];
		dPosition[1] = dPosition[0];
		Gripper[1] = Gripper[1] + dGripper[0];
		Gripper[0] = Gripper[0] + dGripper[0];
		//if (gripChange)
			
		if( comm.Check_Flag(FPEDAL_RIGHT) && comm.Check_Flag(BASIC_START) )
		{
			if (prevpedal!=footpedal)
			{
				firstraven=false;
			}

			cout.setf(ios::fixed,ios::floatfield);
			cout.precision(3);
	//		cout << '\r' << "Grip Force: " <<std::setw(4)<< "hello" << flush;
			//if (dhdSetForceAndGripperForce (0.0,0.0,0.0,-(Gripper[i]-1000*ravenData.grasp[i])*0.1) < DHD_NO_ERROR) 
			//printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			///// END GRIPPER ALIGNMENT /////
		}

		rl2sui = 3-comm.Check_Flag(FPEDAL_RIGHT);
		int grip2[2];
		grip2[0]=(int)dGripper[0];
		grip2[1]=(int)dGripper[1];

		bool rcvudp;
		int fps = FPS.GetFrameRate();
		if(servo%1000 == 1) {
			comm.Update_MA2UI(dPosition, rl2sui, servo, 0, 0, 0, 0, 0,fps);			// Update SUI data
			comm.Send_TCP();
			//cout<<rl2sui;
		}
		if(comm.Check_Flag(BASIC_PROGRAM)) { // when UI allows 'Server' run
		//	if (ravenUpdate)
		//	{
		//		for (int i=0;i<20;i++)
		//		{
		//			

		//			comm.Update_UDP_Data(dPosition, qIncr, Gripper, grip2, footpedal, servo, &xfs );  // Update robot command packet
		//			comm.Send_UDP();
		//			rcvudp=comm.Recv_UDP();
		//			Sleep(1);
		//			servo++;
		//		}
		//	}
		//	else
		//	{
			
				comm.Update_UDP_Data(dPosition, qIncr, dGripper, grip2, footpedal, servo, &xfs );  // Update robot command packet
				comm.Send_UDP();
				rcvudp=comm.Recv_UDP();
		//	}
			if(prevpedal>footpedal && !firstime)
				firstime=true;
			if (!firstraven && firstime && rcvudp)
			{
				firstime=false;
				firstraven=true;
			}

		}
	}
	return NULL;
}
//void* logHaptics(void* pUserData2)
//{
//	//ofstream myfile;
//	//char file_name[50];
//	//sprintf(file_name, "example_%i.txt", trial_number);
//	//myfile.open(file_name);
//	////! simulation clock
//	//cPrecisionClock logClock;
//	//logClock.reset();
//	//logClock.start();
//	//while (status<2)
//	ofstream myfile;
//	double currTime;
//	while(!TimeToStop)
//	{
//		if(hapticData.newfile)
//		{
//			char file_name[50];
//			char file_path[50];
//			sprintf(file_name, "_trial_%i.txt", hapticData.filenumber);
//			//			strcpy(file_path, "logdata");
//			strcpy(file_path, "data\\");
//			strcat(file_path, file_name);
//			myfile.open(file_path);
//			myfile << "Time" << "\t" << "Pos.x" << "\t" << "Pos.y" << "\t" << "Pos.z" <<
//				"\t" << "Vel.x" << "\t" << "Vel.y" << "\t" << "Vel.z" <<  "\t" << "gripper" <<
//				"\t" << "raven.x" << "\t" << "raven.y" << "\t" << "raven.z" << "\t" 
//				"\t" << "raven.qx" << "\t" << "raven.qy" << "\t" << "raven.qz" <<
//				"\t" << "raven.qw" << "\t" << "ravenGripper" << endl;
//			hapticData.newfile = false;
//			hapticData.mark1minute += 60;
//			hapticData.filenumber += 1;
//		}
//		do
//		currTime = timer.get();
//		while (currTime - hapticData.prevTime < 0.001);
//		myfile << currTime << "\t" << hapticData.position[1].x() << "\t" << hapticData.position[1].y() << "\t" << hapticData.position[1].z() << 
//			"\t" << hapticData.velocity[1].x() << "\t" << hapticData.velocity[1].y() << "\t" << hapticData.velocity[1].z() <<
//			"\t" << hapticData.orientation[1].x() << "\t" << hapticData.orientation[1].y() << "\t" << hapticData.orientation[1].z() << "\t" <<
//			"\t" << hapticData.gripper[1] << "\t" << ravenData.px[1] << "\t" <<ravenData.py[1] << "\t" << ravenData.pz[1]<< 
//			"\t" << ravenData.Qx[1] << "\t" << ravenData.Qy[1] << "\t" << ravenData.Qz[1] <<
//			"\t" << ravenData.Qw[1] << "\t" << ravenData.grasp[1] << endl;
//		hapticData.prevTime = currTime;
//		if (currTime > hapticData.mark1minute)
//		{
//			hapticData.newfile = true;
//			myfile.close();
//		}
//		cout<< " " << hapticData.mark1minute << " " << currTime << " " << hapticData.filenumber <<endl;
//		/*}
//		myfile.close();*/
//	}
//	return NULL;
//}