#include <stdio.h>
#include "timer.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <windows.h>
#include <list>
#include <vector>
#include "Haptic.h"
#include "dhdc.h"
#include "drdc.h"
#include <Eigen/Geometry>
#include "Eigen/Eigen"
#include "PracticalSocket\PracticalSocket.h"
#include "sigma_comm.h"
#include "ITPteleoperation.h"
#include "..\VS_GUI_SERVER\CommonDS.h"
#include "VisionThread3.h"
#include <iomanip> 
#include "fdControl.h"

using namespace std;
using namespace Eigen;

extern v_struct ravenData;
extern int TimeToStop;
extern FrameRateCounter FPS;
extern Timer timer;


//double nullPose[DHD_MAX_DOF] = { 0.08, -0.03, 0.0,  // base  (translations)
//                                 0.0, 0.0, 0.0,  // wrist (rotations)
//                                 0.0 };          // gripper

const double PI = 3.1415926535897932384626433832795;

//bool WriteToFile=false;
//double CurrentTime;

//bool ExpOn = false;
//extern bool ServoFlag;
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

extern int footpedal, prevpedal;

void* fdControlLoop(void* pUserData)
{
	bool indexing= false;
	bool firstime=true;
	bool initialized = false;
	bool WriteToFile=false;
	int bttn=0;
	int rl2sui;
	int devHandle[2];
	InitHapticDevice(devHandle);
	static double t0 = dhdGetTime ();
	double        t  = t0 + 0.001;
	double        dt,grasp=0;
	double newGrip=0;


	double rotMat_temp[3][3];
	Matrix3d rotMat;
	Vector3d Position[2], oldPosition[2], dPosition[2], ravenPosition[2],ravendPosition[2],deltaPosition[2],forcePosition[2];
	bool firstraven=false;
	Vector3d Velocity[2];
	Vector3d Orientation[2];
	double Gripper[3], oldGripper[3], dGripper[3];
	double Vgrip[3];
	//static Vector3d oldPosistion[2];
	static Vector3d oldVelocity[2];
	Quaterniond qIncr[2],qCurr[2],qPrev[2];

	Vector3d radius[2];
	radius[0]<< 0.0, -0.02, 0.0;
	//int servo=0;
	int K=50;
	double d[3]={0,0,0};
	// start haptic simulation
	SimulationOn       = true;
	//	SimulationFinished = false;
	dhdEnableForce (DHD_ON, devHandle[0]);
	dhdEnableForce (DHD_ON, devHandle[1]);

	while(!TimeToStop && !UI2Madata.flagControlMode)
	{
		servo++;
		//if(servo%100 == 1) cout<<servo<<endl;
		prevpedal= footpedal;
		footpedal = comm.Check_Flag(FPEDAL_RIGHT) ? TRUE : FALSE;
		t  = dhdGetTime ();
		dt = t - t0;
		t0 = t;
		for(int i=0;i<2;i++)
		{
			oldPosition[i]=Position[i];
			oldGripper[i]=Gripper[i];
			qPrev[i] = qCurr[i];
			dhdSetDevice(devHandle[0]); //AMIT - CHANGE THIS WHEN TWO DEVICES!!!
			// POSITION AND VELOCITY OF END EFFECTOR
			dhdGetPosition(&Position[i].x(),&Position[i].y(),&Position[i].z());
			dhdGetGripperAngleRad(&Gripper[i]);
	
			// ORIENTATION
			dhdGetOrientationFrame(rotMat_temp);
			dhdGetOrientationRad(&Orientation[i].x(),&Orientation[i].y(),&Orientation[i].z());
				qCurr[0] = AngleAxisd(-Orientation[0].x(), Vector3d::UnitX()) // ROLL
			* AngleAxisd(Orientation[0].y(),  Vector3d::UnitY()) // PITCH
			* AngleAxisd(-Orientation[0].z(), Vector3d::UnitZ()); // YAW
			
				
			//	rotMat << rotMat_temp[0][0], rotMat_temp[0][1], rotMat_temp[0][2] ,
			//			rotMat_temp[1][0], rotMat_temp[1][1], rotMat_temp[1][2] ,
			//			rotMat_temp[2][0], rotMat_temp[2][1], rotMat_temp[2][2];
			//dhdGetOrientationFrame(rotMat_temp);
			//rotMat << rotMat_temp[0][0], rotMat_temp[0][1], rotMat_temp[0][2] ,
			//			rotMat_temp[1][0], rotMat_temp[1][1], rotMat_temp[1][2] ,
			//			rotMat_temp[2][0], rotMat_temp[2][1], rotMat_temp[2][2];
			//rotMat.transpose(); //does it need transpose??
			//qCurr[i] = Quaterniond(rotMat);
			if(dhdSetForceAndGripperForce(0.0,0.0,0.0,0.0) < DHD_NO_ERROR) 
				printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			

	//		qIncr[i]=qPrev[i].inverse()*qCurr[i];
			if( comm.Check_Flag(FPEDAL_RIGHT) && comm.Check_Flag(BASIC_START) )
			{
				if (prevpedal!=footpedal)
				{
					firstraven=false;
				}
				
				dPosition[i] = (double)UI2Madata.scale_pos/100.0*(Position[i] - oldPosition[i]); // incremental
				//qIncr[i] = qPrev[i].inverse()*qCurr[i];		
				qIncr[i] = qCurr[i]*qPrev[i].inverse();		
				dGripper[i] = (double)UI2Madata.scale_grip*(oldGripper[i]-Gripper[i])*1000;
			} 
			else 
			{
				dPosition[i].setZero(); // incremental
				qIncr[i].Identity();							
				dGripper[i]=0;
			}			
		}

		rl2sui = 3-comm.Check_Flag(FPEDAL_RIGHT);
		int grip2[2];
		grip2[0]=(int)dGripper[0];
		grip2[1]=(int)dGripper[1];
		// compute projected force on each gripper finger

		bool rcvudp;
		int fps = FPS.GetFrameRate();
		if(servo%100 == 1) {
			comm.Update_MA2UI(dPosition, rl2sui, servo, 0, 0, 0, 0, 0,fps);			// Update SUI data
			comm.Send_TCP();
			//cout<<rl2sui;
		}
		//Gripper[i]=Gripper[i]-300;
		if(comm.Check_Flag(BASIC_PROGRAM)) { // when UI allows 'Server' run
			comm.Update_UDP_Data(dPosition, qIncr, dGripper, grip2, footpedal, servo, NULL );  // Update robot command packet
			comm.Send_UDP();
			rcvudp=comm.Recv_UDP();
			if(prevpedal>footpedal && !firstime)
				firstime=true;
			if (!firstraven && firstime && rcvudp)
			{
				firstime=false;
				firstraven=true;
			}
		}
	}
	stopHapticDevice(devHandle);
	return NULL;
}

void InitHapticDevice(int * const &devHandle)
{
	int done = 0;
	int deviceCount;
	// get device count
	deviceCount = dhdGetDeviceCount ();
	if (deviceCount < 1) {
		printf ("error: %s\n", dhdErrorGetLastStr ());
		return;
	}

	// open the first available device
	int flag_hand=0;
	if ((devHandle[0] = drdOpenID (0)) < 0){
		if(!dhdIsLeftHanded(devHandle[0]))
		{
			devHandle[1]=devHandle[0];
			flag_hand=1;
		}
		printf ("error: %s\n", dhdErrorGetLastStr ());
		printf("\nPress any key to quit.\n");
		getchar();
		return;
	}
	if ((devHandle[flag_hand] = drdOpenID (1)) < 0){
		printf ("error: %s\n", dhdErrorGetLastStr ());
		printf("\nPress any key to quit.\n");
		getchar();
		return;
	}
}
void stopHapticDevice(int * const &devHandle)
{
	dhdClose(devHandle[0]);
	dhdClose(devHandle[1]);
}