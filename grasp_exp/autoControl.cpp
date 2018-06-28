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
#include "autoControl.h"
#include <Eigen/Dense>

extern const double PI;
const double deg2rad(const double deg){ return deg * (PI/180.0); }
const double rad2deg(const double rad){ return rad * (180.0/PI); }

using namespace std;
using namespace Eigen;



extern v_struct ravenData;
extern int TimeToStop;
bool WriteToFile = false;;
extern FrameRateCounter FPS;
extern Timer timer;
extern double gRampupRate1;
extern Matrix4d xfs;
extern int footpedal, prevpedal;



//double nullPose[DHD_MAX_DOF] = { 0.08, -0.03, 0.0,  // base  (translations)
//                                 0.0, 0.0, 0.0,  // wrist (rotations)
//                                 0.0 };          // gripper




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


void* autoControlLoop(void* pUserData)
{
	bool indexing= false;
	bool firstime=true;
	bool initialized = false;
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
	//int servo=0;
	// start haptic simulation
	SimulationOn       = true;
	//	SimulationFinished = false;
	char a;
	bool firstFlag = true;
	int repNum = 0;
	double directionRad = 0,zOffRad = 0;
	double dX, dY, dZ;
	int iterNum,iterDelta;
	bool autoFlag = false;
	bool rcvudp;

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
		if (UI2Madata.moveGo)
		{
			WriteToFile = true;
			if (firstFlag)
			{
				// calculate the movement parameters
				firstFlag = false;
				repNum = 1;
				directionRad = deg2rad(UI2Madata.moveParams.moveDirection-90);
				zOffRad = deg2rad(90-UI2Madata.moveParams.moveZOffset);
				iterNum = (int)ceil(UI2Madata.moveParams.moveAmplitude/10000.0 / STEP_CART);
				dX = sin(zOffRad) * sin(directionRad) * UI2Madata.moveParams.moveAmplitude / 10000.0 / iterNum;
				dY = sin(zOffRad) * cos(directionRad) * UI2Madata.moveParams.moveAmplitude / 10000.0 / iterNum;
				dZ = cos(zOffRad) * UI2Madata.moveParams.moveAmplitude / 10000.0 / iterNum;
				iterDelta = UI2Madata.moveParams.moveTime / iterNum;
			/*	if (iterDelta < 1)
					iterDelta = 1;
				continue;*/
			}
			else
			{
				double time0 = timer.get();
				double timeHold = 0;
				while(timeHold < (double)UI2Madata.moveParams.holdTime/1000.0)
				{
					timeHold = timer.get()-time0;
					comm.Update_UDP_Data(dPosition, qIncr, dGripper, (int*)dGripper, footpedal, servo, &xfs );  // Update robot command packet
					comm.Send_UDP();
					rcvudp=comm.Recv_UDP();
					servo++;
				}
			}
			if (UI2Madata.moveGo)
			{
			dPosition[0].x() = -dX;
			dPosition[0].y() = dY;
			dPosition[0].z() = dZ;
			autoFlag = true;
			}
			repNum++;
			//printf("rep: %i time: %d4.2 /r");
			cout << "rep:"<< repNum << " time:"<< timer.get() << endl;
			if (repNum > UI2Madata.moveParams.moveRepetitions)
			{
				UI2Madata.moveGo = false;
				firstFlag = true;
			}
			
		}
			
		if( comm.Check_Flag(FPEDAL_RIGHT) && comm.Check_Flag(BASIC_START) )
		{
			if (prevpedal!=footpedal)
			{
				firstraven=false;
			}

			cout.setf(ios::fixed,ios::floatfield);
			cout.precision(3);
		}

		rl2sui = 3-comm.Check_Flag(FPEDAL_RIGHT);
		int grip2[2];
		grip2[0]=(int)dGripper[0];
		grip2[1]=(int)dGripper[1];

		
		int fps = FPS.GetFrameRate();
		if(servo%1000 == 1) {
			comm.Update_MA2UI(dPosition, rl2sui, servo, 0, 0, 0, 0, 0,fps);			// Update SUI data
			comm.Send_TCP();
			//cout<<rl2sui;
		}
		if(comm.Check_Flag(BASIC_PROGRAM)) { // when UI allows 'Server' run
			if (autoFlag)
			{
				for (int i=0;i<iterNum-1;i++)
				{
					comm.Update_UDP_Data(dPosition, qIncr, Gripper, grip2, footpedal, servo, &xfs );  // Update robot command packet
					comm.Send_UDP();
					//cout << i << endl;
					rcvudp=comm.Recv_UDP();			
					Sleep(iterDelta);
					servo++;
				}
				autoFlag = false;
			}
			else
			{
				comm.Update_UDP_Data(dPosition, qIncr, dGripper, grip2, footpedal, servo, &xfs );  // Update robot command packet
				comm.Send_UDP();
				rcvudp=comm.Recv_UDP();
				//servo++;
			}
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
void* logHaptics(void* pUserData2)
{
	//ofstream myfile;
	//char file_name[50];
	//sprintf(file_name, "example_%i.txt", trial_number);
	//myfile.open(file_name);
	////! simulation clock
	//cPrecisionClock logClock;
	//logClock.reset();
	//logClock.start();
	//while (status<2)
	int filenumber = 1;
	
	int mark1minute = 0;
	ofstream myfile;
	bool newfile = true;
	double prevTime = 0;
	double currTime = 0;
	while (!WriteToFile);

	double firstTime = timer.get();
	firstTime = timer.get();
	while(!TimeToStop && (UI2Madata.controlMode == 2))
	{
		if(newfile)
		{
			char file_name[50];
			char file_path[50];
			sprintf(file_name, "try1_trial_%i.txt",filenumber);
			//			strcpy(file_path, "logdata");
			strcpy(file_path, "data\\");
			strcat(file_path, file_name);
			myfile.open(file_path);
			myfile << "Time" << "\t" << "raven.x" << "\t" << "raven.y" << "\t" << "raven.z" << "\t" 
				"\t" << "raven.qx" << "\t" << "raven.qy" << "\t" << "raven.qz" <<
				"\t" << "raven.qw" << "\t" << "ravenGripper" << endl;
			newfile = false;
			mark1minute += 60;
			filenumber += 1;
		}
		do
		currTime = timer.get()-firstTime;
		while (currTime - prevTime < 0.02);
		myfile << currTime << "\t" << ravenData.px[1] << "\t" <<ravenData.py[1] << "\t" << ravenData.pz[1]<< 
			"\t" << ravenData.Qx[1] << "\t" << ravenData.Qy[1] << "\t" << ravenData.Qz[1] <<
			"\t" << ravenData.Qw[1] << "\t" << ravenData.grasp[1] << endl;
		prevTime = currTime;
		
	/*	if (currTime > mark1minute)
		{
			newfile = true;
			myfile.close();
		}*/
		//cout<< " " << hapticData.mark1minute << " " << currTime << " " << hapticData.filenumber <<endl;
		/*}
		myfile.close();*/
	}
	myfile.close();
	return NULL;
}