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
using namespace std;
using namespace Eigen;

HapticData hapticData;
extern v_struct ravenData;
extern int TimeToStop;
extern FrameRateCounter FPS;
extern Timer timer;
extern double gRampupRate1;
Matrix4d xfs;
int footpedal, prevpedal;
bool oneOmni=false;
bool leftDevice=false;
bool rightDevice=false;
bool greenArm=false;
bool goldArm=false;
bool firstime=true;

//double nullPose[DHD_MAX_DOF] = { 0.08, -0.03, 0.0,  // base  (translations)
//                                 0.0, 0.0, 0.0,  // wrist (rotations)
//                                 0.0 };          // gripper

double gDeviceOffset1X=0;// 0.063;//0.07
double gDeviceOffset1Y=0;// 0.06;
double gDeviceOffset1Z=0;// 0.062;
double gMaxForceRatio= 1.0;
const double PI = 3.1415926535897932384626433832795;
double ang=PI/4;
extern double nullPose[DHD_MAX_DOF];
//bool WriteToFile=false;
double CurrentTime;
int devHandle[2];
bool ExpOn = false;
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
double scale_pos;
double scale_grip;
double base_z;
double base_y;
double base_x;
Vector3d zerobase(0,0,0);

extern Sigma_Comm comm;
extern int g_footpedal;
extern bool SimulationOn;
#include "defines.h"
extern unsigned int servo;
bool indexing= false;
double nullbase[3] = { 0.0, 0.0, 0.0} ; // base  (translations)
double nullrot[3] = {	0.0, 0.0, M_PI/4} ; // wrist (rotations)
double dr;
void* HapticsLoop(void* pUserData)
{

	static double t0 = dhdGetTime ();
	double        t  = t0 + 0.001;
	double        dt,grasp=0;
	double newGrip=0;
	double x,y,z;
	double vx,vy,vz;
	double rx,ry,rz;
	bool          initialized = false;

	int bttn=0;
	int rl2sui;
	double   r[3][3];
	Vector3d Position[2], oldPosition[2], dPosition[2], ravenPosition[2],ravendPosition[2],deltaPosition[2],forcePosition[2];
	double forceGrip[2];
	bool firstraven=false;
	Vector3d Velocity[2];
	Vector3d Orientation[2];
	double Gripper[3], oldGripper[3], dGripper[3];
	double Vgrip[3];
	//static Vector3d oldPosistion[2];
	static Vector3d oldVelocity[2];
	Quaterniond qIncr[2],qCurr[2],qPrev[2];
	Matrix3d xform[2];
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

	while(!TimeToStop)
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
			// start with no force

			dhdEnableForce (DHD_ON, devHandle[0]);
			dhdSetDevice(devHandle[0]);
			// POSITION AND VELOCITY OF END EFFECTOR
			dhdGetPosition(&x,&y,&z);
			oldPosition[i]=Position[i];
			oldGripper[i]=Gripper[i];
			Position[i] << x,y,z;
			dhdGetLinearVelocity(&vx,&vy,&vz);
			Velocity[i] << vx,vy,vz;		
			//Position *=0.001;	// mm->m
			//	Velocity *=0.001;	// mm/s->m/s
			// GRIPPER POS and VELOCITY
			dhdGetGripperAngleRad(&Gripper[i]);
			dhdGetGripperGap(&Gripper[2]);
			//cout<<Gripper[1]<<endl;

			d[i]=x-zerobase.x();
			d[1]=y-zerobase.y();
			d[2]=z-zerobase.z();

			dr=sqrt(d[1]*d[1]+d[2]*d[2]+d[i]*d[i]);
			//if (!enableForceFeedback)
			/*{dhdSetForceAndGripperForce
			if (dhdSetForce (0.0, 0.0, 0.0) < DHD_NO_ERROR) 
			printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			}*/
			/*else
			{
			K=7/dr;
			if (K>450)
			K=450;
			if (dhdSetForce (K*-d[i], K*-d[1], K*-d[2]) < DHD_NO_ERROR) 
			printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			}*/
			dhdGetGripperAngularVelocityRad(&Vgrip[i]);
			dhdGetGripperLinearVelocity(&Vgrip[2]);
			// ORIENTATION
			dhdGetOrientationFrame (r);
			dhdGetOrientationRad(&rx,&ry,&rz);
			Matrix3d rollmat, pitchmat, yawmat;
			double KG;
			if(Gripper[i]<0.1)
				KG=40;
			else if(Gripper[i]<0.2)
				KG=4/Gripper[i];
			else
				KG=20;
			KG=0;
			K=0;
			if (dhdSetForceAndGripperForce (K*-d[i], K*-d[1], K*-d[2],(0.5-Gripper[i])*KG) < DHD_NO_ERROR) 
				printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			ry=-ry;
			rollmat<<1, 0, 0, 
				0, cos(rx), -sin(rx),
				0, sin(rx), cos(rx);
			pitchmat<<cos(ry), 0, sin(ry), 
				0, 1, 0,
				-sin(ry), 0, cos(ry);
			yawmat<<cos(rz), -sin(rz), 0, 
				sin(rz), cos(rz), 0,
				0, 0, 1;

			Orientation[i] << rx,ry,rz;
			xform[i]  << r[i][i], r[i][1], r[i][2],
				r[1][i], r[1][1], r[1][2],
				r[2][i], r[2][1], r[2][2];
			Matrix3d xinv;
			/*
			xinv<< 0 , 0, 1 ,
			-1 , 0 ,0 ,
			0, -1, 0;
			xinv<< 0 , 0, 0 ,
			0 , 0 ,1 ,
			0, 0, 0;*/
			if (hapticData.enable_orientation)
				xinv<< 1 , 0, 0 ,
				0 , -1 ,0 ,
				0, 0, -1;

			else
				xinv<< 0 , 0 , 0 ,
				0 ,0 , 0,
				0, 0, 0;
			qPrev[i]=qCurr[i];

			/*qCurr[i]=qPrev[i].inverse()*Quaterniond(xform[i]);*/

			qCurr[i]=Quaterniond(xinv*xform[i]);
			/*qCurr[i]=Quaterniond(yawmat*pitchmat*rollmat);*/
			//qCurr[i]=Quaterniond(pitchmat*rollmat*yawmat);

			/*qCurr[i]=Quaterniond(xform[i]);*/
			hapticData.position[i]=Position[i];
			hapticData.velocity[i]=Velocity[i];
			hapticData.gripper[i]=Gripper[i];
			hapticData.gripper[1]=Gripper[1];
			hapticData.vgrip[i]=Vgrip[i];
			hapticData.vgrip[1]=Vgrip[1];
			hapticData.orientation[i]=Orientation[i];		

			// COMMUNICATION
			ravenPosition[i].x()=(-ravenData.py[1]);
			ravenPosition[i].y()=(-ravenData.pz[1]);
			ravenPosition[i].z()=ravenData.px[1];

			// check ravendata //
			/*
			cout.setf(ios::fixed,ios::floatfield);
			cout.precision(3);
			cout << '\r' << "x: " <<std::setw(4)<< ravenData.px[1] << ' y: '<<std::setw(4) << ravenData.py[1] << ' z: '<<std::setw(4)<< ravenData.pz[1]  << " g: " <<std::setw(4)<< ravenData.grasp[1] << flush; 
			*/
			// end check ravendata //

			if( comm.Check_Flag(FPEDAL_RIGHT) && comm.Check_Flag(BASIC_START) )
			{
				if (prevpedal!=footpedal)
				{
					firstraven=false;

					deltaPosition[i]= scale_pos*(Position[i]-zerobase)-ravenPosition[i];
				}

				////// POSITION //////
				if (hapticData.enable_position)
					dPosition[i] = scale_pos*(Position[i] - oldPosition[i]); // incremental
					// dPosition[i] = scale_pos*(Position[i]-zerobase) - deltaPosition[i]; //absolute
				else
					// dPosition[i] << ravenPosition[i]; //absolute;
					dPosition[i] = Vector3d().setZero();
				////// END POSITION //////

				////// ORIENTATION ///////
				qIncr[i]=qCurr[i]*qPrev[i].inverse();
				/*qIncr[i]=qPrev[i].inverse()*qCurr[i];*/
				//qIncr[i]=qCurr[i]; //TRYING TO IMPLEMENT ABSOLUTE
				////// END ORIENTATION ///////
				

				/////// GRIPPER ///////
				if (hapticData.enable_gripper)
				{
					Gripper[i]-=0.172;
				//	Gripper[i]*=1000*scale_grip;
					dGripper[i]=Gripper[i]-oldGripper[i];
				}
				else
					Gripper[i]=1000*ravenData.grasp[i];

				/////// END GRIPPER ///////

				ravendPosition[i].x()=ravenData.pxd[i];
				ravendPosition[i].y()=ravenData.pyd[i];
				ravendPosition[i].z()=ravenData.pzd[i];
				forcePosition[i]=(ravenPosition[i]-dPosition[i])*100;
				if(ravenData.grasp[i]<0.7)
					forceGrip[i]=(ravenData.grasp[i]-ravenData.graspd[i])*10;
				else
					forceGrip[i]=0;
				/*	if (forceGrip[i]<0)
				forceGrip[i]=0;*/

				/*cout.setf(ios::fixed,ios::floatfield);
				cout.precision(3);
				cout << '\r' << "F: " <<std::setw(4)<< forcePosition[i].x() << ':'<<std::setw(4) << forcePosition[i].y() << ':'<<std::setw(4)<< forcePosition[i].z() << " G: " <<std::setw(4)<< forceGrip[i] << flush;*/
				/*if (dhdSetForceAndGripperForce (forcePosition[i].x(), forcePosition[i].y(), forcePosition[i].z(),forceGrip[i]) < DHD_NO_ERROR) 
				printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());*/
				if (dhdSetForceAndGripperForce (0,0,0,0) < DHD_NO_ERROR) 
					printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());

			} 
			else 
			{

				indexing=true;
				dPosition[i] << ravenPosition[i];
				//	cout<<dPosition[i].x()<<" "<<dPosition[i].y()<<" "<<dPosition[i].z()<<endl;
				//dPosition[i]<< 0 , 0 ,0 ;
				//qIncr[i]=qCurr[i]*qPrev[i].inverse();
				qIncr[i]=qPrev[i].inverse()*qCurr[i];
				Gripper[i]*=1000*scale_grip;
				if (dhdSetForceAndGripperForce (0.0,0.0,0.0,0.0) < DHD_NO_ERROR) 
					printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
			}
					///// GRIPPER ALIGNMENT /////
			double gForce = -(Gripper[i]-1000*ravenData.grasp[i])*0.1;
			cout.setf(ios::fixed,ios::floatfield);
			 cout.precision(3);
			cout << '\r' << "Grip Force: " <<std::setw(4)<< gForce << flush;
			//if (dhdSetForceAndGripperForce (0.0,0.0,0.0,-(Gripper[i]-1000*ravenData.grasp[i])*0.1) < DHD_NO_ERROR) 
					//printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
		///// END GRIPPER ALIGNMENT /////
		}

		rl2sui = 3-comm.Check_Flag(FPEDAL_RIGHT);
		int grip2[2];
		grip2[0]=(int)Gripper[0];
		grip2[1]=(int)Gripper[1];
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
			comm.Update_UDP_Data(dPosition, qIncr, Gripper, grip2, footpedal, servo, &xfs );  // Update robot command packet
			comm.Send_UDP();
			rcvudp=comm.Recv_UDP();
			if(prevpedal>footpedal && !firstime)
				firstime=true;
			if (!firstraven && firstime && rcvudp)
			{
				firstime=false;
				firstraven=true;
			}



			//comm.Update_UDP_Raven(&rcvHeader);
		}
		//if (servo%200==1 && comm.Check_Flag(BASIC_PROGRAM))
		//{
		//	
		//	//protocol.run();
		//}


	}
	//SimulationFinished = true;
	// return
	return NULL;
}
//void HapticData::moveto()
//{
//				 drdRegulatePos  (true);
//          drdRegulateRot  (true);
//          drdRegulateGrip (true);
//    //      drdStart();
//          drdMoveTo(nullPose);
//      //    drdStop(true);
//		  			 drdRegulatePos  (false);
//          drdRegulateRot  (false);
//          drdRegulateGrip (false);
// 
//}
void HapticData::InitHapticDevice()
{
	int done = 0;
	int deviceCount;
	// get device count
	deviceCount = dhdGetDeviceCount ();
	if (deviceCount < 1) {
		printf ("error: %s\n", dhdErrorGetLastStr ());
		return;
	}
	oneOmni=true;
	greenArm=true;
	goldArm=false;
	leftDevice=false;
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
	//drdRegulatePos  (true);
	//drdRegulateRot  (true);
	//drdRegulateGrip (true);
	//drdStart();
	//drdMoveTo(nullPose);
	//drdMoveToPos(nullbase[i],nullbase[1],nullbase[2]);
	//	drdMoveToRot(nullrot[i],nullrot[1],nullrot[2]);
	//drdStop(true);
	//SetHapticBaseAngle(0,0,0);
	//double newbase;
	//std::cin>>newbase;
	//dhdSetBaseAngleZDeg(newbase);*/
}
void HapticData::SetHapticBaseAngle(double x,double y, double z)
{
	double baseang[3];
	dhdSetBaseAngleXDeg(x);
	dhdSetDeviceAngleDeg(y);
	dhdSetBaseAngleZDeg(z);
	dhdGetBaseAngleXDeg(&baseang[0]);
	dhdGetDeviceAngleDeg(&baseang[1]);
	dhdGetBaseAngleZDeg(&baseang[2]);
	std::cout<<"new base angles:"<<std::endl;
	for(int i=0;i<3;i++)
		std::cout<<baseang[i]<<std::endl;
	double r = sqrt(nullbase[0]*nullbase[0]+nullbase[1]*nullbase[1]);
	//nullbase[i]=r*sin(baseang[2]/180*M_PI);
	//nullbase[1]=r*cos(baseang[2]/180*M_PI);
	//nullPose[i]=nullbase[i];
	//nullPose[1]=nullbase[1];

}
void Ori_feedback(Eigen::Vector3d ori)
{
	//if (graspStatus>2)
	//{
	double a[4]={*ravenData.Qw,*ravenData.Qx,*ravenData.Qy,*ravenData.Qz};
	double w=a[0];
	double x=a[1];
	double y=a[2];
	double z=a[3];
	Quaterniond newCurr(a);
	AngleAxisd theta(newCurr);
	Vector3d raven=theta.axis();
	//Vector3d raven(3,1);
	//raven << ravenData.roll[i], ravenData.pitch[i], ravenData.yaw[i]; 
	Vector3d force=-0.02*(ori-raven);
	double roll= atan2(2*w*y-2*x*z,1-2*y*y-2*z*z);
	double pitch= atan2(2*x*w-2*y*z,1-2*x*x-2*z*z);
	double yaw = asin(2*x*y+2*w*z);
	cout.setf(ios::fixed,ios::floatfield);
	cout.precision(3);
	cout << '\r' << "Q: " <<std::setw(4)<< raven.x() << ':'<<std::setw(4) << raven.y() << ':'<<std::setw(4)<< raven.z() << " A: " <<std::setw(4)<< roll << ':'<<std::setw(4) << pitch << ':'<<std::setw(4)<< yaw
		<< "  Master: "<<std::setw(4) <<ori.x()<<':'<<std::setw(4)<<ori.y()<<':'<<std::setw(4)<<ori.z()<< flush;

	//	dhdSetForceAndTorque(0,0,0,force.x(),force.y(),force.z());
	//}
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
