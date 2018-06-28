#include "PracticalSocket/PracticalSocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <Windows.h>
#include <conio.h>
#include <GL/glut.h>
#include <ctype.h>
#include <string.h>
#include <list>
#include <math.h>
#include <process.h>
#include <mmsystem.h>
#include "Timer.h"
#include <fstream>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <direct.h>
#include "dhdc.h"
#include "Haptic.h"
#include "sigma_comm.h"
#include "VisionThread2.h"
#include "kbControl.h"
#include "fdControl.h"
#include "autoControl.h"
using namespace std;
using namespace Eigen;
//pthread_mutex_t mutexposition;

//pthread_t threads;
const double PI = 3.1415926535897932384626433832795;
#define TIMEOUT_SECS  0
#define TIMEOUT_USECS 100
unsigned int servo=0;
extern HapticData hapticData;
extern v_struct ravenData;
extern Timer timer;
extern int devHandle[2];
extern bool cameraoff;
float CenterX=0;
float CenterY=0;
//extern std::list<hduVector3Dd>	gForceBuff1;
//extern HDSchedulerHandle gHDCallbackHandle;

//const double deg2rad(const double deg){ return deg * (PI/180.0); }
//const double rad2deg(const double rad){ return rad * (180.0/PI); }
			
	bool SimulationOn;
	int g_footpedal=0;
int TimeToStop=0;
int status_keep=0;
int last_status=-1;
int	gMouse_Last_X=0;
int	gMouse_Last_Y=0;

BOOL stereo_enable= true;
bool gMouse_LDOWN	= false;
bool gMouse_MDOWN	= false;
bool gMouse_RDOWN	= false;
bool ActForce=false;
bool count1 = true;
bool count2 = false;
bool switch_key = false;
bool switch_temperature = false;
DWORD ThreadIdControl,ThreadIdLog;
//int ExpMode[]={1,2};
//int ExpMode[]={0,2,1,2,1,3};
//int ExpModeLen=sizeof(ExpMode)/sizeof(*ExpMode);
//int status = ExpMode[protocol.ModeInd];
int status = 0;
int laststatus;
bool NextModeFlag=false; // a flag that says the 'p' keyboard was pressed- when the pong trial will be finished, reaching will appear
//int PongStatus=0;
extern int graspStatus=0;

// calibration data

bool ServoFlag= false;
float initReachPosX=0.0;
float initReachPosY=-0.1;
int TargNum=140;
float TargDist=0.1; // target distance [m]
float Targ[3][2] = {{initReachPosX-sqrt(pow(TargDist,2)/2), initReachPosY+sqrt(pow(TargDist,2)/2)},{initReachPosX, initReachPosY+TargDist},{initReachPosX+sqrt(pow(TargDist,2)/2), initReachPosY+sqrt(pow(TargDist,2)/2)}};
float TargX;
float TargY;

bool HitFlag= false;
bool InUpdateVecFlag=false;

#define ADDR_BUBBLES   "127.0.0.1"
#define ADDR_BUTTERCUP "127.0.0.1"
extern stMA2UI_DATA Ma2UIdata;
extern stUI2MA_DATA UI2Madata;
extern int footpedal;
Sigma_Comm comm(&Ma2UIdata, &UI2Madata);
char ch = 'r';
void ExitHandler()
{
	Sleep(2000);
	while(!cameraoff);

	dhdClose(devHandle[0]);
	dhdClose(devHandle[1]);
	//fclose(filemaster.Data_file_master);
	//fclose(filemaster.Data_file_slave);
}

void StartExperiment (int mode)
{


	// VISION
	//CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(visionrun2), NULL, NULL, &ThreadIdCameras);
	//SetThreadPriority(&ThreadIdCameras, THREAD_PRIORITY_HIGHEST);
	Sleep(2000); 

	// HAPTICS and NETWORK
//	CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(keyBoardLoop), NULL, NULL, &ThreadId1); //KEYBOARD CONTROL
	if(mode<5)
	{
	 WaitForSingleObject(&ThreadIdControl,INFINITE);
	// CloseHandle(&ThreadIdControl);
	}
	else
		mode=0;

	switch (mode)
	{
		case 0:
			CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(keyBoardLoop), NULL, NULL, &ThreadIdControl); // FORCE DIMENSION
			cout<< "KeyBoard Control"<<endl;
			break;
		case 1:
			CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(fdControlLoop), NULL, NULL, &ThreadIdControl); // FORCE DIMENSION
			cout<< "Haptic Device Control"<<endl;
			break;
		case 2:
			CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(autoControlLoop), NULL, NULL, &ThreadIdControl); // FORCE DIMENSION
			cout<< "AutoMove"<<endl;
			CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(logHaptics), NULL, NULL, &ThreadIdLog);
				SetThreadPriority(&ThreadIdLog, THREAD_PRIORITY_HIGHEST);
			break;
	}


	

	SetThreadPriority(&ThreadIdControl, THREAD_PRIORITY_HIGHEST);

	//CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE)(logHaptics), NULL, NULL, &logThread);
}
int main(int argc, char* argv[])
{
	glutInit(&argc, (char**)argv);
	char *input="testing";
	timer.reset();	
	int hstdin = (int) GetStdHandle(STD_INPUT_HANDLE);
	fd_set readSet;
	TIMEVAL timeout = {TIMEOUT_SECS, TIMEOUT_USECS};
	int nfound;
	int the_tcpsock;
	//hapticData.InitHapticDevice();
	// initialize communication with the robot //
	comm.Initialize_UDP_Robot(argc, argv); 
	Sleep(1000);
	while(!comm.Initialize_TCP_GUI()) {
		cout << "TCPIP initialization failed retry in 5 sec." << endl;
		Sleep(500);
	}
	cout << "KS : TCPIP initialized " << endl;	
	SimulationOn=true;
	// Initialize FD_SET //
	FD_ZERO(&readSet);
	the_tcpsock = comm.tcpsocket->sockDesc;
	FD_SET(the_tcpsock, &readSet);
	StartExperiment(5);


	while ( ch != 'q' && !TimeToStop)
	{// 	ENABLE ME   && comm.Check_Flag(BASIC_PROGRAM) ) {
		timeout.tv_sec = TIMEOUT_SECS;
		timeout.tv_usec = TIMEOUT_USECS;
		nfound = select( 1 , &readSet, NULL, NULL, &timeout );		// check the TCP Socket
		if ( nfound < 0 ) {						// Socket error
			cerr << "Socket error "<<nfound<<" : "<<WSAGetLastError()<<" \n";
			cout << "sockedesc:"<< comm.tcpsocket->sockDesc<<endl;
			//			Sleep(10);
			//			break;
		}
		if ( nfound > 0 && FD_ISSET(the_tcpsock, &readSet)) {			// TCP socket ready to read
			//cout << "recv TCP...\n";
			comm.Recv_TCP();
			comm.Check_UI2MA(1);// 0 means not to dispay the message
		} 
		FD_CLR(the_tcpsock,&readSet);					// Reset fd_set
		FD_SET(the_tcpsock,&readSet);					//  ""

		if( _kbhit() )							// check for keyboard input`
		{
			ch = _getch();
			switch(ch)
			{
			case 'd':
				cout << "set pedal down\n";
				g_footpedal = 1;
				break;
			case 'e':
				cout << "set pedal up\n";
				g_footpedal = 0;
				break;
			case 'q':
				SimulationOn=false;
				break;
			}
		}
		if(!SimulationOn){
			fprintf(stderr, "\nThe main scheduler callback has exited\n");
			fprintf(stderr, "\nPress any key to quit.\n");
			_getch();
			break;
		}
		if (footpedal==1)
		{
		}
		if(UI2Madata.flagControlMode)
		{
			UI2Madata.flagControlMode = false;
			StartExperiment(UI2Madata.controlMode);
		}	
	}
	ExitHandler();
}
