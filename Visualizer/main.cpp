/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
http://dsc.sensable.com

Module Name: 

main.cpp

Description:

The main file that performs all haptics-relevant operation. Within a 
asynchronous callback the graphics thread reads the position and sets
the force. Within a synchronous callback the graphics thread gets the
position and constructs graphics elements (e.g. force vector).

*******************************************************************************/

#include <iostream>
#include <cstdio>
#include <cassert>
#include <string.h>
#include <stdlib.h>

#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include "helper.h"
#include "sckt/sckt.hpp"
#include "itp_teleoperation.h"

#define M_PI 3.14159

#ifdef WIN32
#include <Windows.h>
#endif

using namespace std;

//- Networking parameters
int   localReceivePort;
sckt::UDPSocket rcvSock;
sckt::IPAddress ip("127.0.0.1", 5060);

struct u_struct in_data;

//- Haptic device record.
struct DeviceDisplayState
{
	HHD m_hHD;
	hduVector3Dd position;
	hduQuaternion q;
	hduVector3Dd angVel;
	hduVector3Dd angles;
	hduVector3Dd force;
	hduMatrix xf;
} state[2];

static double sphereRadius = 4.0;

//- Glut callback functions used by helper.cpp 
void glutKeyboard(unsigned char, int, int);
void displayFunction(void);
void handleIdle(void);

//  Start: network functions

/* Initialize networking */
int netInit(){
	printf("Initializing networking...\n");

	WORD            wVersionRequested; 
	WSADATA         wsaData;

	wVersionRequested   =   MAKEWORD(   1,  1); 
	int nErr;
	if  (   nErr    =   WSAStartup  (   wVersionRequested,  &wsaData))
	{   
		//  error
	}

	localReceivePort =36000;
	sckt::Library socketsLib;
	try {
		rcvSock.Open(localReceivePort);
	}catch(sckt::Exc &e){
		std::cout << "Network error: "<< e.What() <<std::endl;
	}

	printf("Networking started.\n");
	return 0;
}

/* Get the data */

#ifdef WIN32

DWORD WINAPI runsocket(void*){

#else

void* runsocket(void*){

#endif

	// Start nets
	netInit();

	Sleep(1);
	sckt::byte buf[sizeof(struct u_struct)*1000];
	struct u_struct in_u;

	while(1){
		int ret = rcvSock.Recv(buf, sizeof(buf), ip);
		if (ret<0)
			cout << "Failed recv:"<<ret<<"\n";
		else if (ret != sizeof(struct u_struct))
			cout << "Recv'd wrong size:"<<ret<<"\n";
		else{
			memcpy(&in_u, buf, ret);
			for (int i = 1; i<2; i++){
				double unit = 1000 * 1000;
				(state[i]).position[0] += in_u.delx[i]  / (unit);
				(state[i]).position[1] += in_u.dely[i]  / (unit);
				(state[i]).position[2] += in_u.delz[i]  / (unit);

				// orientation update...
				(state[i]).q =  (state[i]).q * hduQuaternion(in_u.Qw[i], hduVector3Dd(in_u.Qx[i], in_u.Qy[i], in_u.Qz[i]));
				state[i].q.normalize();
//				state[i].xf = in_u.transform[i];

				//if (  (state[i]).q.s() < 1 || (state[i]).q.s() < -1) 
				//{
				//	cout <<  "shit ---> fan\n";
				//}
				static hduVector3Dd axs;
				static double ang;
				(state[i]).q.toAxisAngle(axs, ang);
			}
		}
	}
}
//  End: network functions

// Calculate checksum for a teleop packet
double checksumData( )
{
	return 1.0;
}

/*******************************************************************************
Graphics main loop function.
*******************************************************************************/
void displayFunction(void)
{
	//- Setup model transformations.
	glMatrixMode(GL_MODELVIEW); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	setupGraphicsState();

	for (int i=1; i<2; i++) {

		glPushMatrix();
		//- Translate
		hduVector3Dd targetPos = (state[i]).position * 1000;
		//  convertToScreen(targetPos);

		glRotated(90, 0,1,0);
		glRotated(90, 1,0,0);
		glTranslated(targetPos[0], targetPos[1], targetPos[2]);

		// Apply rotation from master
		double angle;

		hduVector3Dd itpX(1,0,0);
		hduVector3Dd itpY(0,1,0);
		hduVector3Dd itpZ(0,0,1);
		glPushMatrix();


		hduQuaternion qtemp;
		qtemp = (state[i]).q;
		//else
		//	qtemp = hduQuaternion( (state[0]).xf );

		static hduVector3Dd axs;
		static double ang;
		qtemp.toAxisAngle(axs, ang);
		glRotated( ang * 180/M_PI, axs[0],axs[1],axs[2]);
		//angle = - (state[i]).angles[2];
		//glRotated( angle * 180/M_PI , itpZ[0], itpZ[1], itpZ[2]);

		//angle = (state[i]).angles[1];
		//glRotated( angle * 180/M_PI , itpY[0], itpY[1], itpY[2]);

		//angle = (state[i]).angles[0];
		//glRotated( angle * 180/M_PI , itpX[0], itpX[1], itpX[2]);

		drawAxes(20);
		glPopMatrix();

		//- Draw a sphere to represent the haptic cursor and the dynamic 
		GLUquadricObj* pQuadObj = gluNewQuadric();
		static const float dynamicSphereColor[4] = { .8, .2, .2, .8 };
		drawSphere(pQuadObj, 
			hduVector3Dd(0,0,0),
			dynamicSphereColor,
			sphereRadius);    

		gluDeleteQuadric(pQuadObj);

		glPopMatrix();
	}
	glutSwapBuffers();

}

/*******************************************************************************
Called periodically by the GLUT framework.
*******************************************************************************/
void handleIdle(void)
{
	glutPostRedisplay();

}


/******************************************************************************
This handler gets called when the process is exiting. Ensures that HDAPI is
properly shutdown
******************************************************************************/
void exitHandler()
{
	return;
}

/******************************************************************************
Main function.
******************************************************************************/
int main(int argc, char* argv[])
{
	//- init position
	for (int i=0; i<3; i++) (state[0]).position[i] = 0;
	for (int i=0; i<3; i++) (state[0]).angles[i]   = 0;

	for (int i=0; i<3; i++) (state[1]).position[i] = 0;
	for (int i=0; i<3; i++) (state[1]).angles[i]   = 0;

	(state[0]).position[1] = -0.01;
	(state[1]).position[1] = 0.01;

	printf("1 Starting application\n");

#ifdef WIN32
	HANDLE hThread= CreateThread(NULL,0,runsocket,NULL,0,NULL);
#else
	pthread_t netthread;
	pthread_create(&netthread, NULL, runsocket, NULL);
#endif  

	initGlut(argc, argv);
	glutKeyboardFunc(glutKeyboard);

	// Get the workspace dimensions.
	HDdouble maxWorkspace[6] = {-200,-200,-200,200,200,200};

	// Low/left/back point of device workspace.
	hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
	// Top/right/front point of device workspace.
	hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
	initGraphics(LLB, TRF);

	// Enter GLUT main loop.
	glutMainLoop(); 

	printf("Done\n");
	return 0;
}

/******************************************************************************/



/*******************************************************************************
GLUT callback for key presses.
*******************************************************************************/
void glutKeyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'r':
	case 'R':
		state[0].position[0]=0;
		state[0].position[1]=-0.01;
		state[0].position[2]=0;
		state[0].angles[0]=0;
		state[0].angles[1]=0;
		state[0].angles[2]=0;
		state[1].position[0]=0;
		state[1].position[1]=0.01;
		state[1].position[2]=0;
		state[1].angles[0]=0;
		state[1].angles[1]=0;
		state[1].angles[2]=0;
		break;
	case 'q':
	case 'Q':
		exit(0);
		break;
	}
}



/******************************************************************************/
