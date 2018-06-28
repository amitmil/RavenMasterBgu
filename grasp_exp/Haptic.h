#ifndef _HAPTIC
#define _HAPTIC
#include <vector>
#include <Eigen/Geometry>
#include "Eigen/Eigen"
#include <iostream>
#include <fstream>
using namespace std;

using namespace Eigen;
class HapticData
{
public:
	HapticData()
	{
		InitHandPos=0.05;
		ForceCount=-1;
		IncreaseForceMaxTime=2.0;
		prevTime = -0.001;
		filenumber = 0;
		newfile = true;
		mark1minute = 0;
	};

	void moveto();
	void InitHapticDevice();
	void SetHapticBaseAngle(double x,double y, double z);
	Vector3d position[2];
	Vector3d velocity[2];
	double gripper[2];
	double prevTime;
	double vgrip[2];
	Vector3d orientation[2];
	float InitHandPos;
	int ForceCount;
	float IncreaseForceMaxTime;
	bool enable_gripper;
	bool enable_orientation;
	bool enable_position;
	bool fullscreen;

	int filenumber, mark1minute;
	bool newfile;

};

void *HapticsLoop(void* pUserData);
//void *logHaptics(void* pUserData2);
void Ori_feedback(Vector3d ori);

#endif