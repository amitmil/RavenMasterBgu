#pragma once
#include "flycapture2.h"
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <Windows.h>
#include <fstream>
#include <direct.h>
#include "GL/gl.h"
#include "GL/glu.h"
#include <GL/glut.h>
#include "FrameRateCounter.h"
#include "..\VS_GUI_SERVER\CommonDS.h"

using namespace FlyCapture2;
void InitBitmapStruct( int cols, int rows );
void printerror( Error error );
int GetMinimumPowerOfTwo(int in);
void DrawScene(int nCam);
void GlutDisplayFunc();
void* visionrun2(void*);
void vsScreenSize (bool fullscreen);

void vsCamOff(double offX[2],double offY[2]);
// opencv_ocl249d.lib;opencv_calib3d249d.lib;opencv_contrib249d.lib;opencv_core249d.lib;opencv_flann249d.lib;opencv_gpu249d.lib;opencv_imgproc249d.lib;opencv_ml249d.lib;opencv_nonfree249d.lib;opencv_photo249d.lib;opencv_stitching249d.lib;opencv_superres249d.lib;opencv_ts249d.lib;opencv_video249d.lib;opencv_videostab249d.lib;opencv_legacy249d.lib;opencv_features2d249d.lib;opencv_highgui249d.lib;