#ifndef _STITCH_H_
#define _STITCH_H_

#ifdef __MINGW32__
#include "mingw.thread.h"
#include "mingw.mutex.h"
#else
#include <thread>
#include <mutex>
#endif

using namespace std;
using namespace cv;

static unsigned long long frame = 0;
static bool stitcherInitDone = false;

// Mutexes for Rendering
static std::mutex mtxBuf;
static std::mutex mtxResultBuf;
static std::mutex mtxFrontDraw;
static std::mutex mtxRearDraw;

// Input Image Buffer
static unsigned char* srcBuf = NULL;
static unsigned char* dstBuf = NULL;

static Mat RTSPframe;
static Mat RTSPframe_result;

// PartFrames
/*-----------------*/
/*|   0   |   1   |*/
/*|----------------*/
/*|   2   |   3   |*/
/*|----------------*/
static Mat partFrames[4];
static bool isFrameAvailable = false;
static bool isParamAvailable[CAMDIR_END] = {false, false};
static bool isFrontParamAvailable = false;
static bool isRearParamAvailable = false;

static short padding = 0;
static int recalc_interval = 2000; /*ms*/

// Enable face detection
static bool bFaceDetect = false;

void InitStreamStitcher(int srcWidth, int srcHeight, int outWidth, int outHeight, string fType, int interval = 2000/*recalculation interval in ms*/, bool bFaceDetectON = false);
void RunStreamStitcher();
void BindStreamStitcherInputBuf();

// Internal Rendering routine for stitching front & rear and merge it to one
void FrameRender();

// Test execution code
void TestRun(const char* addr = NULL);
void RawTestRun(const char* addr = NULL);

// Functions internally used
bool processvideo(const char* filename = NULL);
bool processimage(const char* filename = NULL);
bool processRTSP(const char* address = NULL);

// Threading for parameter calcuation
static void _CalcFrontThread();
static void _CalcRearThread();
// Threading for getting latest frame from opencv videoIO
static void _GetRecentFrame(VideoCapture &cap, Mat &target);

std::thread stitchFrontThread;
std::thread stitchRearThread;
std::thread faceDetectThread;

#endif // _STITCH_H_
