#ifndef _STITCH_H_
#define _STITCH_H_

#ifdef __MINGW32__
#include "mingw.thread.h"
#include "mingw.mutex.h"
#else
#include <thread>
#include <mutex>
#endif

#include "LFSecurity.h"

using namespace std;
using namespace cv;

// Threading for parameter calcuation
struct PrivThreads {
    void _CalcFrontThread(stobj* dat);
    void _CalcRearThread(stobj* dat);
};

void InitStreamStitcher(stobj* dat, int srcWidth, int srcHeight, int outWidth, int outHeight, int fType, int interval = 2000/*recalculation interval in ms*/, bool bFaceDetectON = false);
void RunStreamStitcher(stobj* dat, PrivThreads &calcThreads);
void BindStreamStitcherInputBuf(stobj* dat);

// Internal Rendering routine for stitching front & rear and merge it to one
void FrameRender(stobj* dat);
#endif // _STITCH_H_
