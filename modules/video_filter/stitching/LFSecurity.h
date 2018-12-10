#ifndef _LFSecurity_H_
#define _LFSecurity_H_

#define EXPORT_FILE 0

#define ENABLE_CALC_LOG 0
#define ENABLE_RENDER_LOG 0

#ifdef __MINGW32__
#include "mingw.thread.h"
#include "mingw.mutex.h"
#else
#include <thread>
#include <mutex>
#endif

using namespace std;
using namespace cv;

// Camera pairs enumeration
typedef enum {
	FRONT = 0,
	REAR,
	CAMDIR_END
} camDir_t;

typedef struct meta_data_t {
    /*Input/Output Dimensions*/
    // Common
    int OutWidth = 720;
    int OutHeight = 482;

    // RTSP Source(Total)
    int RTSP_SrcWidth = 720;
    int RTSP_SrcHeight = 482;

    // Stop Signal for Providing to FFI interface
    bool bSigStop = false;

    unsigned long long frame = 0;
    bool stitcherInitDone = false;

    // Mutexes for Rendering
    std::mutex mtxBuf;
    std::mutex mtxResultBuf;
    std::mutex mtxFrontDraw;
    std::mutex mtxRearDraw;

    Mat RTSPframe;
    Mat RTSPframe_result;

    // PartFrames
    /*-----------------*/
    /*|   0   |   1   |*/
    /*|----------------*/
    /*|   2   |   3   |*/
    /*|----------------*/
    Mat partFrames[4];
    bool isFrameAvailable = false;
    bool isParamAvailable[CAMDIR_END] = {false, false};

    short padding = 0;
    int recalc_interval = 2000; /*ms*/

    // Enable face detection
    bool bFaceDetect = false;

    std::thread stitchFrontThread;
    std::thread stitchRearThread;
    std::thread faceDetectThread;

    // Stitching Core parameters
    bool applyROItoFeatureDetection = true;
    int features_type = 0; /*0: orb 1: surf*/
} stobj;
#endif // _LFSecurity_H_
