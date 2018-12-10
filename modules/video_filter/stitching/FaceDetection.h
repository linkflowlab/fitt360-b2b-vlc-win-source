#ifndef _FACEDETECTION_H_
#define _FACEDETECTION_H_


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

// Basic
static std::mutex mtxFaceDetect;
static string face_cascade_name = "haarcascade_frontalface_alt2.xml";
static CascadeClassifier face_cascade;

// Image to extract face from
static Mat srcImg;

// Dominant rect vertor of faces
static vector<Rect> faces;

// Detection options
static bool advFaceDetect = true, advFaceDetectThread = false;
static float skin_proportion_threshold = 0.3;
static int cascade_sensitivity = 4;
static int face_min_size = 20;
static int face_max_size = 400;

void InitFaceDetection(stobj* dat);
void InitFaceDetectionAndGetRef(stobj* dat, std::thread& thread);
void RunFaceDetectionIfPossible(Mat &image);
void GetFaceDetectedResult(vector<Rect> &faceRects);

#endif // _FACEDETECTION_H_
