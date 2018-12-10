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

void InitFaceDetection(stobj* dat);
void InitFaceDetectionAndGetRef(stobj* dat, std::thread& thread);
void RunFaceDetectionIfPossible(stobj* dat, Mat &image);
void GetFaceDetectedResult(stobj* dat, vector<Rect> &faceRects);
#endif // _FACEDETECTION_H_
