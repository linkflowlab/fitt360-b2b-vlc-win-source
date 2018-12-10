#ifndef _STITCHCORE_H_
#define _STITCHCORE_H_

#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "LFSecurity.h"

using namespace std;
using namespace cv;
using namespace cv::detail;

void InitParam(stobj* dat, camDir_t seq /*0 = front, 1 = rear*/, int num_image_in_each_seq /*video channel in a row*/);
void ResetParam(stobj* dat, camDir_t seq);
void UpdateParam(stobj* dat, camDir_t seq);
// Memory deallocation for program termination
void DeallocAllParam(stobj* dat, camDir_t seq);
int CalcCameraParam(stobj* dat, camDir_t seq/*Front = 0, Rear = 1*/, Mat srcImg[]);
int Render(stobj* dat, camDir_t seq, Mat srcImg[], Mat destImg);
Rect findMinRect1b(stobj* dat, const Mat1b& src);
bool crop2InsideBox(stobj* dat, int seq, Mat& src, Mat& dst);
bool isCameraParamValid(stobj* dat, camDir_t seq);
#endif // _STITCHCORE_H_
