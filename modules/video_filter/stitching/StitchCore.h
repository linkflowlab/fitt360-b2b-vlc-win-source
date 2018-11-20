#ifndef _STITCHCORE_H_
#define _STITCHCORE_H_

#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

typedef struct _stCalcParam{
        int num_images; // Num of Images in each sequence
        double work_scale;
        double seam_scale;
        double compose_scale;
        vector<CameraParams> cameras;
        float warped_image_scale;
        double seam_work_aspect;
        vector<int> indices;
        vector<Mat> images;
} stCalcParam;

typedef struct _stRenderParam{
        // Updated recently flag
        bool updated;
        int num_images; // Num of Images in each sequence
        double work_scale;
        double seam_scale;
        double compose_scale;
        vector<CameraParams> cameras;
        float warped_image_scale;
        double seam_work_aspect;
        vector<int> indices;
        vector<Mat> images;
        // valid box size cache for rendering (if doCrop ON)
        Rect validBox;
} stRenderParam;

extern string features_type;
extern bool applyROItoFeatureDetection;

static bool try_cuda = false;
static double work_megapix = -1.0;
static double seam_megapix = 0.1;
static double compose_megapix = -1;
static float conf_thresh = 0.0f;
static string ba_cost_func = "ray";
static string ba_refine_mask = "xxxxx";
static bool do_wave_correct = false;
static WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
static string warp_type = "spherical";
static int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
static float match_conf = 0.3f;
static string seam_find_type = "gc_color";
static int blend_type = Blender::MULTI_BAND;
static float blend_strength = 5;
static int feature_detect_threshold = 6;
static float ratioROI = 0.45;
static bool doCrop = true;
static float filter_conf = 0.8;
static float rect_search_start = 0.01;
static float rect_search_end = 0.75;

// Stitching Parameter buffer in use while cacluation. renderParam will be updated with this value, if the work is suceeded.
static stCalcParam calcParam[2];

// Stitching Parameter which is used by rendering loop. This parameter must not be null always.
static stRenderParam renderParam[2];

void InitParam(camDir_t seq /*0 = front, 1 = rear*/, int num_image_in_each_seq /*video channel in a row*/);
void ResetParam(camDir_t seq);
void UpdateParam(camDir_t seq);
// Memory deallocation for program termination
void DeallocAllParam(camDir_t seq);
int CalcCameraParam(camDir_t seq/*Front = 0, Rear = 1*/, Mat srcImg[]);
int Render(camDir_t seq, Mat srcImg[], Mat destImg);
Rect findMinRect1b(const Mat1b& src);
bool crop2InsideBox(int seq, Mat& src, Mat& dst);
bool isCameraParamValid(camDir_t seq);

#endif // _STITCHCORE_H_
