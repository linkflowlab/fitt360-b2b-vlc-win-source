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

#include "opencv2/objdetect.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

// Camera pairs enumeration
typedef enum {
	FRONT = 0,
	REAR,
	CAMDIR_END
} camDir_t;

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

    /* Stitching Core parameters */
    bool applyROItoFeatureDetection = true;
    int features_type = 0; /*0: orb 1: surf*/

	bool try_cuda = false;
	double work_megapix = -1.0;
	double seam_megapix = 0.1;
	double compose_megapix = -1;
	float conf_thresh = 0.0f;
	const string ba_cost_func = "ray";
	const string ba_refine_mask = "xxxxx";
	bool do_wave_correct = false;
	WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
	const string warp_type = "spherical";
	int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
	float match_conf = 0.3f;
	const string seam_find_type = "gc_color";
	int blend_type = Blender::MULTI_BAND;
	float blend_strength = 5;
	int feature_detect_threshold = 6;
	float ratioROI = 0.45;
	bool doCrop = true;
	float filter_conf = 0.8;
	float rect_search_start = 0.01;
	float rect_search_end = 0.75;

	// Stitching Parameter buffer in use while cacluation. renderParam will be updated with this value, if the work is suceeded.
	stCalcParam calcParam[2];

	// Stitching Parameter which is used by rendering loop. This parameter must not be null always.
	stRenderParam renderParam[2];

	/* Face Detection params */
	// Basic
	std::mutex mtxFaceDetect;
	const string face_cascade_name = "haarcascade_frontalface_alt2.xml";
	CascadeClassifier face_cascade;

	// Image to extract face from
	Mat srcImg;

	// Dominant rect vertor of faces
	vector<Rect> faces;

	// Detection options
	bool advFaceDetect = true, advFaceDetectThread = false;
	float skin_proportion_threshold = 0.3;
	int cascade_sensitivity = 4;
	int face_min_size = 20;
	int face_max_size = 400;
} stobj;
#endif // _LFSecurity_H_
