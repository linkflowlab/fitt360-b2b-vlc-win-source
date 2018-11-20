#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <thread>
#include <mutex>
#include <assert.h>
#include <atomic>
#include <iostream>

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_filter.h>
#include <vlc_picture.h>
#include <vlc_image.h>
#include <vlc_modules.h>
#include <vlc_vout.h>

#include "filter_picture.h"

#include "opencv2/core/optim.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core_c.h"

#include "LFSecurity.h"
#include "stitching.h"
#include "LFUtil.h"
#include "StitchCore.h"
#include "FaceDetection.h"

using namespace cv;
using namespace std;

static int  Create      ( vlc_object_t * );
static void Destroy     ( vlc_object_t * );

static picture_t *Filter( filter_t *, picture_t * );
static int FilterCallback( vlc_object_t *, char const *, vlc_value_t, vlc_value_t, void * );

static void PictureToMat( filter_t* p_filter, picture_t* p_in );

vlc_module_begin ()
    set_description( N_("Stitching") )
    set_shortname( N_("Stitching video" ))
    set_help( N_("Stitching front/rear respectively") )
    set_category( CAT_VIDEO )
    set_subcategory( SUBCAT_VIDEO_VFILTER )
    set_capability( "video filter", 0 )
    set_callbacks( Create, Destroy )
vlc_module_end ()

typedef struct {
    image_handler_t* p_image_handle;
    picture_t* p_proc_image;
    picture_t* p_dest_image;
} filter_sys_t;

void InitStreamStitcher(int srcWidth, int srcHeight, int outWidth, int outHeight, string fType, int interval, bool bFaceDetectON)
{
    frame = 0;
    isFrameAvailable = false;

    features_type = fType;
    RTSP_SrcWidth = srcWidth;
    RTSP_SrcHeight = srcHeight;

    // FITT360 Device adaption
    // If bandwidth is under 0.5Mpx, we don't limit feature detection region
    if(RTSP_SrcWidth*RTSP_SrcHeight < 5 * 1e5) {
        if(features_type.compare("orb") == 0) {
            applyROItoFeatureDetection = false;
        }
        padding = 0;
    }

    OutWidth = outWidth;
    OutHeight = outHeight;

    recalc_interval = interval;
    bFaceDetect = bFaceDetectON;

    //Buffer allocation
    RTSPframe = Mat::zeros (RTSP_SrcHeight, RTSP_SrcWidth, CV_8UC3);
    RTSPframe_result = Mat::zeros (OutHeight, OutWidth, CV_8UC3);
}

void RunStreamStitcher()
{
    stitchFrontThread = std::thread(_CalcFrontThread);
    stitchRearThread = std::thread(_CalcRearThread);
    //stitchFrontThread.detach();
    //stitchRearThread.detach();

    if(bFaceDetect)
        InitFaceDetectionAndGetRef(faceDetectThread);
}

void BindStreamStitcherInputBuf()
{
    // subMat은 buffer의 물리적인 주소에 dependent하므로, 데이터의 위치가 셋업 된 이후에 subMat을 설정해주어야 한다.
    partFrames[0] = RTSPframe(cv::Rect(padding, padding, RTSPframe.cols/2 - padding, RTSPframe.rows/2 - padding));
    partFrames[1] = RTSPframe(cv::Rect(RTSPframe.cols/2, padding, RTSPframe.cols/2 - padding, RTSPframe.rows/2 - padding));
    partFrames[2] = RTSPframe(cv::Rect(padding, RTSPframe.rows/2 ,RTSPframe.cols/2 - padding, RTSPframe.rows/2 - padding));
    partFrames[3] = RTSPframe(cv::Rect(RTSPframe.cols/2, RTSPframe.rows/2 ,RTSPframe.cols/2 - padding, RTSPframe.rows/2 - padding));
}

static void _CalcFrontThread()
{
    InitParam(FRONT, 2);
    while(true) {
        if(bSigStop) {
            cout << "CalcFrontThread exit" << endl;
            break;
        }

        if (!isFrameAvailable){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        mtxBuf.lock();
        Mat left = partFrames[0].clone();
        Mat right = partFrames[1].clone();
        mtxBuf.unlock();

        Mat input[2] = {left, right};
        int ret = CalcCameraParam(FRONT, input);
        left.release();
        right.release();
        if(ret != -1 && isCameraParamValid(FRONT)) {
            mtxFrontDraw.lock();
            UpdateParam(FRONT);
            mtxFrontDraw.unlock();
            isParamAvailable[FRONT] = true;

            // If succeded, rest for a while in order to stabilize screen
            std::this_thread::sleep_for(std::chrono::milliseconds(recalc_interval));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

static void _CalcRearThread()
{
    InitParam(REAR, 2);
    while(true) {
        if(bSigStop) {
            cout << "CalcRearThread exit" << endl;
            break;
        }

        if (!isFrameAvailable) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        mtxBuf.lock();
        Mat left = partFrames[2].clone();
        Mat right = partFrames[3].clone();
        mtxBuf.unlock();

        Mat input[2] = {left, right};
        int ret = CalcCameraParam(REAR, input);
        left.release();
        right.release();
        if(ret != -1 && isCameraParamValid(REAR)) {
            mtxRearDraw.lock();
            UpdateParam(REAR);
            mtxRearDraw.unlock();
            isParamAvailable[REAR] = true;

            // If succeded, rest for a while in order to stabilize screen
            std::this_thread::sleep_for(std::chrono::milliseconds(recalc_interval));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void FrameRender()
{
    if(isParamAvailable[FRONT]) {
        mtxFrontDraw.lock();
        Mat input[2] = {partFrames[0], partFrames[1]};
        Render(FRONT, input, RTSPframe_result);
        mtxFrontDraw.unlock();
    } else {
        RTSPframe(Rect(0, 0, RTSPframe.cols, RTSPframe.rows/2)).copyTo(RTSPframe_result(Rect(0, 0, RTSPframe.cols, RTSPframe.rows/2)));
    }

    if(isParamAvailable[REAR]) {
        mtxRearDraw.lock();
        Mat input[2] = {partFrames[2], partFrames[3]};
        Render(REAR, input, RTSPframe_result);
        mtxRearDraw.unlock();
    } else {
        RTSPframe(Rect(0, OutHeight/2, RTSPframe.cols, RTSPframe.rows/2)).copyTo(RTSPframe_result(Rect(0, OutHeight/2, RTSPframe.cols, RTSPframe.rows/2)));
    }

    if(bFaceDetect) {
        RunFaceDetectionIfPossible(RTSPframe_result);

        vector<Rect> faces;
        GetFaceDetectedResult(faces);
        for( size_t i = 0; i < faces.size(); i++ ){
            Point lb(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
            Point tr(faces[i].x, faces[i].y);
            rectangle(RTSPframe_result, lb, tr, Scalar(0, 255, 0), 3, 4, 0);
        }

        // Release vector memory
        // https://stackoverflow.com/questions/10464992/c-delete-vector-objects-free-memory
        vector<Rect>().swap(faces);
        faces.clear();
    }
}

static int Create( vlc_object_t *p_this )
{
    filter_t *p_filter = (filter_t *)p_this;

    switch( p_filter->fmt_in.video.i_chroma ) {
        CASE_PLANAR_YUV_SQUARE
            break;
        default:
            msg_Err( p_filter, "Unsupported input chroma (%4.4s)", (char*)&(p_filter->fmt_in.video.i_chroma) );
            return VLC_EGENERIC;
    }

    if( p_filter->fmt_in.video.i_chroma != p_filter->fmt_out.video.i_chroma )
    {
        msg_Err( p_filter, "Input and output chromas don't match" );
        return VLC_EGENERIC;
    }

    /* Allocate structure */
    filter_sys_t* p_sys = (filter_sys_t*)malloc( sizeof( filter_sys_t ) );
    if(p_sys == NULL )
        return VLC_ENOMEM;

    p_filter->p_sys = p_sys;
    p_filter->pf_video_filter = Filter;
    p_sys->p_image_handle = image_HandlerCreate( p_filter );
    p_sys->p_proc_image = NULL;
    p_sys->p_dest_image = NULL;

    printf("Stitching plugin created\n");

    return VLC_SUCCESS;
}

static void Destroy( vlc_object_t *p_this )
{
    // Prevent destroy repeat
    if(!stitcherInitDone)
        return;

    filter_t *p_filter = (filter_t *)p_this;
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    isParamAvailable[FRONT] = false;
    isParamAvailable[REAR] = false;

    stitcherInitDone = false;
    bSigStop = true;

    if(stitchFrontThread.joinable())
        stitchFrontThread.join();
    if(stitchRearThread.joinable())
        stitchRearThread.join();
    if(bFaceDetect && faceDetectThread.joinable())
        faceDetectThread.join();

    DeallocAllParam(FRONT);
    DeallocAllParam(REAR);

    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
    if(p_sys->p_dest_image) {
        picture_Release(p_sys->p_dest_image);
        p_sys->p_dest_image = NULL;
    }

    RTSPframe.release();
    RTSPframe_result.release();

    bSigStop = false;

    printf("Stitching Plugin destroyed\n");
    free( p_sys );
}

static void ReleaseImages( filter_t* p_filter )
{
    filter_sys_t* p_sys = (filter_sys_t*)p_filter->p_sys;
    if(p_sys->p_dest_image) {
        picture_Release(p_sys->p_dest_image);
        p_sys->p_dest_image = NULL;
    }
    RTSPframe_result.release();
}

static void PictureToRGBMat( filter_t* p_filter, picture_t* p_in, Mat& m)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!p_in) {
        msg_Err( p_filter, "couldn't get a p_in!" );
        return;
    }

    Size sz = Size(p_in->format.i_width, p_in->format.i_height);
    video_format_t fmt_out;

    memset( &fmt_out, 0, sizeof(video_format_t) );

    fmt_out = p_in->format;
    fmt_out.i_chroma = VLC_CODEC_RGB24;

    // Release previous memory
    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
    RTSPframe.release();

    p_sys->p_proc_image = image_Convert(p_sys->p_image_handle, p_in, &(p_in->format), &fmt_out );

    if (!p_sys->p_proc_image)
    {
        msg_Err(p_filter, "can't convert (unsupported formats?), aborting...");
        return;
    }

    m = Mat(sz, CV_8UC3, p_sys->p_proc_image->p[0].p_pixels);
}

static void PrepareResultPicture(filter_t* p_filter, picture_t* ref_pic, picture_t* out_pic)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!ref_pic || !out_pic) {
        msg_Err( p_filter, "No input pictures" );
        return;
    }

    video_format_t fmt_out;
    memset( &fmt_out, 0, sizeof(video_format_t));
    fmt_out = ref_pic->format;

    // RGB -> YUV
    picture_t* p_outpic_tmp = image_Convert(
            p_sys->p_image_handle,
            p_sys->p_dest_image,
            &(p_sys->p_dest_image->format),
            &fmt_out );

    picture_CopyPixels( out_pic, p_outpic_tmp );
    CopyInfoAndRelease( out_pic, p_outpic_tmp );
}

static void PrepareDestPicture(filter_t* p_filter, picture_t* ref_pic, Mat& m)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;
    if(!ref_pic) {
        msg_Err( p_filter, "couldn't get a reference pic!" );
        return;
    }

    video_format_t fmt_out;
    memset( &fmt_out, 0, sizeof(video_format_t) );
    fmt_out = ref_pic->format;
    fmt_out.i_chroma = VLC_CODEC_RGB24;

    p_sys->p_dest_image = picture_NewFromFormat(&fmt_out);
    Size sz = Size(fmt_out.i_width, fmt_out.i_height);
    m = Mat(sz, CV_8UC3, p_sys->p_dest_image->p[0].p_pixels);
}

static picture_t *Filter( filter_t *p_filter, picture_t *p_pic )
{
    if(!p_pic) return NULL;

    picture_t *p_outpic = filter_NewPicture( p_filter );
    if(!p_outpic) {
        msg_Warn( p_filter, "can't get output picture" );
        picture_Release( p_pic );
        return NULL;
    }

    // Prevent repeat create
    if(!stitcherInitDone) {
        stitcherInitDone = true;
        InitStreamStitcher(p_pic->format.i_width, p_pic->format.i_height,
                p_pic->format.i_width, p_pic->format.i_height, "orb");
        printf("Stitching thread initialized\n");
        RunStreamStitcher();
    }

    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    isFrameAvailable = false;
    mtxBuf.lock();
    // Make OpenCV mat from picture and allocate separately
    PictureToRGBMat(p_filter, p_pic, RTSPframe);
    // Set separated sub mat of each camera
    BindStreamStitcherInputBuf();
    mtxBuf.unlock();
    isFrameAvailable = true;

    // Prepare dest picture on which we draw
    PrepareDestPicture(p_filter, p_pic, RTSPframe_result);

    // Render scene of current input
    FrameRender();

    // Make output picture(YUV) from dest picture(RGB)
    PrepareResultPicture(p_filter, p_pic, p_outpic);

    // Release current output buffer and Mat
    ReleaseImages(p_filter);

    // Release picture in order to get next picture
    picture_Release(p_pic);

    // Print frame count
    //printf("frame: %d\n", frame++);
    return p_outpic;
}

static int FilterCallback ( vlc_object_t *p_this, char const *psz_var,
                            vlc_value_t oldval, vlc_value_t newval, void *p_data )
{
    (void) p_this; (void)oldval;
    filter_sys_t *p_sys = (filter_sys_t *)p_data;

    return VLC_SUCCESS;
}
