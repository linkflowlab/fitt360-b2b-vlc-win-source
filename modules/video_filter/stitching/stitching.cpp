/*****************************************************************************
 * stitching.c : Stitching video plugin for vlc
 *****************************************************************************
 * Copyright (C) 2018 LINKFLOW Co., Ltd.
 *
 * Authors: Yongjin Kim <aiden@linkflow.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

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

#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

#include "LFSecurity.h"
#include "stitching.h"
#include "StitchCore.h"
#include "FaceDetection.h"

using namespace cv;
using namespace std;

static int  Create      ( vlc_object_t * );
static void Destroy     ( vlc_object_t * );

static picture_t *Filter( filter_t *, picture_t * );
static int FilterCallback( vlc_object_t *, char const *, vlc_value_t, vlc_value_t, void * );

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

    stobj* dat;
    PrivThreads calcThreads;
} filter_sys_t;

void InitStreamStitcher(stobj* dat, int srcWidth, int srcHeight, int outWidth, int outHeight, int fType, int interval, bool bFaceDetectON)
{
    dat->frame = 0;
    dat->isFrameAvailable = false;

    dat->features_type = fType;
    dat->RTSP_SrcWidth = srcWidth;
    dat->RTSP_SrcHeight = srcHeight;

    // If bandwidth is under 0.5Mpx, we don't limit feature detection region
    //if(RTSP_SrcWidth*RTSP_SrcHeight < 5 * 1e5) {
    //    if(features_type.compare("orb") == 0) {
            dat->applyROItoFeatureDetection = false;
    //    }
    //    padding = 0;
    //}

    dat->OutWidth = outWidth;
    dat->OutHeight = outHeight;

    dat->recalc_interval = interval;
    dat->bFaceDetect = bFaceDetectON;
}

void RunStreamStitcher(stobj* dat, PrivThreads &calcThreads)
{
    // FIXME(AIDEN): Now we use non-static thread. Not sure this is better than static here(not tested)
    // Need to check memory garbage
    // https://stackoverflow.com/questions/22657770/using-c-11-multithreading-on-non-static-member-function
    dat->stitchFrontThread = std::thread(&PrivThreads::_CalcFrontThread, &calcThreads, dat);
    dat->stitchRearThread = std::thread(&PrivThreads::_CalcRearThread, &calcThreads, dat);

    if(dat->bFaceDetect)
        InitFaceDetectionAndGetRef(dat, dat->faceDetectThread);
}

void BindStreamStitcherInputBuf(stobj* dat)
{
    // subMat은 buffer의 물리적인 주소에 dependent하므로, 데이터의 위치가 셋업 된 이후에 subMat을 설정해주어야 한다.
    dat->partFrames[0] = dat->RTSPframe(cv::Rect(dat->padding, dat->padding, dat->RTSPframe.cols/2 - dat->padding, dat->RTSPframe.rows/2 - dat->padding));
    dat->partFrames[1] = dat->RTSPframe(cv::Rect(dat->RTSPframe.cols/2, dat->padding, dat->RTSPframe.cols/2 - dat->padding, dat->RTSPframe.rows/2 - dat->padding));
    dat->partFrames[2] = dat->RTSPframe(cv::Rect(dat->padding, dat->RTSPframe.rows/2 ,dat->RTSPframe.cols/2 - dat->padding, dat->RTSPframe.rows/2 - dat->padding));
    dat->partFrames[3] = dat->RTSPframe(cv::Rect(dat->RTSPframe.cols/2, dat->RTSPframe.rows/2 ,dat->RTSPframe.cols/2 - dat->padding, dat->RTSPframe.rows/2 - dat->padding));
}

void PrivThreads::_CalcFrontThread(stobj* dat)
{
    InitParam(FRONT, 2);
    while(true) {
        if(dat->bSigStop) {
            cout << "CalcFrontThread exit" << endl;
            break;
        }

        if (!dat->isFrameAvailable){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        dat->mtxBuf.lock();
        Mat left = dat->partFrames[0].clone();
        Mat right = dat->partFrames[1].clone();
        dat->mtxBuf.unlock();

        Mat input[2] = {left, right};
        int ret = CalcCameraParam(dat, FRONT, input);
        left.release();
        right.release();
        if(ret != -1 && isCameraParamValid(FRONT)) {
            dat->mtxFrontDraw.lock();
            UpdateParam(FRONT);
            dat->mtxFrontDraw.unlock();
            dat->isParamAvailable[FRONT] = true;

            // If succeded, rest for a while in order to stabilize screen
            std::this_thread::sleep_for(std::chrono::milliseconds(dat->recalc_interval));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PrivThreads::_CalcRearThread(stobj* dat)
{
    InitParam(REAR, 2);
    while(true) {
        if(dat->bSigStop) {
            cout << "CalcRearThread exit" << endl;
            break;
        }

        if (!dat->isFrameAvailable) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        dat->mtxBuf.lock();
        Mat left = dat->partFrames[2].clone();
        Mat right = dat->partFrames[3].clone();
        dat->mtxBuf.unlock();

        Mat input[2] = {left, right};
        int ret = CalcCameraParam(dat, REAR, input);
        left.release();
        right.release();
        if(ret != -1 && isCameraParamValid(REAR)) {
            dat->mtxRearDraw.lock();
            UpdateParam(REAR);
            dat->mtxRearDraw.unlock();
            dat->isParamAvailable[REAR] = true;

            // If succeded, rest for a while in order to stabilize screen
            std::this_thread::sleep_for(std::chrono::milliseconds(dat->recalc_interval));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void FrameRender(stobj* dat)
{
    if(dat->isParamAvailable[FRONT]) {
        dat->mtxFrontDraw.lock();
        Mat input[2] = {dat->partFrames[0], dat->partFrames[1]};
        Render(dat, FRONT, input, dat->RTSPframe_result);
        dat->mtxFrontDraw.unlock();
    } else {
        Mat resized;
        if(dat->RTSPframe.cols != dat->RTSPframe_result.cols || dat->RTSPframe.rows != dat->RTSPframe_result.rows) {
            resize(dat->RTSPframe(Rect(0, 0, dat->RTSPframe.cols, dat->RTSPframe.rows/2)), resized, Size(dat->RTSPframe_result.cols, dat->RTSPframe_result.rows/2), INTER_LINEAR);
        } else {
            resized = dat->RTSPframe(Rect(0, 0, dat->RTSPframe.cols, dat->RTSPframe.rows/2));
        }
        resized.copyTo(dat->RTSPframe_result(Rect(0, 0, dat->RTSPframe_result.cols, dat->RTSPframe_result.rows/2)));
    }

    if(dat->isParamAvailable[REAR]) {
        dat->mtxRearDraw.lock();
        Mat input[2] = {dat->partFrames[2], dat->partFrames[3]};
        Render(dat, REAR, input, dat->RTSPframe_result);
        dat->mtxRearDraw.unlock();
    } else {
        Mat resized;
        if(dat->RTSPframe.cols != dat->RTSPframe_result.cols || dat->RTSPframe.rows != dat->RTSPframe_result.rows) {
            resize(dat->RTSPframe(Rect(0, dat->RTSPframe.rows/2, dat->RTSPframe.cols, dat->RTSPframe.rows/2)), resized, Size(dat->RTSPframe_result.cols, dat->RTSPframe_result.rows/2), INTER_LINEAR);
        } else {
            resized = dat->RTSPframe(Rect(0, dat->RTSPframe.rows/2, dat->RTSPframe.cols, dat->RTSPframe.rows/2));
        }
        resized.copyTo(dat->RTSPframe_result(Rect(0, dat->RTSPframe_result.rows/2, dat->RTSPframe_result.cols, dat->RTSPframe_result.rows/2)));
    }

    if(dat->bFaceDetect) {
        RunFaceDetectionIfPossible(dat->RTSPframe_result);

        vector<Rect> faces;
        GetFaceDetectedResult(faces);
        for( size_t i = 0; i < faces.size(); i++ ){
            Point lb(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
            Point tr(faces[i].x, faces[i].y);
            rectangle(dat->RTSPframe_result, lb, tr, Scalar(0, 255, 0), 3, 4, 0);
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

    //stobj* _dat = (stobj*)malloc(sizeof(stobj));
    stobj* _dat = new stobj();
    if(_dat == NULL)
        return VLC_ENOMEM;

    p_sys->dat = _dat;
    p_filter->p_sys = p_sys;
    p_filter->pf_video_filter = Filter;
    p_sys->p_image_handle = image_HandlerCreate( p_filter );
    p_sys->p_proc_image = NULL;
    p_sys->p_dest_image = NULL;

    printf("Open stitching plugin\n");

    return VLC_SUCCESS;
}

static void Destroy( vlc_object_t *p_this )
{
    filter_t *p_filter = (filter_t *)p_this;
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    // Prevent destroy repeat
    if(!p_sys || !p_sys->dat || !p_sys->dat->stitcherInitDone)
        return;

    stobj* dat = p_sys->dat;

    //dat->isParamAvailable[FRONT] = false;
    //dat->isParamAvailable[REAR] = false;
    //dat->stitcherInitDone = false;
    dat->bSigStop = true;

    if(dat->stitchFrontThread.joinable())
        dat->stitchFrontThread.join();
    if(dat->stitchRearThread.joinable())
        dat->stitchRearThread.join();
    if(dat->bFaceDetect && dat->faceDetectThread.joinable())
        dat->faceDetectThread.join();

    DeallocAllParam(FRONT);
    DeallocAllParam(REAR);

    if (p_sys->p_image_handle) {
        image_HandlerDelete( p_sys->p_image_handle );
    }

    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
    if(p_sys->p_dest_image) {
        picture_Release(p_sys->p_dest_image);
        p_sys->p_dest_image = NULL;
    }

    //dat->bSigStop = false;
    if(dat) {
        dat->RTSPframe.release();
        dat->RTSPframe_result.release();
        free(dat);
        dat = NULL;
    }

    printf("Close stitching plugin\n");
    free( p_sys );
}

static void ReleaseImages(stobj* dat, filter_t* p_filter )
{
    filter_sys_t* p_sys = (filter_sys_t*)p_filter->p_sys;
    if(p_sys->p_dest_image) {
        picture_Release(p_sys->p_dest_image);
        p_sys->p_dest_image = NULL;
    }
    dat->RTSPframe_result.release();
}

static void PictureToRGBMat(stobj* dat, filter_t* p_filter, picture_t* p_in, Mat& m)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!p_in) {
        msg_Err( p_filter, "couldn't get a p_in!" );
        return;
    }

    video_format_t fmt_out;
    memset( &fmt_out, 0, sizeof(video_format_t) );
    fmt_out = p_in->format;
    fmt_out.i_chroma = VLC_CODEC_RGB24;

    // Release previous memory
    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
    dat->RTSPframe.release();

    p_sys->p_proc_image = image_Convert(p_sys->p_image_handle, p_in, &(p_in->format), &fmt_out );

    if (!p_sys->p_proc_image)
    {
        msg_Err(p_filter, "can't convert (unsupported formats?), aborting...");
        return;
    }

    Size sz = cvSize(abs(p_in->p[0].i_visible_pitch / p_in->p[0].i_pixel_pitch), abs(p_in->p[0].i_visible_lines));
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
    Size sz = cvSize(abs(ref_pic->p[0].i_visible_pitch / ref_pic->p[0].i_pixel_pitch), abs(ref_pic->p[0].i_visible_lines));
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

    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;
    if(!p_sys || !p_sys->dat) {
        msg_Warn( p_filter, "can't get stitching data" );
        picture_Release( p_pic );
        return NULL;
    }

    stobj* dat = (stobj*)p_sys->dat;

    // Prevent repeat create
    if(!dat->stitcherInitDone) {
        dat->stitcherInitDone = true;
        int width = abs(p_pic->p[0].i_visible_pitch / p_pic->p[0].i_pixel_pitch);
        int height = abs(p_pic->p[0].i_visible_lines);
        InitStreamStitcher(dat, width, height, width, height, 0/*orb*/, 2000, false);
        RunStreamStitcher(dat, p_sys->calcThreads);
    }

    dat->isFrameAvailable = false;
    dat->mtxBuf.lock();
    // Make OpenCV mat from picture and allocate separately
    PictureToRGBMat(dat, p_filter, p_pic, dat->RTSPframe);
    // Set separated sub mat of each camera
    BindStreamStitcherInputBuf(dat);
    dat->mtxBuf.unlock();
    dat->isFrameAvailable = true;

    // Prepare dest picture on which we draw
    PrepareDestPicture(p_filter, p_pic, dat->RTSPframe_result);

    // Render scene of current input
    FrameRender(dat);

    // Make output picture(YUV) from dest picture(RGB)
    PrepareResultPicture(p_filter, p_pic, p_outpic);

    // Release current output buffer and Mat
    ReleaseImages(dat, p_filter);

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
