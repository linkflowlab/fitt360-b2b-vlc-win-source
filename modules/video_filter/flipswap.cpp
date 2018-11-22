/*****************************************************************************
 * flipswap.c : Flipswap video plugin for vlc
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

using namespace cv;
using namespace std;

static int  Create      ( vlc_object_t * );
static void Destroy     ( vlc_object_t * );

static picture_t *Filter( filter_t *, picture_t * );
static int FilterCallback( vlc_object_t *, char const *, vlc_value_t, vlc_value_t, void * );

vlc_module_begin ()
    set_description( N_("flipswap") )
    set_shortname( N_("flipswap" ))
    set_help( N_("Flip and Swap lower left and lower right image of quad integrated image") )
    set_category( CAT_VIDEO )
    set_subcategory( SUBCAT_VIDEO_VFILTER )
    set_capability( "video filter", 0 )
    set_callbacks( Create, Destroy )
vlc_module_end ()

typedef struct {
    image_handler_t* p_image_handle;

    picture_t* p_proc_image;
} filter_sys_t;

void FrameRender(filter_t* ptrFilter, Mat& frame)
{
    filter_t *p_filter = (filter_t *)ptrFilter;
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!p_sys)
        return;

    Mat lowerleft = frame(Rect(0, frame.rows/2, frame.cols/2, frame.rows/2));
    Mat lowerright = frame(Rect(frame.cols/2, frame.rows/2, frame.cols/2, frame.rows/2));
    flip(lowerleft, lowerleft, 1);
    flip(lowerright, lowerright, 1);

    Mat tmp = lowerright.clone();
    lowerleft.copyTo(frame(Rect(frame.cols/2, frame.rows/2, frame.cols/2, frame.rows/2)));
    tmp.copyTo(frame(Rect(0, frame.rows/2, frame.cols/2, frame.rows/2)));
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

    printf("FlipSwap plugin created\n");

    return VLC_SUCCESS;
}

static void Destroy( vlc_object_t *p_this )
{
    filter_t *p_filter = (filter_t *)p_this;
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }

    printf("FlipSwap Plugin destroyed\n");
    free( p_sys );
}

static void ReleaseImages( filter_t* p_filter )
{
    filter_sys_t* p_sys = (filter_sys_t*)p_filter->p_sys;
    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
}

static void PictureToRGBMat( filter_t* p_filter, picture_t* p_in, Mat& m)
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
            p_sys->p_proc_image,
            &(p_sys->p_proc_image->format),
            &fmt_out );

    picture_CopyPixels( out_pic, p_outpic_tmp );
    CopyInfoAndRelease( out_pic, p_outpic_tmp );
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

    Mat frame;

    // Make OpenCV mat from picture and allocate separately
    PictureToRGBMat(p_filter, p_pic, frame);

    // Render scene of current input
    FrameRender(p_filter, frame);

    // Make output picture(YUV) from dest picture(RGB)
    PrepareResultPicture(p_filter, p_pic, p_outpic);

    // Release current output buffer and Mat
    ReleaseImages(p_filter);

    // Release picture in order to get next picture
    picture_Release(p_pic);
    frame.release();

    return p_outpic;
}

static int FilterCallback ( vlc_object_t *p_this, char const *psz_var,
                            vlc_value_t oldval, vlc_value_t newval, void *p_data )
{
    (void) p_this; (void)oldval;
    filter_sys_t *p_sys = (filter_sys_t *)p_data;

    return VLC_SUCCESS;
}
