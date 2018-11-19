/*****************************************************************************
 * Preamble
 *****************************************************************************/
#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <assert.h>
#include <atomic>

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
    image_handler_t* p_image;

    picture_t *p_proc_image;

    Mat frame;
} filter_sys_t;

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
    p_sys->p_image = image_HandlerCreate( p_filter );
    p_sys->p_proc_image = NULL;

    return VLC_SUCCESS;
}

static void Destroy( vlc_object_t *p_this )
{
    filter_t *p_filter = (filter_t *)p_this;
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;

    free( p_sys );
}

static void ReleaseImages( filter_t* p_filter )
{
}

static void PictureToMat( filter_t* p_filter, picture_t* p_in, picture_t* p_out)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!p_out) {
        msg_Err( p_filter, "couldn't get a p_outpic!" );
        return;
    }

    Size sz = Size(p_in->format.i_width, p_in->format.i_height);
    video_format_t fmt_out;

    memset( &fmt_out, 0, sizeof(video_format_t) );

    fmt_out = p_in->format;
    fmt_out.i_chroma = VLC_CODEC_RGB24;

    p_sys->p_proc_image = image_Convert(p_sys->p_image, p_in, &(p_in->format), &fmt_out );

    if (!p_sys->p_proc_image)
    {
        msg_Err(p_filter, "can't convert (unsupported formats?), aborting...");
        return;
    }

    Mat frame = Mat(sz, CV_8UC3, p_sys->p_proc_image->p[0].p_pixels);
    //cvtColor(frame, frame, CV_RGB2BGR);

    picture_CopyPixels( p_out, p_sys->p_proc_image );
    picture_CopyProperties( p_out, p_sys->p_proc_image );
}

/*****************************************************************************
 * Render: displays previously rendered output
 *****************************************************************************
 * This function send the currently rendered image to Mirror image, waits
 * until it is displayed and switch the two rendering buffers, preparing next
 * frame.
 *****************************************************************************/
static picture_t *Filter( filter_t *p_filter, picture_t *p_pic )
{
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;
    picture_t *p_outpic = filter_NewPicture( p_filter );

    if(!p_pic) return NULL;
    if(!p_outpic) {
        msg_Warn( p_filter, "can't get output picture" );
        picture_Release( p_pic );
        return NULL;
    }

    //printf("Planes: %d Width %d, Height %d\n", p_pic->i_planes, p_pic->format.i_width, p_pic->format.i_height);
    //PictureToMat(p_filter, p_pic, p_outpic);

    picture_CopyPixels( p_outpic, p_pic);
    picture_CopyProperties( p_outpic, p_pic);

    picture_Release(p_pic);
    return p_outpic;
}

static int FilterCallback ( vlc_object_t *p_this, char const *psz_var,
                            vlc_value_t oldval, vlc_value_t newval, void *p_data )
{
    (void) p_this; (void)oldval;
    filter_sys_t *p_sys = (filter_sys_t *)p_data;

    return VLC_SUCCESS;
}
