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
    image_handler_t* p_image_handle;
    picture_t* p_proc_image;
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
    p_sys->p_image_handle = image_HandlerCreate( p_filter );
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
    filter_sys_t* p_sys = (filter_sys_t*)p_filter->p_sys;
    if(p_sys->p_proc_image) {
        picture_Release(p_sys->p_proc_image);
        p_sys->p_proc_image = NULL;
    }
}

static void PictureToBGRMat( filter_t* p_filter, picture_t* p_in, Mat& m)
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

    p_sys->p_proc_image = image_Convert(p_sys->p_image_handle, p_in, &(p_in->format), &fmt_out );

    if (!p_sys->p_proc_image)
    {
        msg_Err(p_filter, "can't convert (unsupported formats?), aborting...");
        return;
    }

    m = Mat(sz, CV_8UC3, p_sys->p_proc_image->p[0].p_pixels);
    cvtColor(m, m, CV_RGB2BGR);
}

static void PrepareResultPicture(filter_t* p_filter, picture_t* ref_pic, picture_t* out_pic)
{
    filter_sys_t* p_sys = (filter_sys_t *)p_filter->p_sys;

    if(!ref_pic || !out_pic) {
        msg_Err( p_filter, "No input pictures" );
        return;
    }

    Size sz = Size(out_pic->format.i_width, out_pic->format.i_height);
    Mat p = Mat(sz, CV_8UC3, p_sys->p_proc_image->p[0].p_pixels);
    cvtColor(p, p, CV_BGR2RGB);

    video_format_t fmt_out;
    memset( &fmt_out, 0, sizeof(video_format_t));
    fmt_out = ref_pic->format;

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
    filter_sys_t *p_sys = (filter_sys_t *)p_filter->p_sys;
    picture_t *p_outpic = filter_NewPicture( p_filter );

    if(!p_pic) return NULL;
    if(!p_outpic) {
        msg_Warn( p_filter, "can't get output picture" );
        picture_Release( p_pic );
        return NULL;
    }

    Mat frame;
    PictureToBGRMat(p_filter, p_pic, frame);
    putText(frame, "LINKFLOW", Point(100, 100), 2, 1.2, Scalar::all(255));
    PrepareResultPicture(p_filter, p_pic, p_outpic);

    ReleaseImages(p_filter);
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
