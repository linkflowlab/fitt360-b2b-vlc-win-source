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

using namespace std;

static int  Create      ( vlc_object_t * );
static void Destroy     ( vlc_object_t * );

static picture_t *Filter( filter_t *, picture_t * );
static int FilterCallback( vlc_object_t *, char const *, vlc_value_t, vlc_value_t, void * );

vlc_module_begin ()
    set_description( N_("test") )
    set_shortname( N_("test" ))
    set_help( N_("test") )
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

    printf("TEST plugin created\n");

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

    printf("TEST Plugin destroyed\n");
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

static picture_t *Filter( filter_t *p_filter, picture_t *p_pic )
{
    if(!p_pic) return NULL;

    picture_t *p_outpic = filter_NewPicture( p_filter );
    if(!p_outpic) {
        msg_Warn( p_filter, "can't get output picture" );
        picture_Release( p_pic );
        return NULL;
    }

    return p_outpic;
}

static int FilterCallback ( vlc_object_t *p_this, char const *psz_var,
                            vlc_value_t oldval, vlc_value_t newval, void *p_data )
{
    (void) p_this; (void)oldval;
    filter_sys_t *p_sys = (filter_sys_t *)p_data;

    return VLC_SUCCESS;
}
