#include <stdio.h>
#include <stdlib.h>

#include <vlc/vlc.h>

int main(int argc, char **argv)
{
    libvlc_instance_t *inst;
    libvlc_media_player_t *mp;
    libvlc_media_t *m;

	static const char* const args[] = {
        "--vout", "mygl",
        "--alpha-blend-projection",
//        "--alpha-blend-ratio-front", "0.5",
//        "--no-alpha-blend-fit-to-display",
//		"--video-filter", "stitching"
	};

    // load the vlc engine
    inst = libvlc_new(sizeof args/ sizeof *args, args);

    // create a new item
//   m = libvlc_media_new_path(inst, "/home/dragon/project/fitt360-app-security/input/tc1/leftfront.avi");
    m = libvlc_media_new_location(inst, "rtsp://192.168.0.99:8551/all");
	//m = libvlc_media_new_path(inst, "/home/dragon/VID_20180112_172426.mp4");

    // create a media play playing environment
    mp = libvlc_media_player_new_from_media(m);

    // no need to keep the media now
    libvlc_media_release(m);

    // play the media_player
    libvlc_media_player_play(mp);

    // test hand-made APIs

#if 0 //stitching video filter test
    // 0. API 1
    libvlc_set_video_filter(mp, "stitching", true);
    sleep(5);
    libvlc_set_video_filter(mp, "stitching", false);
    sleep(3);
    libvlc_set_video_filter(mp, "stitching", true);
    sleep(5);
    libvlc_set_video_filter(mp, "flipswap", true);
    sleep(10);
    libvlc_set_video_filter(mp, "invert", true);
    sleep(3);
    libvlc_set_video_filter(mp, "stitching", false);
    sleep(3);
    libvlc_set_video_filter(mp, "invert", false);
    sleep(3);
    libvlc_set_video_filter(mp, "flipswap", false);

	// 1. API2
	libvlc_video_set_stitching(mp, 1);
	sleep(10);
	libvlc_video_set_stitching(mp, 0);
	sleep(2);

	// 2. API3
	libvlc_set_video_filters_string(mp, "video-filter", "mirror:invert");
	sleep(2);
	libvlc_set_video_filters_string(mp, "video-filter", "mirror");
	sleep(2);
	libvlc_set_video_filters_string(mp, "video-filter", "");
	sleep(2);
	libvlc_set_video_filters_string(mp, "video-filter", "stitching");

	sleep(30);

    // stop playing
    libvlc_media_player_stop(mp);

    // free the media_player
    libvlc_media_player_release(mp);

    libvlc_release(inst);
#else // alpha blending test
    sleep(5);
/*
    float k = 0.0;
    while(true) {
        libvlc_media_player_set_option_float(mp, "alpha-blend-ratio-front", k);
        float oo = libvlc_media_player_get_option_float(mp, "alpha-blend-ratio-front");
        printf("alpha-blend-ratio-front: %f\n", oo);
        k += 0.001;
        usleep(100000);
    }
    */

    libvlc_media_player_set_option_float(mp, "alpha-blend-ratio-front", 0.25);
    float aa = libvlc_media_player_get_option_float(mp, "alpha-blend-ratio-front");
    printf("alpha-blend-ratio-front: %f\n", aa);
    sleep(5);

    libvlc_media_player_set_option_float(mp, "alpha-blend-ratio-rear", 0.25);
    float bb = libvlc_media_player_get_option_float(mp, "alpha-blend-ratio-rear");
    printf("alpha-blend-ratio-front: %f\n", bb);
    sleep(5);

    libvlc_media_player_set_option_bool(mp, "alpha-blend-fit-to-display", false);
    bool cc = libvlc_media_player_get_option_bool(mp, "alpha-blend-fit-to-display");
    printf("alpha-blend-fit-to-display: %s\n", (cc == true) ? "true" : "false");
    sleep(5);

    libvlc_media_player_set_option_bool(mp, "alpha-blend-fit-to-display", true);
    bool dd = libvlc_media_player_get_option_bool(mp, "alpha-blend-fit-to-display");
    printf("alpha-blend-fit-to-display: %s\n", (dd == true) ? "true" : "false");
    sleep(5);

    sleep(10000);
#endif

    return 0;
}
