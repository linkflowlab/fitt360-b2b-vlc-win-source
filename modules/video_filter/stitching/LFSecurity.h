#ifndef _LFSecurity_H_
#define _LFSecurity_H_

#define EXPORT_FILE 0

#define ENABLE_CALC_LOG 0
#define ENABLE_RENDER_LOG 0

using namespace std;

extern string INPUT_DIR;
extern string OUTPUT_DIR;

/*Input/Output Dimensions*/
// Common
extern int OutWidth;
extern int OutHeight;

// RTSP Source(Total)
extern int RTSP_SrcWidth;
extern int RTSP_SrcHeight;

// Video Source(Each)
extern int VID_SrcWidth;
extern int VID_SrcHeight;

// Stop Signal for Providing to FFI interface
extern bool bSigStop;

// Camera pairs enumeration
typedef enum {
	FRONT = 0,
	REAR,
	CAMDIR_END
} camDir_t;

#endif // _LFSecurity_H_
