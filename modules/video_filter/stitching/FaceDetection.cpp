/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http:www.opensource.org/licenses/bsd-license>
 */

/* Copyright (C) LINKFLOW Co.,Ltd. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Yongjin Kim <aiden@linkflow.co.kr>, September 2018
 */

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

#include "LFSecurity.h"
#include "FaceDetection.h"

// SKIN DETECTION: https://bytefish.de/blog/opencv/skin_color_thresholding/
bool R1(int R, int G, int B)
{
    bool e1 = (R>95) && (G>40) && (B>20) && ((max(R,max(G,B)) - min(R, min(G,B)))>15) && (abs(R-G)>15) && (R>G) && (R>B);
    bool e2 = (R>220) && (G>210) && (B>170) && (abs(R-G)<=15) && (R>B) && (G>B);
    return (e1||e2);
}

bool R2(float Y, float Cr, float Cb)
{
    bool e3 = Cr <= 1.5862*Cb+20;
    bool e4 = Cr >= 0.3448*Cb+76.2069;
    bool e5 = Cr >= -4.5652*Cb+234.5652;
    bool e6 = Cr <= -1.15*Cb+301.75;
    bool e7 = Cr <= -2.2857*Cb+432.85;
    return e3 && e4 && e5 && e6 && e7;
}

bool R3(float H, float S, float V)
{
    return (H<25) || (H > 230);
}

Mat GetSkin(Mat const &src)
{
    // allocate the result matrix
    Mat dst = src.clone();

    Vec3b cwhite = Vec3b::all(255);
    Vec3b cblack = Vec3b::all(0);

    Mat src_ycrcb, src_hsv;
    // OpenCV scales the YCrCb components, so that they
    // cover the whole value range of [0,255], so there's
    // no need to scale the values:
    cvtColor(src, src_ycrcb, CV_BGR2YCrCb);
    // OpenCV scales the Hue Channel to [0,180] for
    // 8bit images, so make sure we are operating on
    // the full spectrum from [0,360] by using floating
    // point precision:
    src.convertTo(src_hsv, CV_32FC3);
    cvtColor(src_hsv, src_hsv, CV_BGR2HSV);
    // Now scale the values between [0,255]:
    normalize(src_hsv, src_hsv, 0.0, 255.0, NORM_MINMAX, CV_32FC3);

    for(int i = 0; i < src.rows; i++) {
        for(int j = 0; j < src.cols; j++) {

            Vec3b pix_bgr = src.ptr<Vec3b>(i)[j];
            int B = pix_bgr.val[0];
            int G = pix_bgr.val[1];
            int R = pix_bgr.val[2];
            // apply rgb rule
            bool a = R1(R,G,B);

            Vec3b pix_ycrcb = src_ycrcb.ptr<Vec3b>(i)[j];
            int Y = pix_ycrcb.val[0];
            int Cr = pix_ycrcb.val[1];
            int Cb = pix_ycrcb.val[2];
            // apply ycrcb rule
            bool b = R2(Y,Cr,Cb);

            Vec3f pix_hsv = src_hsv.ptr<Vec3f>(i)[j];
            float H = pix_hsv.val[0];
            float S = pix_hsv.val[1];
            float V = pix_hsv.val[2];
            // apply hsv rule
            bool c = R3(H,S,V);

            if(!(a&&b&&c))
                dst.ptr<Vec3b>(i)[j] = cblack;
        }
    }
    return dst;
}

bool isSkin(stobj* dat, Mat const &src)
{
    Mat src_ycrcb, src_hsv;
    // OpenCV scales the YCrCb components, so that they
    // cover the whole value range of [0,255], so there's
    // no need to scale the values:
    cvtColor(src, src_ycrcb, CV_BGR2YCrCb);
    // OpenCV scales the Hue Channel to [0,180] for
    // 8bit images, so make sure we are operating on
    // the full spectrum from [0,360] by using floating
    // point precision:
    src.convertTo(src_hsv, CV_32FC3);
    cvtColor(src_hsv, src_hsv, CV_BGR2HSV);
    // Now scale the values between [0,255]:
    normalize(src_hsv, src_hsv, 0.0, 255.0, NORM_MINMAX, CV_32FC3);

    int skinPixel = 0;

    for(int i = 0; i < src.rows; i++) {
        for(int j = 0; j < src.cols; j++) {

            Vec3b pix_bgr = src.ptr<Vec3b>(i)[j];
            int B = pix_bgr.val[0];
            int G = pix_bgr.val[1];
            int R = pix_bgr.val[2];
            // apply rgb rule
            bool a = R1(R,G,B);

            Vec3b pix_ycrcb = src_ycrcb.ptr<Vec3b>(i)[j];
            int Y = pix_ycrcb.val[0];
            int Cr = pix_ycrcb.val[1];
            int Cb = pix_ycrcb.val[2];
            // apply ycrcb rule
            bool b = R2(Y,Cr,Cb);

            Vec3f pix_hsv = src_hsv.ptr<Vec3f>(i)[j];
            float H = pix_hsv.val[0];
            float S = pix_hsv.val[1];
            float V = pix_hsv.val[2];
            // apply hsv rule
            bool c = R3(H,S,V);

            if(a && b && c)
                skinPixel++;
        }
    }

    src_hsv.release();
    src_ycrcb.release();

    // Ratio threshold for deciding wheter skin or not
    if(skinPixel >= src.rows * src.cols * dat->skin_proportion_threshold)
        return true;

    return false;
}

//End of Skin Detection//

static void _detectObjectThread(stobj* dat)
{
    Mat frame_gray;
    vector<Rect> tmpfaces;
    vector<Rect> refinedfaces;
    while(true) {
        if(dat->bSigStop) {
            printf("_detectObjectThread exit\n");
            break;
        }

        if(dat->advFaceDetectThread) {
            dat->advFaceDetectThread = false;
            cvtColor(dat->srcImg, frame_gray, COLOR_BGR2GRAY);
            equalizeHist(frame_gray, frame_gray);

            tmpfaces.clear();
            refinedfaces.clear();
            dat->face_cascade.detectMultiScale(frame_gray, tmpfaces, 1.1, dat->cascade_sensitivity, CV_HAAR_DO_CANNY_PRUNING|CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(dat->face_min_size, dat->face_min_size), Size(dat->face_max_size, dat->face_max_size));

            for(int i = 0; i < tmpfaces.size(); i++) {
                Mat rectFace = dat->srcImg(tmpfaces[i]);

                // To speed up, resize face samples to smaller one here.
                Mat scaledRectFace;
                resize(rectFace, scaledRectFace, cv::Size(dat->face_min_size, dat->face_min_size), 0, 0, INTER_NEAREST);
                if(isSkin(dat, scaledRectFace))
                    refinedfaces.push_back(tmpfaces[i]);
            }

            //Mat sk = GetSkin(dat->srcImg);
            //imshow("tt", sk);
            //waitKey(1);

            dat->mtxFaceDetect.lock();
            dat->faces.swap(refinedfaces);
            dat->mtxFaceDetect.unlock();

            dat->advFaceDetect = true;

            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // Release memory
    frame_gray.release();

    vector<Rect>().swap(refinedfaces);
    refinedfaces.clear();

    vector<Rect>().swap(tmpfaces);
    tmpfaces.clear();
}

void InitFaceDetection(stobj* dat)
{
    if(!dat->face_cascade.load(dat->face_cascade_name)) {
        printf("--(!)Error loading face cascade\n");
        return;
    };

    std::thread detectObjThread(_detectObjectThread, dat);
}

void InitFaceDetectionAndGetRef(stobj* dat, std::thread& thread) {
    if(!dat->face_cascade.load(dat->face_cascade_name)) {
        printf("--(!)Error loading face cascade\n");
        return;
    };

    thread = std::thread(_detectObjectThread, dat);
}

void RunFaceDetectionIfPossible(stobj* dat, Mat &image)
{
    if(dat->advFaceDetect) {
        dat->advFaceDetect = false;
        image.convertTo(dat->srcImg, CV_8U);
        dat->advFaceDetectThread = true;
    }
}

void GetFaceDetectedResult(stobj* dat, vector<Rect> &faceRects)
{
    // Copy face vector
    dat->mtxFaceDetect.lock();
    faceRects.assign(dat->faces.begin(), dat->faces.end());
    dat->mtxFaceDetect.unlock();
}
