/* Copyright (C) LINKFLOW Co.,Ltd. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Yongjin Kim <aiden@linkflow.co.kr>, September 2018
 * ------------------------------------------------------------------------
 * This file is based on OpenCV sample file(stitching_detailed.cpp) on OpenCV 3.0 Linux.
 * We don't use non-free extra feature from opencv_contrib.
 * Modified by Yongjin Kim of LINKFLOW Co., Ltd, August 2017.
*/

#include <iostream> 

#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/warpers.hpp"

#include "LFSecurity.h"
#include "StitchCore.h"

#if ENABLE_CALC_LOG == 1
#define LOGC(msg) cout<<msg<<endl;
#else
#define LOGC(msg)
#endif

#if ENABLE_RENDER_LOG == 1
#define LOGR(msg) cout<<msg<<endl;
#else
#define LOGR(msg)
#endif

using namespace std;
using namespace cv;
using namespace cv::detail;

bool applyROItoFeatureDetection = true;
string features_type = "orb";

void InitParam(camDir_t seq, int num_image_in_each_seq)
{
    calcParam[seq].num_images = num_image_in_each_seq;
    renderParam[seq].num_images = num_image_in_each_seq;

    renderParam[seq].updated = false;
    calcParam[seq].work_scale = 1;
    renderParam[seq].work_scale = 1;
    calcParam[seq].seam_scale = 1;
    renderParam[seq].seam_scale = 1;
    calcParam[seq].compose_scale = 1;
    renderParam[seq].compose_scale = 1;
    calcParam[seq].seam_work_aspect = 1;
    renderParam[seq].seam_work_aspect = 1;

    renderParam[seq].validBox = Rect(-1 ,-1, -1, -1);

    calcParam[seq].cameras.clear();
    calcParam[seq].indices.clear();
    for(int i = 0; i < calcParam[seq].images.size(); i++) {
        calcParam[seq].images[i].release();
    }
    calcParam[seq].images.clear();
}

void ResetParam(camDir_t seq)
{
    calcParam[seq].work_scale = 1;
    calcParam[seq].seam_scale = 1;
    calcParam[seq].compose_scale = 1;
    calcParam[seq].seam_work_aspect = 1;

    calcParam[seq].cameras.clear();
    calcParam[seq].indices.clear();
    for(int i = 0; i < calcParam[seq].images.size(); i++) {
        calcParam[seq].images[i].release();
    }
    calcParam[seq].images.clear();
}

void UpdateParam(camDir_t seq)
{
    for(int i = 0; i < renderParam[seq].images.size(); i++)
        renderParam[seq].images[i].release();

    renderParam[seq].updated = true;
    renderParam[seq].work_scale = calcParam[seq].work_scale;
    renderParam[seq].seam_scale = calcParam[seq].seam_scale;
    renderParam[seq].compose_scale = calcParam[seq].compose_scale;
    renderParam[seq].seam_work_aspect = calcParam[seq].seam_work_aspect;
    renderParam[seq].warped_image_scale = calcParam[seq].warped_image_scale;

    renderParam[seq].cameras.swap(calcParam[seq].cameras);
    renderParam[seq].indices.swap(calcParam[seq].indices);
    renderParam[seq].images.swap(calcParam[seq].images);
}

void DeallocAllParam(camDir_t seq)
{
	for(int i = 0; i < renderParam[seq].images.size(); i++) {
        renderParam[seq].images[i].release();
	}
	for(int i = 0; i < calcParam[seq].images.size(); i++) {
        calcParam[seq].images[i].release();
    }

	vector<Mat>().swap(calcParam[seq].images);
	vector<CameraParams>().swap(calcParam[seq].cameras);
	vector<int>().swap(calcParam[seq].indices);

	vector<Mat>().swap(renderParam[seq].images);
	vector<CameraParams>().swap(renderParam[seq].cameras);
	vector<int>().swap(renderParam[seq].indices);
}

int CalcCameraParam(camDir_t seq, Mat srcImg[])
{
#if ENABLE_CALC_LOG
    int64 app_start_time = getTickCount();
#endif

    LOGC("Finding features...");
#if ENABLE_CALC_LOG
    int64 t = getTickCount();
#endif

    bool is_work_scale_set = false, is_seam_scale_set = false;

    Ptr<FeaturesFinder> finder;
    if (features_type == "surf")
    {
        finder = makePtr<SurfFeaturesFinder>();
    }
    // ORB사용시, OPENCV matcher에서 BFMatcher 및 NORM_HAMMING을 사용하도록 수정하여야 한다.
    else if (features_type == "orb")
    {
        finder = makePtr<OrbFeaturesFinder>();
    }
    else
    {
        cout << "Unknown 2D features type: '" << features_type << "'.\n";
        return -1;
    }

    Mat full_img, img;
    vector<ImageFeatures> features(calcParam[seq].num_images);
    calcParam[seq].images.resize(calcParam[seq].num_images);
    vector<Size> full_img_sizes(calcParam[seq].num_images);

    for (int i = 0; i < calcParam[seq].num_images; ++i)
    {
        if(srcImg[i].empty())
            continue;

        full_img = srcImg[i];
        full_img_sizes[i] = full_img.size();

        if (full_img.empty())
        {
            LOGC("Can't open image: " << i);
            full_img.release();
            img.release();

            return -1;
        }

        if (work_megapix < 0)
        {
            img = full_img;
            calcParam[seq].work_scale = 1;
            is_work_scale_set = true;
        } else {
            if (!is_work_scale_set)
            {
                calcParam[seq].work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), calcParam[seq].work_scale, calcParam[seq].work_scale);
        }

        if (!is_seam_scale_set)
        {
            calcParam[seq].seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            calcParam[seq].seam_work_aspect = calcParam[seq].seam_scale / calcParam[seq].work_scale;
            is_seam_scale_set = true;
        }

        if(applyROItoFeatureDetection) {
            Mat partImg;
            if(i == 0) {
                partImg = img(cv::Rect(img.cols * (1.0 - ratioROI), 0, img.cols * ratioROI, img.rows));
            } else if(i == 1) {
                partImg = img(cv::Rect(0, 0, img.cols * ratioROI, img.rows));
            }

            (*finder)(partImg, features[i]);
            features[i].img_idx = i;

            if(i == 0) {
                for(int j = 0; j < features[i].keypoints.size(); j++) {
                    features[i].keypoints[j].pt.x += img.cols * (1.0 - ratioROI);
                }
            }
        } else {
            (*finder)(img, features[i]);
            features[i].img_idx = i;
        }

        LOGC("Features in image #" << i+1 << ": " << features[i].keypoints.size());

        resize(full_img, img, Size(), calcParam[seq].seam_scale, calcParam[seq].seam_scale);
        calcParam[seq].images[i] = img;
    }

    finder->collectGarbage();
    full_img.release();
    img.release();

    LOGC("[#] Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    // If features are not detected enoughly, decide calculation fails.
    for (int i = 0; i < calcParam[seq].num_images; ++i) {
        if(features[i].keypoints.size() < feature_detect_threshold) {
            ResetParam(seq);
            return -1;
        }
    }

    LOGC("Pairwise matching");
#if ENABLE_CALC_LOG
    t = getTickCount();
#endif
    vector<MatchesInfo> pairwise_matches;
    BestOf2NearestMatcher matcher(try_cuda, match_conf);
    matcher(features, pairwise_matches);
    matcher.collectGarbage();

    // Prevent abnormal pairwise matching result. sometimes "pairwise matching count == all feature count" happens
    if(pairwise_matches[1].num_inliers < 6 || pairwise_matches[1].matches.size() >= features[0].keypoints.size() || pairwise_matches[1].matches.size() >= features[1].keypoints.size()) {
        LOGC("[ERR] Pairwise matching failed : abnormal matching output");
        ResetParam(seq);
        return -1;
    }

    LOGC("Pairwise matched: " << pairwise_matches[0].matches.size() << " / " << pairwise_matches[1].matches.size());
    LOGC("[#] Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

#if ENABLE_CALC_LOG
        t = getTickCount();
#endif

    calcParam[seq].indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);

    HomographyBasedEstimator estimator;
    if (!estimator(features, pairwise_matches, calcParam[seq].cameras))
    {
        cout << "[ERR] Homography estimation failed.\n";
        ResetParam(seq);
        return -1;
    }

    for (size_t i = 0; i < calcParam[seq].cameras.size(); ++i)
    {
        Mat R;
        calcParam[seq].cameras[i].R.convertTo(R, CV_32F);
        calcParam[seq].cameras[i].R = R;
        LOGC("Initial intrinsics #" << calcParam[seq].indices[i]+1 << ":\n" << calcParam[seq].cameras[i].K());
        R.release();
    }

    Ptr<detail::BundleAdjusterBase> adjuster;
    if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
    else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
    else
    {
        cout << "[ERR] Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
        ResetParam(seq);
        return -1;
    }

    adjuster->setConfThresh(conf_thresh);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    adjuster->setRefinementMask(refine_mask);
    if (!(*adjuster)(features, pairwise_matches, calcParam[seq].cameras))
    {
        LOGC("[#] Bundle adjustment, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
        LOGC("[ERR] Camera parameters adjusting failed");
        ResetParam(seq);
        return -1;
    }

    LOGC("[#] Bundle adjustment, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    // Find median focal length
    vector<double> focals;
    for (size_t i = 0; i < calcParam[seq].cameras.size(); ++i)
    {
        LOGC("Camera #" << calcParam[seq].indices[i]+1 << ":\n" << calcParam[seq].cameras[i].K());
        focals.push_back(calcParam[seq].cameras[i].focal);
    }

    sort(focals.begin(), focals.end());
    if (focals.size() % 2 == 1)
        calcParam[seq].warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        calcParam[seq].warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

    if (do_wave_correct)
    {
        vector<Mat> rmats;
        for (size_t i = 0; i < calcParam[seq].cameras.size(); ++i)
            rmats.push_back(calcParam[seq].cameras[i].R.clone());
        waveCorrect(rmats, wave_correct);
        for (size_t i = 0; i < calcParam[seq].cameras.size(); ++i)
            calcParam[seq].cameras[i].R = rmats[i];
    }

    LOGC("[#] Parameter Calculation Finished for sequence " << seq << ", total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl);

    return 0;
}

int Render(camDir_t seq, Mat srcImg[], Mat destImg)
{
#if ENABLE_RENDER_LOG
    int64 app_start_time = getTickCount();
#endif

    LOGR("Warping images (auxiliary)... ");
#if ENABLE_RENDER_LOG
    int64 t = getTickCount();
#endif

    vector<Point> corners(renderParam[seq].num_images);
    vector<UMat> masks_warped(renderParam[seq].num_images);
    vector<UMat> images_warped(renderParam[seq].num_images);
    vector<Size> sizes(renderParam[seq].num_images);
    vector<UMat> masks(renderParam[seq].num_images);

    // Preapre images masks
    for (int i = 0; i < renderParam[seq].num_images; ++i)
    {
        masks[i].create(renderParam[seq].images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    // Warp images and their masks

    Ptr<WarperCreator> warper_creator;
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarper>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarper>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarper>();
        else if (warp_type == "fisheye")
            warper_creator = makePtr<cv::FisheyeWarper>();
        else if (warp_type == "stereographic")
            warper_creator = makePtr<cv::StereographicWarper>();
        else if (warp_type == "compressedPlaneA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
        else if (warp_type == "compressedPlaneA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
        else if (warp_type == "compressedPlanePortraitA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
        else if (warp_type == "compressedPlanePortraitA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
        else if (warp_type == "paniniA2B1")
            warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
        else if (warp_type == "paniniA1.5B1")
            warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
        else if (warp_type == "paniniPortraitA2B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
        else if (warp_type == "paniniPortraitA1.5B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
        else if (warp_type == "mercator")
            warper_creator = makePtr<cv::MercatorWarper>();
        else if (warp_type == "transverseMercator")
            warper_creator = makePtr<cv::TransverseMercatorWarper>();
    }

    if (!warper_creator)
    {
        cout << "[ERR] Can't create the following warper '" << warp_type << "'\n";
        images_warped.clear();
        masks.clear();
        masks_warped.clear();

        return 0;
    }

    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(renderParam[seq].warped_image_scale * renderParam[seq].seam_work_aspect));

    for (int i = 0; i < renderParam[seq].num_images; ++i)
    {
        Mat_<float> K;
        renderParam[seq].cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)renderParam[seq].seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;

        corners[i] = warper->warp(renderParam[seq].images[i], K, renderParam[seq].cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();

        warper->warp(masks[i], K, renderParam[seq].cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
        K.release();
    }

    vector<UMat> images_warped_f(renderParam[seq].num_images);
    for (int i = 0; i < renderParam[seq].num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);

    LOGR("[#] Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    LOGR("Exposure Compensating...");
#if ENABLE_RENDER_LOG
    t = getTickCount();
#endif

    Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
    compensator->feed(corners, images_warped, masks_warped);

    LOGR("[#] Exposure compensation, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    LOGR("Seam Finding...");
#if ENABLE_RENDER_LOG
    t = getTickCount();
#endif

    Ptr<SeamFinder> seam_finder;
    if (seam_find_type == "no")
        seam_finder = makePtr<detail::NoSeamFinder>();
    else if (seam_find_type == "voronoi")
        seam_finder = makePtr<detail::VoronoiSeamFinder>();
    else if (seam_find_type == "gc_color")
    {
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
    }
    else if (seam_find_type == "gc_colorgrad")
    {
            seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
    }
    else if (seam_find_type == "dp_color")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
    else if (seam_find_type == "dp_colorgrad")
        seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
    if (!seam_finder)
    {
        cout << "[ERR] Can't create the following seam finder '" << seam_find_type << "'\n";
        images_warped.clear();
        images_warped_f.clear();
        masks.clear();
        masks_warped.clear();

        return 0;
    }

    seam_finder->find(images_warped_f, corners, masks_warped);

    // Release unused memory
    images_warped.clear();
    images_warped_f.clear();
    masks.clear();

    LOGR("[#] Seam finding, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    LOGR("Compositing...");
#if ENABLE_RENDER_LOG
    t = getTickCount();
#endif

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    //double compose_seam_aspect = 1;
    double compose_work_aspect = 1;
    bool is_compose_scale_set = false;
    vector<Size> full_img_sizes(renderParam[seq].num_images);
    Mat full_img, img;
    for (int i = 0; i < renderParam[seq].num_images; ++i)
        full_img_sizes[i] = srcImg[i].size();

    for (int img_idx = 0; img_idx < renderParam[seq].num_images; ++img_idx)
    {
        LOGR("Compositing image #" << renderParam[seq].indices[img_idx]+1);

        // Read image and resize it if necessary
        full_img = srcImg[img_idx];

        if (!is_compose_scale_set)
        {
            if (compose_megapix > 0)
                renderParam[seq].compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
            is_compose_scale_set = true;

            // Compute relative scales
            //compose_seam_aspect = compose_scale / seam_scale;
            compose_work_aspect = renderParam[seq].compose_scale / renderParam[seq].work_scale;

            // Update warped image scale
            float temp_warped_image_scale = renderParam[seq].warped_image_scale * static_cast<float>(compose_work_aspect);
            //renderParam[seq].warped_image_scale *= static_cast<float>(compose_work_aspect);
            warper = warper_creator->create(temp_warped_image_scale);

            // Update corners and sizes
            for (int i = 0; i < renderParam[seq].num_images; ++i)
            {
                // Update intrinsics
                renderParam[seq].cameras[i].focal *= compose_work_aspect;
                renderParam[seq].cameras[i].ppx *= compose_work_aspect;
                renderParam[seq].cameras[i].ppy *= compose_work_aspect;

                // TEMP: 한번 focal, ppx, ppy를 셋업하고 난 후에는 캐쉬의 compose_scale과 work_scale을 1.0으로 셋업하여 중첩스케일링이 되지 않도록 한다
                renderParam[seq].compose_scale = 1.0;
                renderParam[seq].work_scale = 1.0;

                // Update corner and size
                Size sz = full_img_sizes[i];
                if (std::abs(renderParam[seq].compose_scale - 1) > 1e-1)
                {
                    sz.width = cvRound(full_img_sizes[i].width * renderParam[seq].compose_scale);
                    sz.height = cvRound(full_img_sizes[i].height * renderParam[seq].compose_scale);
                }

                Mat K;
                renderParam[seq].cameras[i].K().convertTo(K, CV_32F);
                Rect roi = warper->warpRoi(sz, K, renderParam[seq].cameras[i].R);
                corners[i] = roi.tl();
                sizes[i] = roi.size();

                K.release();
            }
        }
        if (abs(renderParam[seq].compose_scale - 1) > 1e-1)
            resize(full_img, img, Size(), renderParam[seq].compose_scale, renderParam[seq].compose_scale);
        else
            img = full_img;
        full_img.release();
        Size img_size = img.size();

        Mat K;
        renderParam[seq].cameras[img_idx].K().convertTo(K, CV_32F);

        // Warp the current image
        warper->warp(img, K, renderParam[seq].cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

        // Warp the current image mask
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, renderParam[seq].cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

        // Compensate exposure
        compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();
        K.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());
        resize(dilated_mask, seam_mask, mask_warped.size());
        mask_warped = seam_mask & mask_warped;

        if (!blender)
        {
            blender = Blender::createDefault(blend_type, try_cuda);
            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_cuda);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                LOGR("Multi-band blender, number of bands: " << mb->numBands());
            }
            else if (blend_type == Blender::FEATHER)
            {
                FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
                fb->setSharpness(1.f/blend_width);
                LOGR("Feather blender, sharpness: " << fb->sharpness());
            }
            blender->prepare(corners, sizes);
        }
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }

    Mat result, result_mask;
    blender->blend(result, result_mask);

    LOGR("[#] Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

#if ENABLE_RENDER_LOG
        t = getTickCount();
#endif

    Point ptImgL, ptImgR;
	switch (seq)
	{
	case FRONT:
		ptImgL = Point(0, 0);
		ptImgR = Point(OutWidth/2, 0);
		break;
	case REAR:
		ptImgL = Point(0, OutHeight/2);
		ptImgR = Point(OutWidth/2, OutHeight/2);
		break;
	default:
		LOGR("[ERR] Unexpected ERROR. Wierd image sequence");
		return -1;
	}

	if(!result.empty()) {
		if(doCrop) {
			Mat viewport;
			result.convertTo(result, CV_8UC3);
			if(crop2InsideBox(seq, result, viewport)) {
				resize(viewport, viewport, cv::Size(OutWidth, OutHeight/2), 0, 0, CV_INTER_LINEAR);
				viewport.copyTo(destImg(cv::Rect(ptImgL, Size(viewport.cols, viewport.rows))));
				//rectangle(gray, box, rectColor(255, 0, 0), 2);
			} else {
				// Fall back: If cannot obtain unique rectangle blob, displays separate screen
				resize(srcImg[0], srcImg[0], cv::Size(OutWidth/2, OutHeight/2), 0, 0, CV_INTER_LINEAR);
				srcImg[0].copyTo(destImg(cv::Rect(ptImgL ,Size(srcImg[0].cols, srcImg[0].rows))));
				resize(srcImg[1], srcImg[1], cv::Size(OutWidth/2, OutHeight/2), 0, 0, CV_INTER_LINEAR);
				srcImg[1].copyTo(destImg(cv::Rect(ptImgR, Size(srcImg[1].cols, srcImg[1].rows))));
			}
			viewport.release();
		} else {
			resize(result, result, cv::Size(OutWidth, OutHeight/2), 0, 0, CV_INTER_LINEAR);
			result.copyTo(destImg(cv::Rect(ptImgL ,Size(result.cols, result.rows))));
		}
	} else {
		//Error case
		LOGR("[ERR] Unexpected ERROR. Cannot render this frame");
	}

    LOGR("[#] Image Integration, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

    masks_warped.clear();
    result.release();
    result_mask.release();

	LOGR("[#] Warping Finished for sequence " << seq << ", total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << endl);

    return 1;
}

bool crop2InsideBox(int seq, Mat& src, Mat& dst) {
	if(renderParam[seq].updated) {
		renderParam[seq].updated = false;

		Mat gray, binary;
		cvtColor(src, gray, CV_BGR2GRAY);

		// Detect edges using Threshold
		threshold(gray, binary, 0, 255, THRESH_BINARY);

		// Find contour
		vector<vector<Point> > contours;
		findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		//vector<Vec4i> hierarchy;
		//findContours(binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		//for (int i = 0; i < contours.size(); ++i)
		if(contours.size() >= 0) {
			// Find largest contour
			int ctrIdx = 0;
			int ctrArea = 0;
			for(int i = 0; i < contours.size(); i++) {
				double area = contourArea(contours[i], false);
				if(area > ctrArea) {
					ctrArea = area;
					ctrIdx = i;
				}
			}

			// Create a mask for each single blob
			Mat1b maskSingleContour(gray.rows, gray.cols, uchar(0));
			drawContours(maskSingleContour, contours, ctrIdx, Scalar(255), CV_FILLED);

			// Find minimum rect for each blob
			renderParam[seq].validBox = findMinRect1b(~maskSingleContour);

			dst = src(Rect(renderParam[seq].validBox.x, renderParam[seq].validBox.y, renderParam[seq].validBox.width, renderParam[seq].validBox.height));
		} else if(renderParam[seq].validBox.x >= 0 && renderParam[seq].validBox.x < src.cols && renderParam[seq].validBox.y >= 0 && renderParam[seq].validBox.y < src.rows) {
		    //If fail to find unique contour, then use previous size.
		    //If previous box is larger than current src image, limits to current src image size.
		    //FIXME: Is there more intelligent way to adjust crop size?
			renderParam[seq].validBox.width = ((renderParam[seq].validBox.width + renderParam[seq].validBox.x) < src.cols) ? renderParam[seq].validBox.width : (src.cols - renderParam[seq].validBox.x);
			renderParam[seq].validBox.height = ((renderParam[seq].validBox.height + renderParam[seq].validBox.y) < src.rows) ? renderParam[seq].validBox.height : (src.rows - renderParam[seq].validBox.y);

			dst = src(Rect(renderParam[seq].validBox.x, renderParam[seq].validBox.y, renderParam[seq].validBox.width, renderParam[seq].validBox.height));
		} else {
			renderParam[seq].validBox.x = -1;
			renderParam[seq].validBox.y = -1;
			renderParam[seq].validBox.width = -1;
			renderParam[seq].validBox.height = -1;

			return false;
		}

		return true;
	} else {
		if(renderParam[seq].validBox.width != -1 && renderParam[seq].validBox.height != -1) {
			dst = src(Rect(renderParam[seq].validBox.x, renderParam[seq].validBox.y, renderParam[seq].validBox.width, renderParam[seq].validBox.height));
			return true;
		}
		else
			return false;
	}
}

// https://stackoverflow.com/questions/34896431/creating-rectangle-within-a-blob-using-opencv
// https://stackoverflow.com/a/30418912/5008845
Rect findMinRect1b(const Mat1b& src)
{
    Mat1f W(src.rows, src.cols, float(0));
    Mat1f H(src.rows, src.cols, float(0));

    Rect maxRect(0, 0, 0, 0);
    float maxArea = 0.f;

    register short search_start = (short)((int)src.cols * (float)rect_search_start);
    register short search_end = (short)((int)src.cols * (float)rect_search_end);

    for (int r = 0; r < src.rows; ++r)
    {
        for (int c = 0; c < src.cols; ++c)
        {
            if (src(r, c) == 0)
            {
                H(r, c) = 1.f + ((r>0) ? H(r - 1, c) : 0);
                W(r, c) = 1.f + ((c>0) ? W(r, c - 1) : 0);
            }

            // Heuristic optimization for rect searching(skip scheme)
            if(c > search_end || c < search_start) {
                float minw = W(r, c);
                for (int h = 0; h < H(r, c); ++h)
                {
                    minw = min(minw, W(r - h, c));
                    float area = (h + 1) * minw;
                    if (area > maxArea)
                    {
                        maxArea = area;
                        maxRect = Rect(Point(c - minw + 1, r - h), Point(c + 1, r + 1));
                    }
                }
            }
        }
    }

    return maxRect;
}

bool isCameraParamValid(camDir_t seq)
{
    return true; 

    for(int a = 0; a < calcParam[seq].cameras.size(); a++){
        if(countNonZero(calcParam[seq].cameras[a].R != Mat::eye(3 , 3, CV_32F)) == 0) {
            return false;
        }

        for(int i = 0; i < 9; i += 4) {
            if(calcParam[seq].cameras[a].R.at<float>(i) <= filter_conf) {
                return false;
            }
        }
    }

    return true;
}

static void printUsage()
{
    cout <<
        "Rotation model images stitcher.\n\n"
        "stitching_detailed img1 img2 [...imgN] [flags]\n\n"
        "Flags:\n"
        "  --try_cuda (yes|no)\n"
        "      Try to use CUDA. The default value is 'no'. All default values\n"
        "      are for CPU mode.\n"
        "\nMotion Estimation Flags:\n"
        "  --work_megapix <float>\n"
        "      Resolution for image registration step. The default is 0.6 Mpx.\n"
        "  --features (surf|orb)\n"
        "      Type of features used for images matching. The default is surf.\n"
        "  --match_conf <float>\n"
        "      Confidence for feature matching step. The default is 0.65 for surf and 0.3 for orb.\n"
        "  --conf_thresh <float>\n"
        "      Threshold for two images are from the same panorama confidence.\n"
        "      The default is 1.0.\n"
        "  --ba (reproj|ray)\n"
        "      Bundle adjustment cost function. The default is ray.\n"
        "  --ba_refine_mask (mask)\n"
        "      Set refinement mask for bundle adjustment. It looks like 'x_xxx',\n"
        "      where 'x' means refine respective parameter and '_' means don't\n"
        "      refine one, and has the following format:\n"
        "      <fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'. If bundle\n"
        "      adjustment doesn't support estimation of selected parameter then\n"
        "      the respective flag is ignored.\n"
        "  --wave_correct (no|horiz|vert)\n"
        "      Perform wave effect correction. The default is 'horiz'.\n"
        "\nCompositing Flags:\n"
        "  --warp (plane|cylindrical|spherical|fisheye|stereographic|compressedPlaneA2B1|compressedPlaneA1.5B1|compressedPlanePortraitA2B1|compressedPlanePortraitA1.5B1|paniniA2B1|paniniA1.5B1|paniniPortraitA2B1|paniniPortraitA1.5B1|mercator|transverseMercator)\n"
        "      Warp surface type. The default is 'spherical'.\n"
        "  --seam_megapix <float>\n"
        "      Resolution for seam estimation step. The default is 0.1 Mpx.\n"
        "  --seam (no|voronoi|gc_color|gc_colorgrad)\n"
        "      Seam estimation method. The default is 'gc_color'.\n"
        "  --compose_megapix <float>\n"
        "      Resolution for compositing step. Use -1 for original resolution.\n"
        "      The default is -1.\n"
        "  --expos_comp (no|gain|gain_blocks)\n"
        "      Exposure compensation method. The default is 'gain_blocks'.\n"
        "  --blend (no|feather|multiband)\n"
        "      Blending method. The default is 'multiband'.\n"
        "  --blend_strength <float>\n"
        "      Blending strength from [0,100] range. The default is 5.\n"
        "  --output <result_img>\n"
        "      The default is 'result.jpg'.\n";
}
