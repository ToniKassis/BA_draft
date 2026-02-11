// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#include "AppLogic.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <conio.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <atomic>
#include "glm/ext.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include <Varjo.h>
#include <Varjo_d3d11.h>
#include <Varjo_events.h>
#include <Varjo_mr.h>
#include <Varjo_types_mr.h>

#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <k4abt.hpp>

#include "D3D11MultiLayerView.hpp"

// VarjoExamples namespace contains simple example wrappers for using Varjo API features.
// These are only meant to be used in SDK example applications. In your own application,
// use your own production quality integration layer.
using namespace VarjoExamples;

namespace
{
    // Varjo application priority to force this as background behind other apps.
    // The default value is 0, so we go way less than that.
    constexpr int32_t c_appOrderBg = -1000;

}  // namespace
//---------------------------------------------------------------------------

AppLogic::~AppLogic()
{
    //let all threads join
    if (processFrameThread.joinable()) {
        processFrameThread.join();
    }
    m_camera.reset();
    //free IPCConnector resources
    m_IPCConnector.reset();

    //free point cloud resources
    m_pointCloud.reset();

    // Free data stremer resources
    m_streamer.reset();

    // Free view resources
    m_varjoView.reset();

    m_renderer.reset();

    // Shutdown the varjo session. Can't check errors anymore after this.
    LOG_DEBUG("Shutting down Varjo session..");
    varjo_SessionShutDown(m_session);
    m_session = nullptr;
    m_trackerOk = false;
}

#include <algorithm> // for std::clamp

static k4a::image makeFakeIR(
    const cv::Mat& color,      // BGRA 8-bit
    int target_width,
    int target_height,
    uint16_t clampMin = 0,
    uint16_t clampMax = 3500,  // "IR-like" range
    float cropFactor = 1.0f,   // 0 < cropFactor <= 1, e.g. 0.7 for 70%
    int offsetX = 0,           // shift crop center in pixels (right +, left -)
    int offsetY = 0            // shift crop center in pixels (down +, up -)
)
{
    // Expect BGRA 8-bit image
    CV_Assert(!color.empty());
    CV_Assert(color.type() == CV_8UC4);

    const int src_width = color.cols;
    const int src_height = color.rows;
    const size_t src_stride = color.step[0]; // bytes per row

    if (clampMax <= clampMin)
        clampMax = clampMin + 1;  // avoid zero range

    const uint32_t range = static_cast<uint32_t>(clampMax - clampMin);

    //----------------------------------------------------------------------
    // 1. Convert BGRA -> IR16 (full resolution)
    //----------------------------------------------------------------------

    k4a::image ir_full = k4a::image::create(
        K4A_IMAGE_FORMAT_IR16,
        src_width,
        src_height,
        src_width * static_cast<int>(sizeof(uint16_t))
    );

    const uint8_t* src = color.data;
    uint16_t* dst = reinterpret_cast<uint16_t*>(ir_full.get_buffer());

    for (int y = 0; y < src_height; ++y)
    {
        const uint8_t* row_src = src + y * src_stride;        // BGRA row
        uint16_t* row_dst = dst + y * src_width;         // IR row

        for (int x = 0; x < src_width; ++x)
        {
            const uint8_t* px = row_src + x * 4; // BGRA
            uint8_t b = px[0];
            uint8_t g = px[1];
            uint8_t r = px[2];

            // Grayscale (approx luma)
            uint8_t gray8 = static_cast<uint8_t>(
                (77 * r + 150 * g + 29 * b) >> 8
                );

            // Map 0..255 -> [clampMin, clampMax]
            uint16_t gray16 = static_cast<uint16_t>(
                clampMin + (range * static_cast<uint32_t>(gray8)) / 255u
                );

            row_dst[x] = gray16;
        }
    }

    //----------------------------------------------------------------------
    // 2. Build a cropped view (center crop + offset)
    //----------------------------------------------------------------------

    // clamp cropFactor
    cropFactor = std::max(0.01f, std::min(cropFactor, 1.0f));

    int crop_w = static_cast<int>(std::round(src_width * cropFactor));
    int crop_h = static_cast<int>(std::round(src_height * cropFactor));

    crop_w = std::max(1, std::min(crop_w, src_width));
    crop_h = std::max(1, std::min(crop_h, src_height));

    // center of the source
    int cx = src_width / 2;
    int cy = src_height / 2;

    // top-left of crop region, offset from center
    int x0 = cx - crop_w / 2 + offsetX;
    int y0 = cy - crop_h / 2 + offsetY;

    // clamp so ROI stays inside image
    x0 = std::clamp(x0, 0, src_width - crop_w);
    y0 = std::clamp(y0, 0, src_height - crop_h);

    cv::Mat ir_full_mat(
        src_height,
        src_width,
        CV_16U,
        (void*)ir_full.get_buffer(),
        src_width * static_cast<int>(sizeof(uint16_t))
    );

    cv::Rect roi(x0, y0, crop_w, crop_h);
    cv::Mat ir_cropped = ir_full_mat(roi);  // view only, no copy

    //----------------------------------------------------------------------
    // 3. Resize cropped region to target size (or keep size if matching)
    //----------------------------------------------------------------------

    cv::Mat ir_final_mat;
    if (crop_w == target_width && crop_h == target_height)
    {
        // same size: just copy
        ir_final_mat = ir_cropped.clone();
    }
    else
    {
        cv::resize(ir_cropped, ir_final_mat,
            cv::Size(target_width, target_height),
            0, 0, cv::INTER_NEAREST);
    }

    //----------------------------------------------------------------------
    // 4. Write to final K4A IR16 image
    //----------------------------------------------------------------------

    k4a::image ir_out = k4a::image::create(
        K4A_IMAGE_FORMAT_IR16,
        target_width,
        target_height,
        target_width * static_cast<int>(sizeof(uint16_t))
    );

    memcpy(ir_out.get_buffer(),
        ir_final_mat.data,
        static_cast<size_t>(target_width) * target_height * sizeof(uint16_t));

    return ir_out;
}



static cv::Mat visualize_fakeIR(const k4a::image& ir_image)
{
    if (!ir_image.handle())
        return cv::Mat();   

    const int width = ir_image.get_width_pixels();
    const int height = ir_image.get_height_pixels();

    cv::Mat ir16(height, width, CV_16U,
        (void*)ir_image.get_buffer(),
        ir_image.get_stride_bytes());

    double minVal = 0.0, maxVal = 0.0;
    cv::minMaxLoc(ir16, &minVal, &maxVal);

    static int frameCount = 0;
    if (frameCount < 10)
    {
        std::cout << "IR16 min=" << minVal << " max=" << maxVal << "\n";
        frameCount++;
    }

    if (maxVal <= minVal)
        maxVal = minVal + 1.0;

    cv::Mat ir8;
    ir16.convertTo(
        ir8,
        CV_8U,
        255.0 / (maxVal - minVal),
        -minVal * 255.0 / (maxVal - minVal)
    );

    return ir8;  
}

static cv::Mat k4aColorToMat(const k4a::image& color_image)
{
    int width = color_image.get_width_pixels();
    int height = color_image.get_height_pixels();

    cv::Mat bgra(height, width, CV_8UC4,
        (void*)color_image.get_buffer(),
        color_image.get_stride_bytes());
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);

    return bgr.clone();
}

static k4a::image rasterize_with_cv_projectPoints(
    const std::vector<std::array<float, 3>>& xyz_mm,
    int width, int height,
    float fx, float fy, float cx, float cy,
    // Optional extrinsics
    const cv::Vec3d& rvec = cv::Vec3d(0, 0, 0),
    const cv::Vec3d& tvec = cv::Vec3d(0, 0, 0),
    // Optional radial/tangential 

    const cv::Mat& distCoeffs = cv::Mat(),  //  zeros
    uint16_t z_min_mm = 300, uint16_t z_max_mm = 8000)
{
    // Camera matrix
    cv::Matx33d K(fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    // Convert input to cv::Point3f (keep in millimeters to match z-buffer semantics)
    std::vector<cv::Point3f> pts;
    pts.reserve(xyz_mm.size());
    for (const auto& p : xyz_mm) {
        // p = [Xmm, Ymm, Zmm]
        // Keep only depth-valid range here if you like (optional early filter)
        if (p[2] > 0.0f && p[2] >= z_min_mm && p[2] <= z_max_mm)
            pts.emplace_back(p[0], p[1], p[2]);
    }
    if (pts.empty()) {
        return k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * int(sizeof(uint16_t)));
    }

    // Project with OpenCV
    std::vector<cv::Point2f> imgPts;
    imgPts.reserve(pts.size());
    cv::projectPoints(pts, rvec, tvec, K, distCoeffs, imgPts);

    k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * int(sizeof(uint16_t)));
    uint16_t* d = reinterpret_cast<uint16_t*>(depth.get_buffer());
    std::fill(d, d + size_t(width) * size_t(height), uint16_t(0));

    // Z-buffer: keep nearest z (in mm)
    for (size_t i = 0; i < imgPts.size(); ++i) {
        const float Zmm = pts[i].z;
        if (!(Zmm > 0.0f) || Zmm < z_min_mm || Zmm > z_max_mm) continue;

        const int x = int(std::lround(imgPts[i].x));
        const int y = int(std::lround(imgPts[i].y));
        if (unsigned(x) >= unsigned(width) || unsigned(y) >= unsigned(height)) continue;

        uint16_t& dst = d[size_t(y) * size_t(width) + size_t(x)];
        const uint16_t zmm_u16 = static_cast<uint16_t>(std::lround(Zmm));
        if (dst == 0 || zmm_u16 < dst) dst = zmm_u16;
    }

    return depth;
}

// produces LSD-like depth maps
static k4a::image rasterize_to_depth16_mm_splat(
    const std::vector<std::array<float, 3>>& xyz_mm,
    int width, int height,
    float fx, float fy, float cx, float cy,
    uint16_t z_min_mm = 300, uint16_t z_max_mm = 6000)
{
    // Depth buffer as float (mm) for precise z-tests
    std::vector<float> zbuf(size_t(width) * size_t(height), std::numeric_limits<float>::infinity());

    auto ztest_write = [&](int x, int y, float zmm) {
        if (unsigned(x) >= unsigned(width) || unsigned(y) >= unsigned(height)) return;
        float& dst = zbuf[size_t(y) * width + x];
        if (zmm < dst) dst = zmm; // keep nearest
        };

    // Splat each point to up to 4 neighbors (bilinear footprint)
    for (const auto& p : xyz_mm) {
        const float X = p[0], Y = p[1], Z = p[2];
        if (!(Z > 0.0f)) continue;
        if (Z < z_min_mm || Z > z_max_mm) continue;

        const float uf = fx * (X / Z) + cx;
        const float vf = fy * (Y / Z) + cy;
        const int   x0 = (int)std::floor(uf);
        const int   y0 = (int)std::floor(vf);
        const float ax = uf - (float)x0;
        const float ay = vf - (float)y0;

        ztest_write(x0, y0, Z);
        ztest_write(x0 + 1, y0, Z);
        ztest_write(x0, y0 + 1, Z);
        ztest_write(x0 + 1, y0 + 1, Z);
    }

    // Convert to K4A Depth16 
    k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * int(sizeof(uint16_t)));
    uint16_t* d = reinterpret_cast<uint16_t*>(depth.get_buffer());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float z = zbuf[size_t(y) * width + x];
            d[size_t(y) * width + x] = std::isfinite(z) ? (uint16_t)std::lround(z) : 0u;
        }
    }
    return depth;
}



static cv::Mat pushPullFillDepth(const cv::Mat& d16_in,
    uint16_t zNear_mm, uint16_t zFar_mm,
    int levels = -1, float holeThresh = 1e-12f)
{
    CV_Assert(!d16_in.empty() && d16_in.type() == CV_16UC1);
    const int W = d16_in.cols, H = d16_in.rows;

    if (levels <= 0) {
        int m = std::min(W, H);
        levels = std::max(3, (int)std::floor(std::log2((double)m)) - 1);
        levels = std::min(levels, 10);
    }

    cv::Mat depth_m;  d16_in.convertTo(depth_m, CV_32F, 1.0 / 1000.0);
    cv::Mat valid01 = (d16_in > 0);                 // CV_8U
    valid01.convertTo(valid01, CV_32F, 1.0 / 255.0);  // CV_32F 0/1

    cv::Mat num0 = depth_m.mul(valid01);
    std::vector<cv::Mat> numPyr{ num0 }, wPyr{ valid01 }, valPyr;
    numPyr.reserve(levels); wPyr.reserve(levels); valPyr.reserve(levels);

    for (int i = 1; i < levels; ++i) {
        cv::Size s(std::max(1, numPyr.back().cols / 2), std::max(1, numPyr.back().rows / 2));
        cv::Mat nC, wC;
        cv::resize(numPyr.back(), nC, s, 0, 0, cv::INTER_AREA);
        cv::resize(wPyr.back(), wC, s, 0, 0, cv::INTER_AREA);
        numPyr.push_back(nC); wPyr.push_back(wC);
    }

    const float eps = 1e-8f;
    for (int i = 0; i < levels; ++i) {
        cv::Mat wSafe; cv::max(wPyr[i], eps, wSafe);
        valPyr.push_back(numPyr[i].mul(1.0f / wSafe)); // CV_32F
    }

    cv::Mat curr = valPyr.back().clone();
    for (int i = levels - 2; i >= 0; --i) {
        cv::Mat up;  cv::resize(curr, up, valPyr[i].size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat holes8u;  // <= threshold → hole
        cv::compare(wPyr[i], holeThresh, holes8u, cv::CMP_LE);  // CV_8U mask
        cv::Mat out = valPyr[i].clone();  // CV_32F
        up.copyTo(out, holes8u);          // mask must be CV_8U
        curr = out;
    }

    // Keep original valid pixels exactly
    cv::Mat result_m = curr;
    cv::Mat valid8u; cv::compare(d16_in, 0, valid8u, cv::CMP_GT);
    depth_m.copyTo(result_m, valid8u);

    // Clamp & back to mm
    cv::threshold(result_m, result_m, (float)zFar_mm / 1000.0f, (float)zFar_mm / 1000.0f, cv::THRESH_TRUNC);
    cv::threshold(result_m, result_m, (float)zNear_mm / 1000.0f, 0.0f, cv::THRESH_TOZERO);

    cv::Mat out16; result_m.convertTo(out16, CV_16U, 1000.0);
    return out16;
}

// --- Edge-aware, small-hole-only fill: writes *into* d16 (CV_16UC1 mm).
//    - maxHoleArea: only fill connected zero regions up to this many pixels
//    - maxDz: reject fills if push-pull estimate differs > maxDz from 3x3 neighborhood range
//    - keep a 1px invalid ring around silhouettes
//    - addNoiseMM: optional tiny noise to accepted fills (helps tracker stats)
static int conservativeFillDepth16InPlace(cv::Mat& d16,
    uint16_t zNear_mm, uint16_t zFar_mm,
    int maxHoleArea = 512, int maxDz = 80,
    bool addNoise = false, int addNoiseMM = 2)
{
    CV_Assert(!d16.empty() && d16.type() == CV_16UC1);

    // 1) Push–pull estimate (CV_16U mm). Does NOT alter d16.
    cv::Mat estimate = pushPullFillDepth(d16, zNear_mm, zFar_mm);

   
    cv::Mat valid8u;  cv::compare(d16, 0, valid8u, cv::CMP_GT);   // valid (CV_8U)
    cv::Mat holes8u;  cv::compare(d16, 0, holes8u, cv::CMP_EQ);   // holes (CV_8U)

    // 3) One-pixel ring (silhouette)
    //cv::Mat dilValid; cv::dilate(valid8u, dilValid, cv::getStructuringElement(cv::MORPH_RECT, { 3,3 }));
    //cv::Mat ring = dilValid & (~valid8u);  // 1-px band next to valid depth

    // --- NEW: hole core (strip the ring away up front with a single erosion) ---
    cv::Mat holeCore;
    cv::erode(holes8u, holeCore, cv::getStructuringElement(cv::MORPH_RECT, { 3,3 }));
    // If erosion nuked everything (pure 1-px holes), keep original holes as fallback later
    bool haveCore = cv::countNonZero(holeCore) > 0;

    // 4) Small hole selection (connected components) — first try CORE, then fallback to ALL holes
    auto smallHoleMaskFrom = [&](const cv::Mat& holesMask) {
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(holesMask, labels, stats, centroids, 8, CV_32S);
        cv::Mat small = cv::Mat::zeros(holesMask.size(), CV_8U);
        for (int i = 1; i < n; ++i) { // 0=background
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area <= maxHoleArea) {
                small.setTo(255, (labels == i));
            }
        }
        return small;
        };

    cv::Mat smallHoles = smallHoleMaskFrom(haveCore ? holeCore : holes8u);

    // 5) Neighborhood bounds (3×3) for edge-aware gating
    cv::Mat dMin = d16.clone();
    dMin.setTo(65535, valid8u == 0);
    cv::erode(dMin, dMin, cv::getStructuringElement(cv::MORPH_RECT, { 3,3 }));
    dMin.setTo(0, dMin == 65535);

    cv::Mat dMax = d16.clone();
    dMax.setTo(0, valid8u == 0);
    cv::dilate(dMax, dMax, cv::getStructuringElement(cv::MORPH_RECT, { 3,3 }));

    cv::Mat est32s, dMin32s, dMax32s;
    estimate.convertTo(est32s, CV_32S);
    dMin.convertTo(dMin32s, CV_32S);
    dMax.convertTo(dMax32s, CV_32S);

    cv::Mat lower = dMin32s - maxDz;
    cv::Mat upper = dMax32s + maxDz;

    cv::Mat geLB, leUB;
    cv::compare(est32s, lower, geLB, cv::CMP_GE);
    cv::compare(est32s, upper, leUB, cv::CMP_LE);

    cv::Mat within; cv::bitwise_and(geLB, leUB, within);

    // 6) Acceptance mask
    //    First try: fill only small hole *cores* (or small holes if no core) that are within range.
    cv::Mat accept = smallHoles & within;

    // If nothing survives (likely all cores sit on ring), relax the ring restriction for this frame.
    // We still never overwrite original non-zero depth.
    int accepted = cv::countNonZero(accept);
    if (accepted == 0) {
        // Try small (non-core) holes without subtracting ring
        if (haveCore) {
            cv::Mat smallAll = smallHoleMaskFrom(holes8u);
            accept = smallAll & within;
            accepted = cv::countNonZero(accept);
        }
        // As a final fallback, allow all holes that are within range (still respects maxDz)
        if (accepted == 0) {
            accept = holes8u & within;
            accepted = cv::countNonZero(accept);
        }
    }
    else {
        // If we had something already, keep a thin safety ring OFF to preserve crisp edges
      //  accept.setTo(0, ring);
        accepted = cv::countNonZero(accept);
        // If that killed everything, restore pre-ring accept
        if (accepted == 0) {
            accept = smallHoles & within; // ring back on
            accepted = cv::countNonZero(accept);
        }
    }

    // (Optional) tiny dilation to feather a bit after acceptance, but only inside original holes
    if (accepted > 0) {
        cv::Mat accDil;
        cv::dilate(accept, accDil, cv::getStructuringElement(cv::MORPH_RECT, { 3,3 }));
        accept = accDil & holes8u; // still never touch original valid pixels
        accepted = cv::countNonZero(accept);
    }

    // 7) Write fills
    if (accepted > 0) {
        if (addNoise) {
            cv::Mat noise(d16.size(), CV_16S);
            cv::randn(noise, 0, addNoiseMM);
            cv::Mat tmp;
            cv::add(estimate, noise, tmp, accept, CV_16U);
            tmp.copyTo(d16, accept);
        }
        else {
            estimate.copyTo(d16, accept);
        }
    }

    // --- debug ---
    auto nz = [](const cv::Mat& m) { return cv::countNonZero(m); };
    int zerosBefore = int(d16.total() - nz(valid8u));
    int nHoles = nz(holes8u);
 //   int nRing = nz(ring);
    int nCore = haveCore ? nz(holeCore) : 0;
    int nSmall = nz(smallHoles);
    int nWithin = nz(within);
    cv::Mat estOnHoles; estimate.copyTo(estOnHoles, holes8u);
    int nEstHoles = nz(estOnHoles);

    LOG_INFO("[fill] zerosBefore=%d holes=%d  core = % d small = % d within = % d accept = % d estOnHolesNZ = % d",
        zerosBefore, nHoles /* , nRing */, nCore, nSmall, nWithin, accepted, nEstHoles);

    // visualize (debug)
 //   cv::Mat dbg; cv::cvtColor(d16 == 0, dbg, cv::COLOR_GRAY2BGR);
  //  dbg.setTo(cv::Scalar(0, 255, 255), ring);           // ring = yellow
  //  dbg.setTo(cv::Scalar(0, 0, 255), smallHoles);       // small holes = red
   // dbg.setTo(cv::Scalar(0, 255, 0), accept);           // accepted = green
  //  cv::imshow("fill debug", dbg); cv::waitKey(1);



    return accepted;
}


//old colorising method, works well with lambert shading
static cv::Mat colorize_ir16_for_display(const k4a::image& ir,
    uint16_t irMin = 300, uint16_t irMax = 4500)
{
    CV_Assert(ir && ir.get_format() == K4A_IMAGE_FORMAT_IR16);
    const int w = ir.get_width_pixels();
    const int h = ir.get_height_pixels();
    cv::Mat ir16(h, w, CV_16UC1, (void*)ir.get_buffer(), ir.get_stride_bytes());

    // Clamp to [irMin, irMax], map to 0..255; zeros stay black
    cv::Mat clamped = ir16.clone();
    for (int y = 0; y < h; ++y) {
        uint16_t* row = clamped.ptr<uint16_t>(y);
        for (int x = 0; x < w; ++x) {
            const uint16_t v = row[x];
            if (v == 0 || v < irMin || v > irMax) row[x] = 0;
        }
    }
    cv::Mat u8;
    const double scale = 255.0 / std::max<int>(1, irMax - irMin);
    const double shift = -255.0 * irMin / std::max<int>(1, irMax - irMin);
    clamped.convertTo(u8, CV_8U, scale, shift);
    u8.setTo(0, clamped == 0);

    cv::Mat bgr;
    cv::applyColorMap(u8, bgr, cv::COLORMAP_JET); // or COLORMAP_BONE if you prefer
    return bgr; // independent copy
}

static void median3_depth16(cv::Mat& d16)
{
    CV_Assert(d16.type() == CV_16UC1);
    cv::medianBlur(d16, d16, 3);
}


// not working properly
static cv::Mat pushPullFillDepth16(const cv::Mat& d16_in,
    uint16_t zNear_mm, uint16_t zFar_mm,
    int levels = -1)
{
    CV_Assert(d16_in.type() == CV_16UC1);
    const int W = d16_in.cols, H = d16_in.rows;

    // Levels: coarsest ~16–32 px – good rule of thumb
    if (levels <= 0) {
        int m = std::min(W, H);
        levels = std::max(3, (int)std::floor(std::log2((double)m)) - 1);
        levels = std::min(levels, 9);
    }

    // Work in meters (float)
    cv::Mat depth_m; d16_in.convertTo(depth_m, CV_32F, 1.0 / 1000.0);

    // Validity mask (float 0/1). Treat 0 and out-of-range as invalid.
    cv::Mat valid01 = (d16_in >= zNear_mm) & (d16_in <= zFar_mm);
    valid01.convertTo(valid01, CV_32F, 1.0 / 255.0);

    // Numerator and weight at base
    const float eps = 1e-8f;
    cv::Mat num0 = depth_m.mul(valid01); // value * weight

    // --- PUSH: build pyramids of numerator and weight (area averaging) ---
    std::vector<cv::Mat> numPyr, wPyr, valPyr;
    numPyr.reserve(levels); wPyr.reserve(levels); valPyr.reserve(levels);
    numPyr.push_back(num0); wPyr.push_back(valid01);

    for (int i = 1; i < levels; ++i) {
        cv::Size s(std::max(1, numPyr.back().cols / 2), std::max(1, numPyr.back().rows / 2));
        cv::Mat nC, wC;
        cv::resize(numPyr.back(), nC, s, 0, 0, cv::INTER_AREA);
        cv::resize(wPyr.back(), wC, s, 0, 0, cv::INTER_AREA);
        numPyr.push_back(nC); wPyr.push_back(wC);
    }

    // Coarse values = numerator / weight
    for (int i = 0; i < levels; ++i) {
        cv::Mat wSafe; cv::max(wPyr[i], eps, wSafe);
        valPyr.push_back(numPyr[i] / wSafe);
    }

    // --- PULL: upsample and fill only where invalid (linear upsampling) ---
    cv::Mat curr = valPyr.back().clone();
    for (int i = levels - 2; i >= 0; --i) {
        cv::Mat up; cv::resize(curr, up, valPyr[i].size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat out = valPyr[i].clone();
        cv::Mat holes; cv::compare(wPyr[i], 0.0, holes, cv::CMP_EQ);
        up.copyTo(out, holes);   // fill holes only
        curr = out;
    }

    // Keep original valid pixels exactly
    cv::Mat result_m = curr.clone();
    cv::Mat keepMask; cv::compare(valid01, 0.0, keepMask, cv::CMP_GT);
    depth_m.copyTo(result_m, keepMask);

    // Clamp to range and invalidate non-positive just in case
    cv::threshold(result_m, result_m, (float)zFar_mm / 1000.0f, (float)zFar_mm / 1000.0f, cv::THRESH_TRUNC);
    cv::threshold(result_m, result_m, (float)zNear_mm / 1000.0f, (float)zNear_mm / 1000.0f, cv::THRESH_TOZERO);

    // Back to DEPTH16 mm
    cv::Mat out16; result_m.convertTo(out16, CV_16U, 1000.0);
    // Ensure holes stayed holes where input was invalid (kept originals already, but be explicit)
    out16.setTo(0, (keepMask == 0));
    return out16;
}


static cv::Mat colorize_depth16_for_display(const k4a::image& depth, uint16_t zMin = 100, uint16_t zMax = 8000)
{
    const int w = depth.get_width_pixels();
    const int h = depth.get_height_pixels();
    cv::Mat dmm(h, w, CV_16UC1, (void*)depth.get_buffer(), depth.get_stride_bytes());

    // Clamp to [zMin,zMax], map to 0..255; zeros stay zero (black)
    cv::Mat clamped = dmm.clone();
    for (int y = 0; y < h; ++y) {
        uint16_t* row = clamped.ptr<uint16_t>(y);
        for (int x = 0; x < w; ++x) {
            uint16_t z = row[x];
            if (z == 0 || z < zMin || z > zMax) row[x] = 0;
        }
    }
    cv::Mat u8;
    clamped.convertTo(u8, CV_8U, 255.0 / std::max(1, int(zMax - zMin)), -255.0 * zMin / std::max(1, int(zMax - zMin)));
    u8.setTo(0, clamped == 0);

    cv::Mat bgr;
    cv::applyColorMap(u8, bgr, cv::COLORMAP_JET);
    return bgr; // independent copy
}


static k4a::image rasterize_to_depth16_mm(const std::vector<std::array<float, 3>>& xyz_mm,
    int width, int height,
    float fx, float fy, float cx, float cy,
    uint16_t z_min_mm = 300, uint16_t z_max_mm = 8000)
{
    // Allocate depth image
    k4a::image depth = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * int(sizeof(uint16_t)));
    uint16_t* d = reinterpret_cast<uint16_t*>(depth.get_buffer());
    const size_t px = size_t(width) * size_t(height);
    std::fill(d, d + px, uint16_t(0));

    // Z-buffer in mm (store nearest)
    for (const auto& p : xyz_mm) {
        const float X = p[0], Y = p[1], Z = p[2];
        if (!(Z > 0.0f)) continue; // behind camera or invalid
        if (Z < z_min_mm || Z > z_max_mm) continue;

        const float u = fx * (X / Z) + cx;
        const float v = fy * (Y / Z) + cy;
        const int x = int(std::lround(u));
        const int y = int(std::lround(v));
        if (unsigned(x) >= unsigned(width) || unsigned(y) >= unsigned(height)) continue;

        const uint16_t zmm = (uint16_t)std::lround(Z);
        uint16_t& dst = d[size_t(y) * width + x];
        if (dst == 0 || zmm < dst) dst = zmm; // keep nearest
    }

    return depth;
}
// bones according to the k4abt template
static const std::vector<std::pair<int, int>> bone_connections = {
   {0,1},{1,2},{2,3},{3,4},{3,11},{3,26},
   {4,5},{5,6},{6,7},{7,8},{8,9},{7,10},
   {11,12},{12,13},{13,14},{14,15},{15,16},{14,17},
   {0,18},{18,19},{19,20},{20,21},
   {0,22},{22,23},{23,24},{24,25},
   {27,28},{27,29},{27,30},{27,31}
};

// Lambertian IR from DEPTH16 using estimated normals in camera space.
// - Uses Azure Kinect depth intrinsics (fx,fy,cx,cy) from 'calibration' to lift pixels to 3D.
// - Normal = normalize( (P(x+1,y)-P) x (P(x,y+1)-P) ).
// - We flip normals to face the camera (negative Z), then lambert = max(0, -n.z).
// - IR16 is written in [irMin, irMax], 0 for invalid pixels.
// - Depth range is clamped to [min_mm, max_mm].
static k4a::image make_lambert_ir_from_depth(const k4a::image& depth,
    const k4a::calibration& calibration,
    uint16_t min_mm = 300,
    uint16_t max_mm = 8000,
    uint16_t irMin = 300,
    uint16_t irMax = 4500)
{
    if (!depth) return {};

    const int w = depth.get_width_pixels();
    const int h = depth.get_height_pixels();
    const size_t px = size_t(w) * size_t(h);

    const uint16_t* dz = reinterpret_cast<const uint16_t*>(depth.get_buffer());

    const auto& cam = calibration.depth_camera_calibration;
    const float fx = static_cast<float>(cam.intrinsics.parameters.param.fx);
    const float fy = static_cast<float>(cam.intrinsics.parameters.param.fy);
    const float cx = static_cast<float>(cam.intrinsics.parameters.param.cx);
    const float cy = static_cast<float>(cam.intrinsics.parameters.param.cy);


    k4a::image ir = k4a::image::create(K4A_IMAGE_FORMAT_IR16, w, h, w * int(sizeof(uint16_t)));
    uint16_t* irBuf = reinterpret_cast<uint16_t*>(ir.get_buffer());

    auto depthAt = [&](int x, int y) -> uint16_t {
        if (unsigned(x) >= unsigned(w) || unsigned(y) >= unsigned(h)) return 0;
        return dz[size_t(y) * w + x];
        };


    std::fill(irBuf, irBuf + px, uint16_t(0));


    for (int y = 0; y < h - 1; ++y) {
        for (int x = 0; x < w - 1; ++x) {
            const uint16_t zc_mm = depthAt(x, y);
            const uint16_t zr_mm = depthAt(x + 1, y);
            const uint16_t zd_mm = depthAt(x, y + 1);

            if (zc_mm < min_mm || zc_mm > max_mm ||
                zr_mm < min_mm || zr_mm > max_mm ||
                zd_mm < min_mm || zd_mm > max_mm) {

                continue;
            }


            const float Zc = 0.001f * zc_mm;
            const float Zr = 0.001f * zr_mm;
            const float Zd = 0.001f * zd_mm;


            const float Xc = ((float)x - cx) * Zc / fx;
            const float Yc = ((float)y - cy) * Zc / fy;

            const float Xr = ((float)(x + 1) - cx) * Zr / fx;
            const float Yr = ((float)y - cy) * Zr / fy;

            const float Xd = ((float)x - cx) * Zd / fx;
            const float Yd = ((float)(y + 1) - cy) * Zd / fy;


            const cv::Vec3f vR{ Xr - Xc, Yr - Yc, Zr - Zc };
            const cv::Vec3f vD{ Xd - Xc, Yd - Yc, Zd - Zc };


            cv::Vec3f n = vR.cross(vD);
            const float len = std::sqrt(n.dot(n));
            if (len < 1e-6f) { continue; }
            n *= (1.0f / len);
            if (n[2] > 0.0f) n = -n;


            float lambert = std::max(0.0f, -n[2]);
            const float ambient = 0.05f;
            lambert = std::clamp(lambert + ambient, 0.0f, 1.0f);

            const float irf = irMin + lambert * float(irMax - irMin);
            irBuf[size_t(y) * w + x] = static_cast<uint16_t>(irf + 0.5f);
        }
    }


    k4a_image_set_device_timestamp_usec(ir.handle(), depth.get_device_timestamp().count());

    return ir;
}


struct Intrinsics {
    double principalPointX;
    double principalPointY;
    double focalLengthX;
    double focalLengthY;
    double distortionCoefficients[6];
   
};



bool AppLogic::init(GfxContext& context)
{
    // Initialize the varjo session.
    LOG_DEBUG("Initializing Varjo session..");
    m_session = varjo_SessionInit();
    if (CHECK_VARJO_ERR(m_session) != varjo_NoError) {
        LOG_ERROR("Creating Varjo session failed.");
        return false;
    }

    {
        // Get graphics adapter used by Varjo session
        auto dxgiAdapter = D3D11MultiLayerView::getAdapter(m_session);

        // Init graphics
        context.init(dxgiAdapter.Get());

        // Create D3D11 renderer instance.
        auto d3d11Renderer = std::make_unique<D3D11Renderer>(dxgiAdapter.Get());

        // Create varjo multi layer view
        m_varjoView = std::make_unique<D3D11MultiLayerView>(m_session, *d3d11Renderer);

        // Store as generic renderer
        m_renderer = std::move(d3d11Renderer);

        // Set application Z order so that this application is on background behind other apps. Only
        // used if application runs in headless mode not intended to render anything by itself.
        varjo_SessionSetPriority(m_session, c_appOrderBg);
        CHECK_VARJO_ERR(m_session);

        // Create varjo multi layer view
       // m_varjoView = std::make_unique<HeadlessView>(m_session);
    }
    // Create scene instance
    m_scene = std::make_unique<MRScene>(*m_renderer);


    // Create data streamer instance
    m_streamer = std::make_unique<DataStreamer>(m_session, std::bind(&AppLogic::onFrameReceived, this, std::placeholders::_1));

    // Create mixed reality camera manager instance
    m_camera = std::make_unique<CameraManager>(m_session);

    //Create point cloud instance
    m_pointCloud = std::make_unique<PointCloud>(m_session, std::bind(&AppLogic::onPointCloudSnapshotReceived, this, std::placeholders::_1));

    //Create IPCConnector instance
    m_IPCConnector = std::make_unique<IPCConnectorHost>();

    // Check if Mixed Reality features are available.
    varjo_Bool mixedRealityAvailable = varjo_False;
    varjo_SyncProperties(m_session);
    CHECK_VARJO_ERR(m_session);
    if (varjo_HasProperty(m_session, varjo_PropertyKey_MRAvailable)) {
        mixedRealityAvailable = varjo_GetPropertyBool(m_session, varjo_PropertyKey_MRAvailable);
    }

    // Handle mixed reality availability
    onMixedRealityAvailable(mixedRealityAvailable == varjo_True, false);

    //Azure Kinect
    // the recording to get the calibration and serial number of a Azure DK device so that the tracker can work
    
        std::string filepath = R"(D:\Toni\toni_ba\bens_wie es ist\mixed-reality-attention-direction\varjo-sdk-experimental\build\1.mkv)";
        if (!std::filesystem::exists(filepath)) {
            std::cerr << "File not found: " << filepath << "\n";
            return -1;
        }

        try {
            if (!filepath.empty()) {
            // --------- FROM MKV RECORDING ----------
            k4a::playback playback = k4a::playback::open(filepath.c_str());

            const auto recCfg = playback.get_record_configuration();
            if (!recCfg.depth_track_enabled) {
                throw std::runtime_error("MKV has no depth track; body tracking requires depth.");
            }

            // Get calibration from the file (matches how it was recorded)
            m_k4aCalib = playback.get_calibration();
            playback.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);

            // Cache intrinsics for depth camera
            const auto& cam = m_k4aCalib.depth_camera_calibration;
            m_k4aWidth = cam.resolution_width;
            m_k4aHeight = cam.resolution_height;

            m_k4aFx = static_cast<float>(cam.intrinsics.parameters.param.fx);
            m_k4aFy = static_cast<float>(cam.intrinsics.parameters.param.fy);
            m_k4aCx = static_cast<float>(cam.intrinsics.parameters.param.cx);
            m_k4aCy = static_cast<float>(cam.intrinsics.parameters.param.cy);

            m_k4aCalibOk = true;
            LOG_INFO("K4A calib (MKV): %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                m_k4aWidth, m_k4aHeight, m_k4aFx, m_k4aFy, m_k4aCx, m_k4aCy);

            // Create body tracker (prefer GPU; fall back to CPU)
            k4abt_tracker_configuration_t tcfg = K4ABT_TRACKER_CONFIG_DEFAULT;
            tcfg.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
            tcfg.gpu_device_id = 0;
            try {
                m_tracker = k4abt::tracker::create(m_k4aCalib, tcfg);
                m_trackerOk = true;
                LOG_INFO("k4abt tracker created from MKV calib (GPU)");
            }
            catch (...) {
                LOG_WARNING("k4abt GPU create failed; trying CPU...");
                tcfg.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
                m_tracker = k4abt::tracker::create(m_k4aCalib, tcfg);
                m_trackerOk = true;
                LOG_INFO("k4abt tracker created from MKV calib (CPU)");
            }

           

            // Varjo->K4A transform 
            m_T_varjoCam_to_k4aDepth = glm::mat4(1.0f);
            m_T_varjoCam_to_k4aDepth[1][1] = -1.0f;
            m_T_varjoCam_to_k4aDepth[2][2] = -1.0f;
        }
        else {
            // --------- FROM LIVE DEVICE ----------
            if (k4a::device::get_installed_count() == 0) {
                LOG_WARNING("No Azure Kinect connected and no MKV path provided.");
                m_k4aCalibOk = false;
            }
            else {
                k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
                m_k4aCalib = dev.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_OFF);
                dev.close();

                const auto& cam = m_k4aCalib.depth_camera_calibration;
                m_k4aWidth = cam.resolution_width;
                m_k4aHeight = cam.resolution_height;
                m_k4aFx = static_cast<float>(cam.intrinsics.parameters.param.fx);
                m_k4aFy = static_cast<float>(cam.intrinsics.parameters.param.fy);
                m_k4aCx = static_cast<float>(cam.intrinsics.parameters.param.cx);
                m_k4aCy = static_cast<float>(cam.intrinsics.parameters.param.cy);

                m_k4aCalibOk = true;
                LOG_INFO("K4A calib (LIVE): %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                    m_k4aWidth, m_k4aHeight, m_k4aFx, m_k4aFy, m_k4aCx, m_k4aCy);

                k4abt_tracker_configuration_t tcfg = K4ABT_TRACKER_CONFIG_DEFAULT;
                tcfg.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
                tcfg.gpu_device_id = 0;
                try {
                    m_tracker = k4abt::tracker::create(m_k4aCalib, tcfg);
                    m_trackerOk = true;
                    LOG_INFO("k4abt tracker created (GPU)");
                }
                catch (...) {
                    LOG_WARNING("k4abt GPU create failed; trying CPU...");
                    tcfg.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
                    m_tracker = k4abt::tracker::create(m_k4aCalib, tcfg);
                    m_trackerOk = true;
                    LOG_INFO("k4abt tracker created (CPU)");
                }

               

                m_T_varjoCam_to_k4aDepth = glm::mat4(1.0f);
                m_T_varjoCam_to_k4aDepth[1][1] = -1.0f;
                m_T_varjoCam_to_k4aDepth[2][2] = -1.0f;
            }
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("K4A init/tracker error: %s", e.what());
        m_k4aCalibOk = false;
        m_trackerOk = false;
    }

    m_initialized = true;



    return true;
}

void AppLogic::setVideoRendering(bool enabled)
{
    varjo_MRSetVideoRender(m_session, enabled ? varjo_True : varjo_False);
    if (CHECK_VARJO_ERR(m_session) == varjo_NoError) {
        LOG_INFO("Video rendering: %s", enabled ? "ON" : "OFF");
    }
    m_appState.options.videoRenderingEnabled = enabled;
}

void AppLogic::setState(const AppState& appState, bool force)
{
    // Store previous state and set new one
    const auto prevState = m_appState;
    m_appState = appState;

    // Check for mixed reality availability
    if (!m_appState.general.mrAvailable) {
        // Toggle video rendering off
        if (m_appState.options.videoRenderingEnabled) {
            setVideoRendering(false);
        }
        return;
    }

    // React MR events
    if (force || appState.options.reactToConnectionEvents != prevState.options.reactToConnectionEvents) {
        LOG_INFO("Handling connection events: %s", appState.options.reactToConnectionEvents ? "ON" : "OFF");
    }

    // Data stream buffer handling
    if (force || appState.options.delayedBufferHandlingEnabled != prevState.options.delayedBufferHandlingEnabled) {
        m_streamer->setDelayedBufferHandlingEnabled(appState.options.delayedBufferHandlingEnabled);
        LOG_INFO("Buffer handling: %s", appState.options.delayedBufferHandlingEnabled ? "DELAYED" : "IMMEDIATE");
    }

    // Data stream: YUV
    if (force || appState.options.dataStreamColorEnabled != prevState.options.dataStreamColorEnabled) {
        const auto streamType = varjo_StreamType_DistortedColor;
        const auto streamFormat = m_colorStreamFormat;
        const auto streamChannels = varjo_ChannelFlag_Left | varjo_ChannelFlag_Right;
        varjo_ChannelFlag currentChannels = varjo_ChannelFlag_None;

        if (appState.options.dataStreamColorEnabled) {
            if (m_streamer->isStreaming(streamType, streamFormat, currentChannels)) {
                if (currentChannels == streamChannels) {
                    // If running stream channels match, match we want to stop it
                    //m_streamer->stopDataStream(streamType, streamFormat);
                    //do nothing
                }
                else {
                    // When color correction is enabled, stop stream and start new stream with data channels
                    //should never be the case in this implementation
                    LOG_INFO("Switching to color stream with data channels..");
                    m_streamer->stopDataStream(streamType, streamFormat);
                    m_streamer->startDataStream(streamType, streamFormat, streamChannels);
                }
            }
            else {
                // No streams running, just start our color data stream
                m_streamer->startDataStream(streamType, streamFormat, streamChannels);
            }

            //start frame export
            m_IPCConnector.get()->SetVSTFrameExport(true);

        }
        else {
            if (m_streamer->isStreaming(streamType, streamFormat, currentChannels)) {
                if (currentChannels == streamChannels) {
                    // If running stream channels match, match we want to stop it
                    m_streamer->stopDataStream(streamType, streamFormat);
                }
                else {
                    m_streamer->stopDataStream(streamType, streamFormat);
                }
            }
            else {
                // No streams running, just ignore
            }

            // Reset textures if disabled
            {
                std::lock_guard<std::mutex> streamLock(m_frameDataMutex);
                m_frameData.colorFrames = {};
            }
            //stop frame export
            m_IPCConnector.get()->SetVSTFrameExport(false);
        }

        // Write stream status back to state
        m_appState.options.dataStreamColorEnabled = m_streamer->isStreaming(streamType, streamFormat);
    }
}

void AppLogic::onFrameReceived(const DataStreamer::Frame& frame)
{
    const auto& streamFrame = frame.metadata.streamFrame;

    std::lock_guard<std::mutex> streamLock(m_frameDataMutex);
    switch (streamFrame.type) {
    case varjo_StreamType_DistortedColor: {
        // We update color stream metadata from first channel only
        if (frame.metadata.channelIndex == varjo_ChannelIndex_First) {
            m_frameData.metadata = streamFrame.metadata.distortedColor;
        }
        m_frameData.colorFrames[static_cast<size_t>(frame.metadata.channelIndex)] = frame;
    } break;
    case varjo_StreamType_EnvironmentCubemap: {
        // Do nothing since we do not need the cubemap frame
    } break;
    default: {
        LOG_ERROR("Unsupported stream type: %d", streamFrame.type);
        assert(false);
    } break;
    }
}

void AppLogic::onPointCloudSnapshotReceived(const varjo_PointCloudSnapshotContent& content)
{
    std::lock_guard<std::mutex> streamLock(m_pointCloudSnapshotContentMutex);

    if (content.pointCount > 0) {
        m_pointCloudSnapshotContent = content;
        validSnapshot = true;
    }
}

void AppLogic::processFrame()
{
    // Get latest frame data
    FrameData frameData{};
    {
        std::lock_guard<std::mutex> streamLock(m_frameDataMutex);

        frameData.metadata = m_frameData.metadata;

        // Move frame datas
       
        frameData.cubemapFrame = std::move(m_frameData.cubemapFrame);
        m_frameData.cubemapFrame = std::nullopt;
    }
    bool showFrameProcessingPictures = true;
    bool saveFrameProcessingPictures = false;
    processingFrame = false;
}

void AppLogic::update()
{
    // Check for new mixed reality events
    checkEvents();

    // Handle delayed data stream buffers
    m_streamer->handleDelayedBuffers();

    // Sync frame
    m_varjoView->syncFrame();

    // Update frame time
    m_appState.general.frameTime += m_varjoView->getDeltaTime();
    m_appState.general.frameCount = m_varjoView->getFrameNumber();

    //only process new frame, if previous processing is done
    if (!processingFrame) {
        if (processFrameThread.joinable()) {
            processFrameThread.join();
        }
        processingFrame = true;
        processFrameThread = std::thread(&AppLogic::processFrame, this);
    }
    FrameData frameData{};
    {
        std::lock_guard<std::mutex> streamLock(m_frameDataMutex);

        frameData.metadata = m_frameData.metadata;

        // Move frame datas
        for (size_t ch = 0; ch < m_frameData.colorFrames.size(); ch++) {
            frameData.colorFrames[ch] = std::move(m_frameData.colorFrames[ch]);
            m_frameData.colorFrames[ch] = std::nullopt;
        }

        frameData.cubemapFrame = std::move(m_frameData.cubemapFrame);
        m_frameData.cubemapFrame = std::nullopt;

        frameData.cubemapMetadata = m_frameData.cubemapMetadata;
    }

    // Update scene
    MRScene::UpdateParams updateParams{};

    // Get latest color adjustments. These will take effect only if cubemap is not using auto adaptation.
  
    m_scene->update(m_varjoView->getFrameTime(), m_varjoView->getDeltaTime(), m_varjoView->getFrameNumber(), updateParams);

    // Get latest cubemap frame.
    if (frameData.cubemapFrame.has_value()) {
        const auto& cubemapFrame = frameData.cubemapFrame.value();
        m_scene->updateHdrCubemap(cubemapFrame.metadata.bufferMetadata.width, cubemapFrame.metadata.bufferMetadata.format,
            cubemapFrame.metadata.bufferMetadata.rowStride, cubemapFrame.data.data());
    }

    // Get latest color frame
    for (size_t ch = 0; ch < frameData.colorFrames.size(); ch++) {
        if (!frameData.colorFrames[ch].has_value()) continue;

        const auto& colorFrame = frameData.colorFrames[ch].value();

        if (colorFrame.metadata.bufferMetadata.byteSize == 0) continue;


        if (m_appState.options.undistortEnabled) {
            // We can downsample here to any resolution
            constexpr int c_downScaleFactor = 4;
            const auto w = colorFrame.metadata.bufferMetadata.width; // / c_downScaleFactor;
            const auto h = colorFrame.metadata.bufferMetadata.height; // / c_downScaleFactor;
            const auto rowStride = w * 4;

            // Projection for rectified image. In case no projection matrix is provided, uniform projection will be used.
            std::optional<varjo_Matrix> projection = m_varjoView->getLayer(0).getProjection(static_cast<int>(ch));
            // std::optional<varjo_Matrix> projection = std::nullopt;

            // Convert to rectified RGBA in lower resolution
            std::vector<uint8_t> bufferRGBA(rowStride * h);
            DataStreamer::convertDistortedYUVToRectifiedRGBA(colorFrame.metadata.bufferMetadata, colorFrame.data.data(), glm::ivec2(w, h),
                bufferRGBA.data(), colorFrame.metadata.extrinsics, colorFrame.metadata.intrinsics, projection);

            cv::Mat bgra(h, w, CV_8UC4, bufferRGBA.data(), w * 4);
            m_leftBGRA = bgra.clone();

            // Update frame data
            m_scene->updateColorFrame(static_cast<int>(ch), glm::ivec2(w, h), varjo_TextureFormat_R8G8B8A8_UNORM, rowStride, bufferRGBA.data());
        }
        else {
            const auto w = colorFrame.metadata.bufferMetadata.width;
            const auto h = colorFrame.metadata.bufferMetadata.height;
            const auto rowStride = w * 4;

            // Convert to RGBA in full res (this conversion is pixel to pixel, no scaling allowed)
            std::vector<uint8_t> bufferRGBA(rowStride * h);
            DataStreamer::convertToR8G8B8A(colorFrame.metadata.bufferMetadata, colorFrame.data.data(), bufferRGBA.data());

            cv::Mat bgra(h, w, CV_8UC4, bufferRGBA.data(), w * 4);
            m_leftBGRA = bgra.clone();

            // Update frame data
            m_scene->updateColorFrame(static_cast<int>(ch), glm::ivec2(w, h), varjo_TextureFormat_R8G8B8A8_UNORM, rowStride, bufferRGBA.data());
        }

        {
            std::lock_guard<std::mutex> streamLock(m_frameDataMutex);
            m_frameData.colorFrames[ch] = std::nullopt;
        }
    }



    // Begin frame
    m_varjoView->beginFrame();

    // Render layer
    {
        // Get layer for rendering
        constexpr int layerIndex = 0;
        auto& layer = m_varjoView->getLayer(layerIndex);

        // Setup render params
        MultiLayerView::Layer::SubmitParams submitParams{};
        submitParams.submitColor = m_appState.options.renderVREnabled;
        submitParams.submitDepth = m_appState.options.submitVRDepthEnabled;
        submitParams.depthTestEnabled = m_appState.options.VideoDepthEstimationEnabled;
        submitParams.depthTestRangeEnabled = m_appState.options.vrDepthTestRangeEnabled;
        submitParams.depthTestRangeLimits = {
            0.0, submitParams.depthTestRangeEnabled ? std::max(static_cast<double>(m_appState.options.vrDepthTestRangeValue), 0.0) : 0.0 };
        submitParams.chromaKeyEnabled = m_appState.options.chromaKeyingEnabled;
        submitParams.alphaBlend = m_appState.options.videoRenderingEnabled || !m_appState.options.drawVRBackgroundEnabled;

        // Begin layer rendering
        layer.begin(submitParams);

        // Clear frame
        const glm::vec4 opaqueBg(0.15f, 0.5f, 0.6f, 1.0f);
        const glm::vec4 transpBg(0.0f, 0.0f, 0.0f, 0.0f);
        const glm::vec4 clearColor = submitParams.alphaBlend ? transpBg : opaqueBg;
        layer.clear(MultiLayerView::Layer::ClearParams(clearColor));

        // Render frame
        layer.renderScene(*m_scene);

        // End layer rendering
        layer.end();
    }

    // Submit varjo frame with rendered layers
    m_varjoView->endFrame();

    //update pointcloud
    std::lock_guard<std::mutex> streamLock(m_pointCloudSnapshotContentMutex);
    if (validSnapshot) {
        m_IPCConnector.get()->updatePointCloudSnapshotContent(m_pointCloudSnapshotContent);
        validSnapshot = false;
    }

    if (m_pointCloud) {
        std::vector<std::array<float, 3>> xyz_mm;
        // range (meters) for filtering
        const float zMin_m = 0.1f, zMax_m = 6.0f;
        
        //LOG_INFO("Trying Varjo->K4A extract...");

        if (m_pointCloud->extractXYZmmForK4A(xyz_mm, m_T_varjoCam_to_k4aDepth, zMin_m, zMax_m)) {
           
            // Bail if K4A calib or tracker aren't ready
            if (!m_k4aCalibOk || !m_trackerOk) {
                // nothing to do this frame
                LOG_INFO(" Calibration or tracker no set properly ")
            }
            else {




//                k4a::image depth = rasterize_with_cv_projectPoints(
  //                  xyz_mm,
    //                m_k4aWidth, m_k4aHeight,
      //              m_k4aFx, m_k4aFy, m_k4aCx, m_k4aCy,
        //            /*rvec*/ cv::Vec3d(0, 0, 0),
          //          /*tvec*/ cv::Vec3d(0, 0, 0),
            //        /*distCoeffs*/ cv::Mat(), 
              //        /*z_min_mm*/ 100, /*z_max_mm*/ 6000);
               
                k4a::image depth = rasterize_to_depth16_mm(
                    xyz_mm,
                   m_k4aWidth, m_k4aHeight,
            
                   m_k4aFx,
                   m_k4aFy, 
                   m_k4aCx, 
                   m_k4aCy, 
                   
                    /*z_min_mm*/ 100, /*z_max_mm*/ 6000);


                //result alway for body tracker 0
            //    k4a::image depth = rasterize_to_depth16_mm_splat(xyz_mm, m_k4aWidth, m_k4aHeight, m_k4aFx, m_k4aFy, m_k4aCx, m_k4aCy);
                if (depth) {
                    
                    // push-pull interpolation 28.09
                    cv::Mat d16(m_k4aHeight, m_k4aWidth, CV_16UC1, (void*)depth.get_buffer(), depth.get_stride_bytes());
                    if (!d16.empty()) {
                        
                        median3_depth16(d16);

                       // pushPullFillDepth16(d16, 100, 6000);

                     
                        int holesBefore = cv::countNonZero(d16 == 0);
                        int filledCount = conservativeFillDepth16InPlace(d16, 100, 6000, 400000, 400000, false, 2);
                        int holesAfter = cv::countNonZero(d16 == 0);
                        std::cout << "[holes] before=" << holesBefore
                            << " filled=" << filledCount
                            << " after=" << holesAfter << "\n";
                         
                    }

                    // 3) Synthetic IR from depth
              //  k4a::image ir = make_lambert_ir_from_depth(
              //      depth, m_k4aCalib,
              //      /*min_mm*/ 100, /*max_mm*/ 6000,
              //      /*irMin*/ 30, /*irMax*/ 3500);
                   
                    k4a::image ir = makeFakeIR(m_leftBGRA, m_k4aWidth, m_k4aHeight, 30, 4000, 0.8f, -80, -30);

                   
             // 4) Feed K4A
               
                k4a::capture cap = k4a::capture::create();
                    
                LOG_INFO("capture created");
               size_t frameNr = 0;
                cap.set_depth_image(depth);
                cap.set_ir_image(ir);

                for (;;) {
                    
                    const auto wr = m_tracker.enqueue_capture(cap, std::chrono::milliseconds(5));
                    if (wr == K4A_WAIT_RESULT_SUCCEEDED) break;
                    if (wr == K4A_WAIT_RESULT_TIMEOUT) {
                        // queue full -> pop one result quickly and retry
                        try { (void)m_tracker.pop_result(std::chrono::milliseconds(20)); }
                        catch (...) {}
                        continue;
                    }
                        // FAILED -> give up this frame
                    break;
                }

                    // 5) Non-blocking pop 
                try {
                    k4abt::frame bf = m_tracker.pop_result(std::chrono::milliseconds(20));

                    if (bf) {
                        // number of bodies this frame
                        uint32_t nb = bf.get_num_bodies();
                        LOG_INFO("Frame number=%u bodies=%u", (unsigned)m_frameIdxBT, (unsigned)nb);

                        // --- coordinate transforms ---
                        // k4a depth -> varjo cam 
                        glm::mat4 T_k4a_to_varjoCam = glm::inverse(m_T_varjoCam_to_k4aDepth);

                        // world <- varjo cam
                        // for now, identity keeps skeleton in VarjoCam space
                        glm::mat4 T_world_from_varjoCam = glm::mat4(1.0f);

                        auto toWorld = [&](const k4abt_joint_t& j)->glm::vec3 {
                            // Kinect joint position is in millimeters, K4A depth camera coords
                            glm::vec4 p_k4a_mm(j.position.v[0], j.position.v[1], j.position.v[2], 1.0f);
                            glm::vec4 p_k4a_m = p_k4a_mm * (1.0f / 1000.0f);        // mm -> m
                            glm::vec4 p_varjo = T_k4a_to_varjoCam * p_k4a_m;        // into Varjo cam coords
                            glm::vec4 p_world = T_world_from_varjoCam * p_varjo;    // into world
                            return glm::vec3(p_world);
                            };

                        auto confToAlpha = [](k4abt_joint_confidence_level_t c)->float {
                            // map NONE..HIGH (0..3) to [0.25 .. 1.0]
                            const float base = 0.25f;
                            return base + (float(c) / 3.0f) * (1.0f - base);
                            };

                        std::vector<MRScene::Segment> segs;
                        segs.reserve(bone_connections.size() * nb);

                        for (uint32_t b = 0; b < nb; ++b) {
                            k4abt_body_t body = bf.get_body(b);

                            for (auto e : bone_connections) {
                                const auto& j1 = body.skeleton.joints[e.first];
                                const auto& j2 = body.skeleton.joints[e.second];
                                if (j1.confidence_level <= K4ABT_JOINT_CONFIDENCE_NONE ||
                                    j2.confidence_level <= K4ABT_JOINT_CONFIDENCE_NONE)
                                    continue;

                                MRScene::Segment s;
                                s.a = toWorld(j1);
                                s.b = toWorld(j2);
                                s.radius = 0.03f; // ~3cm tubes
                                s.alpha = 0.5f * confToAlpha(j1.confidence_level)
                                    + 0.5f * confToAlpha(j2.confidence_level);
                                segs.push_back(s);
                            }
                        }

                        // hand off to the scene for this frame
                        m_scene->setSkeletonSegments(std::move(segs));
                    }
                }

                    catch (...) {
                        // no result available yet — fine
                    }

                    // --- Build the 7 panels (3 | 1 | 3) ---
                    
                    
                    cv::Mat depthFrame = colorize_depth16_for_display(depth, 100, 4000);
                    cv::Mat irFrame = visualize_fakeIR(ir);

                    m_depthPanel = depthFrame;   
                    m_irPanel = irFrame;
                   // m_colorPanel = m_frameData;

                    
                    m_cloudPanel = Visualization::placeholder("Point Cloud rendetion", { 800, 800 });

                    // Compose the wall
                    std::array<cv::Mat, 7> panels;
                    panels[4] = m_depthPanel;                                  
                    panels[5] = m_irPanel;                                      
                    panels[6] = m_colorPanel;                                    
                    panels[3] = m_cloudPanel;                                   
                    panels[0] = Visualization::placeholder("Kinect Depth", { 1,1 });        
                    panels[1] = Visualization::placeholder("Kinect IR", { 1,1 }); 
                    panels[2] = Visualization::placeholder("Kinect tracker", { 1,1 });               

                    m_viz.show(panels);

                     ++m_frameIdxBT;
                }
            }        
        }
    }
}

void AppLogic::onMixedRealityAvailable(bool available, bool forceSetState)
{
    m_appState.general.mrAvailable = available;

    if (available) {
        // Update stuff here if needed

        // Get format for color stream
        m_colorStreamFormat = m_streamer->getFormat(varjo_StreamType_DistortedColor);

        if (m_appState.options.reactToConnectionEvents && !m_appState.options.videoRenderingEnabled) {
            LOG_INFO("Enabling video rendering on MR available event..");
            setVideoRendering(true);
        }
        m_appState.options.dataStreamColorEnabled = true;   // start DistortedColor stream
        m_appState.options.renderVREnabled = true;          // allow the layer to submit color
        setState(m_appState, /*force=*/true);

    }
    else {
        LOG_ERROR("Mixed Reality features not available.");

        // Update stuff here if needed
        if (m_appState.options.reactToConnectionEvents && m_appState.options.videoRenderingEnabled) {
            LOG_INFO("Disabling video rendering on MR unavailable event..");
            setVideoRendering(false);
        }

    }
    varjo_MRSetVideoRender(m_session, varjo_True);

    //update point cloud status
    m_pointCloud.get()->onMixedRealityAvailable(available);
   // m_IPCConnector.get()->SetPointCloudDataExport(available);

    // Force set state when MR becomes active
    if (forceSetState) {
        setState(m_appState, true);
    }
}

void AppLogic::checkEvents()
{
    varjo_Bool ret = varjo_False;

    do {
        varjo_Event evt{};
        ret = varjo_PollEvent(m_session, &evt);
        CHECK_VARJO_ERR(m_session);

        if (ret == varjo_True) {
            switch (evt.header.type) {
            case varjo_EventType_MRDeviceStatus: {
                switch (evt.data.mrDeviceStatus.status) {
                    // Occurs when Mixed Reality features are enabled
                case varjo_MRDeviceStatus_Connected: {
                    LOG_INFO("EVENT: Mixed reality device status: %s", "Connected");
                    constexpr bool forceSetState = true;
                    onMixedRealityAvailable(true, forceSetState);
                } break;
                                                   // Occurs when Mixed Reality features are disabled
                case varjo_MRDeviceStatus_Disconnected: {
                    LOG_INFO("EVENT: Mixed reality device status: %s", "Disconnected");
                    constexpr bool forceSetState = false;
                    onMixedRealityAvailable(false, forceSetState);
                } break;
                default: {
                    // Ignore unknown status.
                } break;
                }
            } break;

            case varjo_EventType_DataStreamStart: {
                LOG_INFO("EVENT: Data stream started: id=%d", static_cast<int>(evt.data.dataStreamStart.streamId));
            } break;

            case varjo_EventType_DataStreamStop: {
                LOG_INFO("EVENT: Data stream stopped: id=%d", static_cast<int>(evt.data.dataStreamStop.streamId));
            } break;

            default: {
                // Ignore unknown event.
            } break;
            }
        }
    } while (ret);
}