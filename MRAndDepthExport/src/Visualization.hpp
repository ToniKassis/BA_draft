#pragma once
#include <opencv2/opencv.hpp>
#include <array>
#include <string>




class Visualization {
public:
    explicit Visualization(cv::Size canvas = { 1600, 900 },
        const std::string& winName = "Realtime Viz")
        : canvasSize_(canvas), winName_(winName)
    {
        cv::namedWindow(winName_, cv::WINDOW_NORMAL);
        cv::resizeWindow(winName_, canvasSize_.width, canvasSize_.height);
        cv::moveWindow(winName_, 80, 60);
    }

    // panels[0..2] -> left column (top..bottom)
    // panels[3]    -> center (big; your point cloud when you have an image/texture)
    // panels[4..6] -> right column (top..bottom)
    void show(const std::array<cv::Mat, 7>& panels) {
        cv::Mat wall = compose(panels);
        cv::imshow(winName_, wall);
        cv::waitKey(1); // non-blocking tick for HighGUI
    }

    // Convenience: produce a labeled placeholder of the target size
    static cv::Mat placeholder(const std::string& title, cv::Size sz) {
        cv::Mat m(sz, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::rectangle(m, { 0,0 }, { sz.width - 1, sz.height - 1 }, { 90,90,90 }, 2);
        int base = std::max(1, sz.height / 28);
        cv::putText(m, title, { 16, 4 * base },
            cv::FONT_HERSHEY_SIMPLEX, std::max(0.7, sz.height / 720.0), { 240,240,240 }, 2, cv::LINE_AA);
        return m;
    }

private:
    cv::Size canvasSize_;
    std::string winName_;

    cv::Mat compose(const std::array<cv::Mat, 7>& p) const {
        const int W = canvasSize_.width, H = canvasSize_.height;
        const int wL = (int)std::round(0.25 * W);
        const int wC = (int)std::round(0.50 * W);
        const int wR = W - wL - wC;
        const int hSide = H / 3;

        struct R { int x, y, w, h; };
        std::array<R, 7> rc{ {
            {0,          0,   wL, hSide},           // L0
            {0,       hSide,  wL, hSide},           // L1
            {0,    2 * hSide,   wL, H - 2 * hSide},     // L2
            {wL,        0,    wC, H},               // Center
            {wL + wC,     0,    wR, hSide},           // R0
            {wL + wC,  hSide,   wR, hSide},           // R1
            {wL + wC,2 * hSide,   wR, H - 2 * hSide},     // R2
        } };

        cv::Mat wall(H, W, CV_8UC3, cv::Scalar(20, 20, 20));

        for (int i = 0; i < 7; ++i) {
            cv::Mat src = p[i];
            if (src.empty()) {
                src = placeholder(defaultTitle(i), { rc[i].w, rc[i].h });
            }
            else if (src.type() == CV_8UC1) {
                cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
            }
            cv::Mat resized;
            cv::resize(src, resized, { rc[i].w, rc[i].h }, 0, 0, cv::INTER_AREA);
            resized.copyTo(wall(cv::Rect(rc[i].x, rc[i].y, rc[i].w, rc[i].h)));
        }
        return wall;
    }

    static const char* defaultTitle(int idx) {
        switch (idx) {
        case 0: return "Left-Top (L0)";
        case 1: return "Left-Mid (L1)";
        case 2: return "Left-Bot (L2)";
        case 3: return "CENTER — Point Cloud";
        case 4: return "Right-Top (R0)";
        case 5: return "Right-Mid (R1)";
        case 6: return "Right-Bot (R2)";
        default: return "";
        }
    }
};
