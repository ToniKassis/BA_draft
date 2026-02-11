// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#pragma once

#include <memory>
#include <chrono>
#include <glm/glm.hpp>

#include <fstream>
#include <iomanip>
#include <atomic>
#include "Globals.hpp"
#include "D3D11Renderer.hpp"
#include "MultiLayerView.hpp"
#include "Scene.hpp"
#include "MarkerTracker.hpp"
#include "CameraManager.hpp"
#include "DataStreamer.hpp"


#include "AppState.hpp"
#include "GfxContext.hpp"
#include "MRScene.hpp"

#include <atomic>


#include "HeadlessView.hpp"

#include "PointCloud.hpp"

#include <Varjo_mr_experimental.h>
#include "IPCConnectorHost.hpp"
//#include "Joints2D.hpp"

#include <opencv2/opencv.hpp>
#include "Visualization.hpp"

#include <k4a/k4a.hpp>
#include <k4abt.hpp>

//! Application logic class
class AppLogic
{
public:
    //! Constructor
    AppLogic() = default;

    //! Destruictor
    ~AppLogic();

    void drawSegmentsOnRgb(cv::Mat& bgr);


    // Disable copy and assign
    AppLogic(const AppLogic& other) = delete;
    AppLogic(const AppLogic&& other) = delete;
    AppLogic& operator=(const AppLogic& other) = delete;
    AppLogic& operator=(const AppLogic&& other) = delete;

    //! Initialize application
    bool init(VarjoExamples::GfxContext& context);

    //! Check for Varjo API events
    void checkEvents();

    //! Update application state
    void setState(const AppState& appState, bool force);

    //! Returns application state
    const AppState& getState() const { return m_appState; }

    //! Update application
    void update();

    //! Return true is app has been initialized successfully
    bool isInitialized() const { return m_initialized; }

    //! Return camera manager instance
    VarjoExamples::CameraManager& getCamera() const { return *m_camera; }


    //! Return data streamer instance
    VarjoExamples::DataStreamer& getStreamer() const { return *m_streamer; }

    // --- Kinect + K4ABT
    k4a::calibration  m_k4aCalib;
    bool              m_k4aCalibOk = false;
    int               m_k4aWidth = 0, m_fx=0, m_k4aHeight = 0;

    float             m_k4aFx = 0.f, m_k4aFy = 0.f, m_k4aCx = 0.f, m_k4aCy = 0.f;

    k4abt::tracker    m_tracker;           // created once
    bool              m_trackerOk = false;

    // Varjo -> Kinect depth transform (meters). Start identity; replace when you calibrate.
    glm::mat4         m_T_varjoCam_to_k4aDepth = glm::mat4(1.0f);

    // Scratch buffers per frame (kept here to reduce allocs)
    std::vector<std::array<float, 3>> m_xyz_mm;

    //  simple performance / log
    size_t            m_frameIdxBT = 0;

    //visual Joints
    // --- preview / overlay state ---
    std::mutex m_previewMtx;

    // latest LEFT-eye rectified BGR and its intrinsics/extrinsics
    cv::Mat m_leftBGR;
    cv::Mat m_leftBGRA; // CV_8UC3
    bool    m_leftReady = false;
    
    int     m_leftW = 0, m_leftH = 0;
    double  m_leftFx = 0, m_leftFy = 0, m_leftCx = 0, m_leftCy = 0;  

    glm::mat4 m_leftExtr;           // newExtGLM used for rectified frame

    // Varjo<->K4A transform (you already have m_T_varjoCam_to_k4aDepth)
   
    // cache the inverse too (K4A depth -> Varjo sensing)
    glm::mat4 m_T_k4aDepth_to_varjoCam;

    // latest skeleton joints (3D in K4A depth coords, meters)
    std::vector<glm::vec3> m_lastJointsK4A; // size K4ABT_JOINT_COUNT or empty
    bool m_haveSkeleton = false;


    varjo_CameraIntrinsics m_intrinsicsLeft{};
    varjo_CameraIntrinsics m_intrinsicsRight{};
    bool m_haveIntrinsicsLeft = false;
    bool m_haveIntrinsicsRight = false;

private:
    //! Toggle VST rendering
    void setVideoRendering(bool enabled);

    //! Handle mixed reality availablity
    void onMixedRealityAvailable(bool available, bool forceSetState);

    //! Handles new color stream frames from the data streamer (run in DataStreamer's worker thread)
    void onFrameReceived(const VarjoExamples::DataStreamer::Frame& frame);

    //! Handles new pointcloud snapshot data from the pointcloud (run in pointcloud's worker thread)
    void onPointCloudSnapshotReceived(const varjo_PointCloudSnapshotContent& content);

    //! Undistorts latest color stream frames, creates a depth and normal map, and exports the data with shared memory.
    void processFrame();

private:
    bool m_initialized{ false };
    varjo_Session* m_session{nullptr};                           //!< Varjo session
    std::unique_ptr<VarjoExamples::D3D11Renderer> m_renderer; //!< Renderer instance
    std::unique_ptr<VarjoExamples::MultiLayerView> m_varjoView;  //!< Varjo layer ext view instance

  //  std::unique_ptr<VarjoExamples::HeadlessView> m_varjoView;  //!< Varjo layer ext view instance
    AppState m_appState{};                                       //!< Application state

    std::unique_ptr<MRScene> m_scene;                         //!< Scene instance

    std::unique_ptr<VarjoExamples::DataStreamer> m_streamer;  //!< Data streamer instance
    std::unique_ptr<VarjoExamples::CameraManager> m_camera;   //!< Camera manager instance

    struct FrameData {
        std::optional<varjo_DistortedColorFrameMetadata> metadata;                     //!< Color stream metadata
        std::array<std::optional<VarjoExamples::DataStreamer::Frame>, 2> colorFrames;  //!< Color stream stereo frames
        std::optional<VarjoExamples::DataStreamer::Frame> cubemapFrame;                //!< HDR cubemap frame
        std::optional<varjo_EnvironmentCubemapFrameMetadata> cubemapMetadata;          //!< HDR cubemap metadata
    };
    FrameData m_frameData;        //!< Latest frame data
    std::mutex m_frameDataMutex;  //!< Mutex for locking frame data

    varjo_PointCloudSnapshotContent m_pointCloudSnapshotContent; //!< Latest pointcloud snapshot
    std::mutex m_pointCloudSnapshotContentMutex; //!< Mutex for locking pointcloud snapshot data

    std::unique_ptr<PointCloud> m_pointCloud;  //!< Point Cloud instance

    std::unique_ptr<IPCConnectorHost> m_IPCConnector; //!< IPCConnector instance

    varjo_TextureFormat m_colorStreamFormat{varjo_TextureFormat_INVALID};  //!< Texture format for color stream

    std::thread processFrameThread; //!< Thread for processing the latest frame for shared memory export

    std::atomic<bool> processingFrame = false; //!< Bool for indicating if a frame is currently processed by the processFrameThread

    std::atomic<bool> validSnapshot = false; //!<Bool for indicating if a new pointcloud snapshot is available

    // demo of all streams
    Visualization m_viz{ cv::Size(1600, 900), "Realtime Viz" };

    
    cv::Mat m_depthPanel;   
    cv::Mat m_irPanel;      
    cv::Mat m_colorPanel;    
    cv::Mat m_cloudPanel;   

    std::atomic<uint64_t> m_frameCounter{ 0 };

    void saveCurrentPointCloudPLY(const std::string& folder, bool binary = true);



};
