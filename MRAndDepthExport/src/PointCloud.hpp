// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#pragma once

#include "Globals.hpp"
#include <mutex>
#include <thread>
#include <Varjo_mr_experimental.h>


#include <fstream>
#include <iomanip>
#include <atomic>

//! Application logic class
class PointCloud
{
public:
    //! Constructor
    PointCloud(varjo_Session* session, const std::function<void(const varjo_PointCloudSnapshotContent&)>& onPointCloudCallback);

    //! Destructor. Ends pointcloud snapshot creation and joins pointcloudthread.
    ~PointCloud();

    // Disable copy and assign
    PointCloud(const PointCloud& other) = delete;
    PointCloud(const PointCloud&& other) = delete;
    PointCloud& operator=(const PointCloud& other) = delete;
    PointCloud& operator=(const PointCloud&& other) = delete;

    //! Update point cloud
    void update();

    //! Handle mixed reality availablity. Enables pointcloud creation based on mixed reality availability.
    void onMixedRealityAvailable(bool available);

    //! Create a depth and normal map based on camera parameters and latest pointcloud snapshot
    void createDepthAndNormalMap(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, std::vector<uint32_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap);

    void createDepthAndNormalMapForDisplay(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, std::vector<uint8_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap);

    //! Create a depth and normal map based on camera parameters and a list of supplied points in 3D space.
    void createDepthAndNormalMapFromPoints(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, glm::vec3* points, int pointcount, std::vector<uint32_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap);

    bool extractXYZmmForK4A(std::vector<std::array<float, 3>>& xyz_mm,
        const glm::mat4& T_varjoCam_to_k4aDepth,
        float zMin_m, float zMax_m) const;

private:

    varjo_Session* m_session; //!< Varjo session

    varjo_PointCloudSnapshotId m_PointCloudSnapshotId = varjo_PointCloudSnapshotId_Invalid; //!< Id of latest requested Pointcloud snapshot. Set initially to invalid

    const std::function<void(const varjo_PointCloudSnapshotContent&)> m_onPointCloudCallback;  //!< Frame callback function

    bool mrAvailable = false; //!< Indicator if mixed reality is available

    bool updateSnapshot = false; //!< Indicator if the pointcloud thread should continue updating the snapshot
    
    std::thread PointCloudThread; //!< Thread to concurrently updating the pointcloud snapshot

    varjo_PointCloudSnapshotContent m_latestSnapshot = varjo_PointCloudSnapshotContent(); //!< Latest acquired pointcloud snapshot

    mutable std::mutex pointCloudMutex; //!< Mutex for locking pointcloud snapshot data

    mutable std::mutex m_pointsMutex{};
    std::vector<varjo_PointCloudPoint> m_pointsOwned; // owned copy for other threads
    std::atomic<bool> validSnapshot{ false };           // optional
    varjo_PointCloudSnapshotContent m_lastMeta{};     // optional metadata (points=null)

    std::atomic<uint64_t> m_frameCounter{ 0 };
    void saveCurrentPointCloudPLY(const std::string& folder, bool binary = true);
    bool PointCloud::writeAsciiPlyAtomic(const std::filesystem::path& finalPath);



};
