// Copyright 2019-2021 Varjo Technologies Oy. All rights reserved.

#include "PointCloud.hpp"
#include <cstdio>
#include <vector>
#include <string>
#include <limits>
#include <map>
#include <array>
#include <iostream>
#include <glm/gtx/matrix_transform_2d.hpp>
#include "glm/ext.hpp"

#include <fstream>
#include <filesystem>
#include <cerrno>
#include <cstring>

#include <fstream>
#include <filesystem>
#include <system_error>
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#endif




// VarjoExamples namespace contains simple example wrappers for using Varjo API features.
// These are only meant to be used in SDK example applications. In your own application,
// use your own production quality integration layer.
using namespace VarjoExamples;

using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;


PointCloud::PointCloud(varjo_Session* session, const std::function<void(const varjo_PointCloudSnapshotContent&)>& onPointCloudCallback)
    :m_session(session), m_onPointCloudCallback(onPointCloudCallback)
{
}

PointCloud::~PointCloud()
{
    //stop thread
    updateSnapshot = false;

    //wait for thread to join
    if (PointCloudThread.joinable()) {
        PointCloudThread.join();
    }

    //end capturing snapshot
    if (varjo_MRGetPointCloudSnapshotStatus(m_session, m_PointCloudSnapshotId) != 0) {
        varjo_MREndPointCloudSnapshot(m_session, m_PointCloudSnapshotId);
    }

    m_PointCloudSnapshotId = varjo_PointCloudSnapshotId_Invalid;

    // Disable 3D reconstruction
    varjo_MRSetReconstruction(m_session, varjo_False);
    CHECK_VARJO_ERR(m_session);
}



static inline void writePlyHeader(std::ostream& os, size_t vertexCount, bool binary)
{
    os << "ply\n";
    os << (binary ? "format binary_little_endian 1.0\n"
        : "format ascii 1.0\n");
    os << "element vertex " << vertexCount << "\n";
    os << "property uint id\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "property float nx\nproperty float ny\nproperty float nz\n";
    os << "property uchar confidence\n";
    os << "property float radius\n";
    os << "end_header\n";
}

bool PointCloud::writeAsciiPlyAtomic(const std::filesystem::path& finalPath)
{
    std::vector<varjo_PointCloudPoint> pts;
    {
        std::lock_guard<std::mutex> lock(m_pointsMutex);
        if (m_pointsOwned.empty()) return false;
        pts = m_pointsOwned;
    }

    std::filesystem::create_directories(finalPath.parent_path());

    const auto tmpPath = finalPath.parent_path() / (finalPath.filename().string() + ".tmp");

    std::ofstream ofs(tmpPath, std::ios::out | std::ios::trunc);
    if (!ofs) {
        LOG_ERROR("Failed to open %s", tmpPath.string().c_str());
        return false;
    }

    // Filter and count valid points
    size_t kept = 0;
    for (const auto& p : pts) {
        if ((p.indexConfidence & 0xFFu) >= 1) ++kept;
    }

    // --- Header ---
    ofs << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << kept << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property float nx\nproperty float ny\nproperty float nz\n"
        << "end_header\n";

    ofs.setf(std::ios::fixed);
    ofs.precision(7);

    // --- Body ---
    for (const auto& p : pts) {
        const uint8_t conf = static_cast<uint8_t>(p.indexConfidence & 0xFFu);
        if (conf < 1) continue;

        const glm::vec2 posXY = glm::unpackHalf2x16(p.positionXY);
        const glm::vec2 posZr = glm::unpackHalf2x16(p.positionZradius);
        const glm::vec2 nXY = glm::unpackHalf2x16(p.normalXY);
        const glm::vec2 nZr = glm::unpackHalf2x16(p.normalZcolorR);

        ofs << posXY.x << " " << posXY.y << " " << posZr.x << " "
            << nXY.x << " " << nXY.y << " " << nZr.x << "\n";
    }

    ofs.flush();
    ofs.close();

    // --- Atomic rename ---
#ifdef _WIN32
    MoveFileExA(tmpPath.string().c_str(),
        finalPath.string().c_str(),
        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
#else
    std::filesystem::rename(tmpPath, finalPath);
#endif

    LOG_INFO("Wrote PLY: %s", finalPath.string().c_str());
    return true;
}

void PointCloud::saveCurrentPointCloudPLY(const std::string& folder, bool binary)
{
    // Safely copy points
    std::vector<varjo_PointCloudPoint> pts;
    {
        std::lock_guard<std::mutex> lock(m_pointsMutex);
        if (m_pointsOwned.empty()) return;
        pts = m_pointsOwned;
    }

    // Make sure the folder exists
    std::filesystem::path outDir(folder);
    std::error_code ec;
    if (!std::filesystem::exists(outDir)) {
        if (!std::filesystem::create_directories(outDir, ec)) {
            LOG_ERROR("Failed to create directory '%s': %s",
                outDir.string().c_str(), ec.message().c_str());

            // Fallback to temp directory to avoid losing frames
            outDir = std::filesystem::temp_directory_path() / "VarjoPointCloud";
            ec.clear();
            std::filesystem::create_directories(outDir, ec);
            if (ec) {
                LOG_ERROR("Also failed to create fallback dir '%s': %s",
                    outDir.string().c_str(), ec.message().c_str());
                return;
            }
            LOG_INFO("Falling back to '%s'", outDir.string().c_str());
        }
    }

    // Count how many points we'll keep
    size_t kept = 0;
    for (const auto& p : pts) {
        const uint8_t conf = static_cast<uint8_t>(p.indexConfidence & 0xFFu);
        if (conf >= 1) ++kept;
    }
    if (kept == 0) return;

    // Build a unique filename
    const uint64_t frame = m_frameCounter.fetch_add(1, std::memory_order_relaxed);
    const std::filesystem::path outPath = outDir / ("cloud_" + std::to_string(frame) + ".ply");

    // Open file (always include trunc so re-runs overwrite cleanly)
    std::ofstream ofs(outPath, (binary ? (std::ios::binary | std::ios::trunc)
        : (std::ios::out | std::ios::trunc)));
    if (!ofs.is_open()) {
        LOG_ERROR("Failed to open %s: %s", outPath.string().c_str(), std::strerror(errno));
        return;
    }

    // ---- Header
    auto writePlyHeader = [&](std::ostream& os) {
        os << "ply\n"
            << (binary ? "format binary_little_endian 1.0\n" : "format ascii 1.0\n")
            << "element vertex " << kept << "\n"
            << "property uint id\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "property float nx\nproperty float ny\nproperty float nz\n"
            << "property uchar confidence\n"
            << "property float radius\n"
            << "end_header\n";
        };
    writePlyHeader(ofs);

    // ---- Payload
    if (binary) {
        struct PlyVertex {
            uint32_t id;
            float x, y, z;
            float nx, ny, nz;
            uint8_t confidence;
            float radius;
        } v{};

        for (const auto& p : pts) {
            const uint32_t id = (p.indexConfidence >> 8);
            const uint8_t  conf = static_cast<uint8_t>(p.indexConfidence & 0xFFu);
            if (conf < 1) continue;

            const glm::vec2 posXY = glm::unpackHalf2x16(p.positionXY);
            const glm::vec2 posZradius = glm::unpackHalf2x16(p.positionZradius);
            const glm::vec2 normalXY = glm::unpackHalf2x16(p.normalXY);
            const glm::vec2 normalZcolorR = glm::unpackHalf2x16(p.normalZcolorR);

            v.id = id;
            v.x = posXY.x;  v.y = posXY.y;  v.z = posZradius.x;
            v.nx = normalXY.x; v.ny = normalXY.y; v.nz = normalZcolorR.x;
            v.confidence = conf;
            v.radius = posZradius.y;

            ofs.write(reinterpret_cast<const char*>(&v), sizeof(v));
        }
    }
    else {
        ofs.setf(std::ios::fixed); ofs.precision(7);
        for (const auto& p : pts) {
            const uint32_t id = (p.indexConfidence >> 8);
            const uint8_t  conf = static_cast<uint8_t>(p.indexConfidence & 0xFFu);
            if (conf < 1) continue;

            const glm::vec2 posXY = glm::unpackHalf2x16(p.positionXY);
            const glm::vec2 posZradius = glm::unpackHalf2x16(p.positionZradius);
            const glm::vec2 normalXY = glm::unpackHalf2x16(p.normalXY);
            const glm::vec2 normalZcolorR = glm::unpackHalf2x16(p.normalZcolorR);

            ofs << id << " "
                << posXY.x << " " << posXY.y << " " << posZradius.x << " "
                << normalXY.x << " " << normalXY.y << " " << normalZcolorR.x << " "
                << static_cast<unsigned>(conf) << " "
                << posZradius.y << "\n";
        }
    }

    ofs.close();
    LOG_INFO("Saved PLY: %s", std::filesystem::absolute(outPath).string().c_str());
}



void PointCloud::update()
{

    //update point cloud snapshot
    while (updateSnapshot) {

        //only update snapshot if mixed reality is available
        LOG_DEBUG("Trying to update Snapshot...");
        varjo_PointCloudSnapshotStatus snapshotStatus = varjo_MRGetPointCloudSnapshotStatus(m_session, m_PointCloudSnapshotId);
        if (snapshotStatus == 0) {
            //need to begin new snapshot
            LOG_DEBUG("Beginning new snapshot...");
            m_PointCloudSnapshotId = varjo_MRBeginPointCloudSnapshot(m_session);
        }
        else if (snapshotStatus == 1) {
            //ignore since snapshot is not ready
            LOG_DEBUG("Snapshot not ready...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (snapshotStatus == 2) {
            LOG_DEBUG("Snapshot ready, copying...");
            varjo_PointCloudSnapshotContent snap{};
            varjo_MRGetPointCloudSnapshotContent(m_session, m_PointCloudSnapshotId, &snap);

            // Copy to owned storage while snapshot is still held
            {
                std::lock_guard<std::mutex> lock(m_pointsMutex);
                m_pointsOwned.assign(snap.points, snap.points + snap.pointCount);
                m_lastMeta = snap;
                m_lastMeta.points = nullptr;              // never expose raw pointer
                validSnapshot.store(snap.pointCount > 0, std::memory_order_relaxed);
            }

            varjo_MRReleasePointCloudSnapshot(m_session, m_PointCloudSnapshotId);

            // Save PLY for this frame
            // comment 1 line below to deactivte the exporting of ply files
            //saveCurrentPointCloudPLY("PLY", /*binary*/ false);

            //live one file
           // writeAsciiPlyAtomic("PLY/live.ply");


            if (m_onPointCloudCallback && snap.pointCount > 0) {
                auto safe = snap; safe.points = nullptr;
                m_onPointCloudCallback(safe);
            }
            varjo_MREndPointCloudSnapshot(m_session, m_PointCloudSnapshotId);
            m_PointCloudSnapshotId = varjo_MRBeginPointCloudSnapshot(m_session);
        }

    }
}

void PointCloud::onMixedRealityAvailable(bool available)
{
    mrAvailable = available;

    if (available && !updateSnapshot) {

        // When MR becomes available, Enable 3D reconstruction

        // Enable 3D reconstruction
        varjo_MRSetReconstruction(m_session, varjo_True);
        CHECK_VARJO_ERR(m_session);

        //Begin capturing Snapshot
        m_PointCloudSnapshotId = varjo_MRBeginPointCloudSnapshot(m_session);

        updateSnapshot = available;

        //start PointCloudThread
        PointCloudThread = std::thread(&PointCloud::update, this);

        LOG_INFO("Started capturing pointcloud snapshots");

    }
    else if (!available && updateSnapshot) {
        LOG_ERROR("Mixed Reality features not available.");

        // When MR becomes unavailable, disable 3D reconstruction

        //stop thread
        updateSnapshot = available;

        //wait for thread to join
        if (PointCloudThread.joinable()) {
            PointCloudThread.join();
        }

        //end capturing snapshot
        if (varjo_MRGetPointCloudSnapshotStatus(m_session, m_PointCloudSnapshotId) != 0) {
            varjo_MREndPointCloudSnapshot(m_session, m_PointCloudSnapshotId);
        }

        m_PointCloudSnapshotId = varjo_PointCloudSnapshotId_Invalid;

        // Disable 3D reconstruction
        varjo_MRSetReconstruction(m_session, varjo_False);
        CHECK_VARJO_ERR(m_session);

        LOG_INFO("Stopped capturing pointcloud snapshots");

    }
}

// A 3D point with an id and a surface normal
 
struct IdentifiedPoint3D {
    uint32_t id;
    glm::vec3 coord;
    glm::vec3 normal;
public:
    IdentifiedPoint3D(uint32_t idIn, glm::vec3 coordIn)
        :id(idIn), coord(coordIn)
    {
    };
    IdentifiedPoint3D(uint32_t idIn, glm::vec3 coordIn, glm::vec3 normalIn)
        :id(idIn), coord(coordIn), normal(normalIn)
    {
    };
};

/**
 * An id of a pointcloud point with a distance from the camera and a surface normal
 */
struct IdAndDepth {
    uint32_t id;
    float dist;
    glm::vec3 normal;
public:
    IdAndDepth(uint32_t idIn, float distIn, glm::vec3 normalIn)
        :id(idIn), dist(distIn), normal(normalIn)
    {
    }
    IdAndDepth(uint32_t idIn, float distIn)
        :id(idIn), dist(distIn)
    {
    }
    IdAndDepth()
        :id(std::numeric_limits<uint32_t>::max()), dist(0.0), normal(glm::vec3(0, 0, 0))
    {
        //setting id default to maximal uint32 value, because the maximal used id from varjo struct is uint32 >> 8 and so this value can't be reached when the id is valid
    }
};

void PointCloud::createDepthAndNormalMap(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, std::vector<uint32_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap)
{
    //only generate depth and normal maps when snapshot is currently generated.
    if (updateSnapshot) {
        //Acquire latest pointcloud snapshot
        varjo_PointCloudSnapshotContent latestContent = varjo_PointCloudSnapshotContent();
        {
            std::lock_guard<std::mutex> streamLock(pointCloudMutex);
            latestContent = m_latestSnapshot;
        }

        //only generate depth and normal maps when we have points
        if (latestContent.pointCount > 0) {
            maxDist = std::numeric_limits<float>::min();
            minDist = std::numeric_limits<float>::max();

            //build intrinsic matrix
            const glm::mat3x3 offset = glm::translate(glm::mat3(1.0f), glm::vec2((intrinsics.principalPointX - 0.5) * -2.0, (0.5 - intrinsics.principalPointY) * -2.0));
            const glm::mat3x3 scale = glm::scale(glm::mat3(1.0f), glm::vec2(intrinsics.focalLengthX, intrinsics.focalLengthY));
            glm::mat3x3 intrinsicsMat = offset * scale;

            //get camera center point direction (in varjo camera space negative z goes forward, so in this case it would just be (0,0,-1))
            glm::vec3 center(0, 0, -1.0);
            glm::vec3 centerNormed = glm::normalize(center); //not necessary, but included for completion

            //get extrinsic matrix
            glm::mat4x4 extrinsicsGLM = VarjoExamples::fromVarjoMatrix(extrinsics);
            std::vector<IdentifiedPoint3D> pointCameraCoordinates;

            for (int i = 0; i < latestContent.pointCount; i++) {

                //unpack pointcloud point coordinates and normals
                varjo_PointCloudPoint p = latestContent.points[i];
                uint32_t id = (p.indexConfidence >> 8);
                uint32_t confidence = (p.indexConfidence & 0xFF);
                glm::vec2 positionXY = glm::unpackHalf2x16(p.positionXY);
                glm::vec2 positionZradius = glm::unpackHalf2x16(p.positionZradius);
                glm::vec2 normalXY = glm::unpackHalf2x16(p.normalXY);
                glm::vec2 normalZcolorR = glm::unpackHalf2x16(p.normalZcolorR);

                //convert to camera coordinates
                glm::vec4 vec4P(glm::vec3(positionXY.x, positionXY.y, positionZradius.x), 1);
                glm::vec4 cameraP = extrinsicsGLM * vec4P;
                glm::vec3 vec3P = glm::vec3(cameraP.x / cameraP.w, cameraP.y / cameraP.w, cameraP.z / cameraP.w);

                //filter points based on confidence here
                if (confidence < 1) {
                    continue;
                }

                //compute dot product with camera vector to sort out points behind the camera/that can't be in view
                float d = glm::dot(centerNormed, vec3P);

                if (d >= 0) {
                    //keep point
                    pointCameraCoordinates.push_back(IdentifiedPoint3D(id, vec3P, glm::vec3(normalXY.x, normalXY.y, normalZcolorR.x))); //vec3n
                }
            }


            //project the rest of the points onto the camera plane and check if resulting u v values are within the width and height specified by the image resolution
            //set value of resulting pixel to distance of the point from camera (clamped between minimal and maximal distance)
            //if multiple values result in the same pixel, then keep value of closest point (~like a z buffer)

            //structure to save per pixel depth values
            std::vector<std::vector<IdAndDepth>> pixelToDepth(resolution.y, std::vector <IdAndDepth>(resolution.x));

            for (IdentifiedPoint3D p : pointCameraCoordinates) {

                //project 3D point coordinates on the camera plane
                glm::vec3 projP = intrinsicsMat * p.coord;

                //use -z coordinate because coordinate system has negative z direction as forward axis
                glm::vec2 projP2D = glm::vec2(projP.x / -projP.z, projP.y / -projP.z);

                //calculate corresponding pixel coordinate
                glm::ivec2 pixel;
                pixel.x = glm::floor(((projP2D.x + 1.0) * 0.5 * resolution.x) - 0.5);
                pixel.y = glm::floor(((1.0 - projP2D.y) * 0.5 * resolution.y) - 0.5);

                if (pixel.x >= 0 && pixel.x < resolution.x && pixel.y >= 0 && pixel.y < resolution.y) {
                    //if point is within our defined width and height, keep value;

                    //compute distance to camera (in camera coordinate system the camera is in the origin)
                    float dist = glm::distance(glm::vec3(0, 0, 0), p.coord);

                    //save distance value in array at pixel location
                    if (pixelToDepth[pixel.y][pixel.x].id != std::numeric_limits<uint32_t>::max()) {
                        //there is already a point projected to this pixel, select closest of both
                        float currentDist = pixelToDepth[pixel.y][pixel.x].dist;

                        if (dist < currentDist) {
                            //save new depth value
                            pixelToDepth[pixel.y][pixel.x] = IdAndDepth(p.id, dist);
                        }
                        else {
                            //keep old depth and id
                        }
                    }
                    else {
                        //save id to pixel
                        pixelToDepth[pixel.y][pixel.x] = IdAndDepth(p.id, dist, p.normal);
                    }
                }
            }

            //calculate max and min distances
            for (std::vector<IdAndDepth> vec : pixelToDepth) {
                for (IdAndDepth pixel : vec) {

                    if (pixel.id != std::numeric_limits<uint32_t>::max()) {
                        //save overall min and max distances for clamping the range between 0.0 and 1.0 later
                        if (pixel.dist > maxDist) {
                            maxDist = pixel.dist;
                        }

                        if (pixel.dist < minDist) {
                            minDist = pixel.dist;
                        }

                    }
                }
            }

            //initialize vectors
            depthMap = std::vector<uint32_t>();

            normalMap = std::vector<uint8_t>();

            //decrease minDist, so no pixel with a value that was set above has 0 as value and we know which pixels are invalid
            if (minDist - 0.01 >= 0.0) {
                minDist = minDist - 0.01f;
            }

            //transform distance values to depth values between 0.0 and 1.0 and save depth image data
            for (std::vector<IdAndDepth> vec : pixelToDepth) {
                for (IdAndDepth pixel : vec) {

                    if (pixel.id != std::numeric_limits<uint32_t>::max()) {
                        //pixel value was set in the procedure above. At least one pointcloud point could be projected to the pixel

                        //calculate depth value between 0.0 an 1.0
                        float color = (pixel.dist - minDist) / (maxDist - minDist);

                        if (color <= 1.0 && color >= 0.0) {

                            //convert to int32_t
                            uint32_t convertedColor;
                            if (color >= 1.0) {
                                convertedColor = std::numeric_limits<uint32_t>::max();
                            }
                            else {
                                convertedColor = floor(color * 4294967296);
                            }

                            //save depth to depht map
                            depthMap.push_back(convertedColor);

                            //save normal to normal map
                            float normalX = pixel.normal.x;
                            float normalY = pixel.normal.y;
                            float normalZ = pixel.normal.z;

                            //shift to 0-1 range
                            normalX = (normalX * 0.5) + 0.5;
                            normalY = (normalY * 0.5) + 0.5;
                            normalZ = (normalZ * 0.5) + 0.5;

                            //convert to uint_8
                            uint8_t intX = glm::packUnorm1x8(normalX);
                            uint8_t intY = glm::packUnorm1x8(normalY);
                            uint8_t intZ = glm::packUnorm1x8(normalZ);
                            //r = x
                            normalMap.push_back(intX);
                            //g = y
                            normalMap.push_back(intY);
                            //b = z
                            normalMap.push_back(intZ);


                        }
                        else {
                            //invalid value. set pixel to black
                            depthMap.push_back(0);
                            //r = x
                            normalMap.push_back(0);
                            //g = y
                            normalMap.push_back(0);
                            //b = z
                            normalMap.push_back(0);
                        }
                    }
                    else {
                        //no pixel value was set above. No pointcloud point could be projected to the pixel. Set color to black/0
                        depthMap.push_back(0);
                        //r = x
                        normalMap.push_back(0);
                        //g = y
                        normalMap.push_back(0);
                        //b = z
                        normalMap.push_back(0);
                    }

                }
            }
        }
    }
}

void PointCloud::createDepthAndNormalMapForDisplay(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, std::vector<uint8_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap)
{
    //only generate depth and normal maps when snapshot is currently generated.
    if (updateSnapshot) {
        //Acquire latest pointcloud snapshot
        varjo_PointCloudSnapshotContent latestContent = varjo_PointCloudSnapshotContent();
        {
            std::lock_guard<std::mutex> streamLock(pointCloudMutex);
            latestContent = m_latestSnapshot;
        }

        //only generate depth and normal maps when we have points
        if (latestContent.pointCount > 0) {
            maxDist = std::numeric_limits<float>::min();
            minDist = std::numeric_limits<float>::max();

            //build intrinsic matrix
            const glm::mat3x3 offset = glm::translate(glm::mat3(1.0f), glm::vec2((intrinsics.principalPointX - 0.5) * -2.0, (0.5 - intrinsics.principalPointY) * -2.0));
            const glm::mat3x3 scale = glm::scale(glm::mat3(1.0f), glm::vec2(intrinsics.focalLengthX, intrinsics.focalLengthY));
            glm::mat3x3 intrinsicsMat = offset * scale;

            //get camera center point direction (in varjo camera space negative z goes forward, so in this case it would just be (0,0,-1))
            glm::vec3 center(0, 0, -1.0);
            glm::vec3 centerNormed = glm::normalize(center); //not necessary, but included for completion

            //get extrinsic matrix
            glm::mat4x4 extrinsicsGLM = VarjoExamples::fromVarjoMatrix(extrinsics);
            std::vector<IdentifiedPoint3D> pointCameraCoordinates;

            for (int i = 0; i < latestContent.pointCount; i++) {

                //unpack pointcloud point coordinates and normals
                varjo_PointCloudPoint p = latestContent.points[i];
                uint32_t id = (p.indexConfidence >> 8);
                uint32_t confidence = (p.indexConfidence & 0xFF);
                glm::vec2 positionXY = glm::unpackHalf2x16(p.positionXY);
                glm::vec2 positionZradius = glm::unpackHalf2x16(p.positionZradius);
                glm::vec2 normalXY = glm::unpackHalf2x16(p.normalXY);
                glm::vec2 normalZcolorR = glm::unpackHalf2x16(p.normalZcolorR);

                //convert to camera coordinates
                glm::vec4 vec4P(glm::vec3(positionXY.x, positionXY.y, positionZradius.x), 1);
                glm::vec4 cameraP = extrinsicsGLM * vec4P;
                glm::vec3 vec3P = glm::vec3(cameraP.x / cameraP.w, cameraP.y / cameraP.w, cameraP.z / cameraP.w);

                //filter points based on confidence here
                if (confidence < 1) {
                    continue;
                }


                //compute dot product with camera vector to sort out points behind the camera/that can't be in view
                float d = glm::dot(centerNormed, vec3P);

                if (d >= 0) {
                    //keep point
                    pointCameraCoordinates.push_back(IdentifiedPoint3D(id, vec3P, glm::vec3(normalXY.x, normalXY.y, normalZcolorR.x))); //vec3n
                }
            }


            //project the rest of the points onto the camera plane and check if resulting u v values are within the width and height specified by the image resolution
            //set value of resulting pixel to distance of the point from camera (clamped between minimal and maximal distance)
            //if multiple values result in the same pixel, then keep value of closest point (~like a z buffer)

            //structure to save per pixel depth values
            std::vector<std::vector<IdAndDepth>> pixelToDepth(resolution.y, std::vector <IdAndDepth>(resolution.x));

            for (IdentifiedPoint3D p : pointCameraCoordinates) {

                //project 3D point coordinates on the camera plane
                glm::vec3 projP = intrinsicsMat * p.coord;

                //use -z coordinate because coordinate system has negative z direction as forward axis
                glm::vec2 projP2D = glm::vec2(projP.x / -projP.z, projP.y / -projP.z);

                //calculate corresponding pixel coordinate
                glm::ivec2 pixel;
                pixel.x = glm::floor(((projP2D.x + 1.0) * 0.5 * resolution.x) - 0.5);
                pixel.y = glm::floor(((1.0 - projP2D.y) * 0.5 * resolution.y) - 0.5);

                if (pixel.x >= 0 && pixel.x < resolution.x && pixel.y >= 0 && pixel.y < resolution.y) {
                    //if point is within our defined width and height, keep value;

                    //compute distance to camera (in camera coordinate system the camera is in the origin)
                    float dist = glm::distance(glm::vec3(0, 0, 0), p.coord);

                    //save distance value in array at pixel location
                    if (pixelToDepth[pixel.y][pixel.x].id != std::numeric_limits<uint32_t>::max()) {
                        //there is already a point projected to this pixel, select closest of both
                        float currentDist = pixelToDepth[pixel.y][pixel.x].dist;

                        if (dist < currentDist) {
                            //save new depth value
                            pixelToDepth[pixel.y][pixel.x] = IdAndDepth(p.id, dist);
                        }
                        else {
                            //keep old depth and id
                        }
                    }
                    else {
                        //save id to pixel
                        pixelToDepth[pixel.y][pixel.x] = IdAndDepth(p.id, dist, p.normal);
                    }
                }
            }

            //calculate max and min distances
            for (std::vector<IdAndDepth> vec : pixelToDepth) {
                for (IdAndDepth pixel : vec) {

                    if (pixel.id != std::numeric_limits<uint32_t>::max()) {
                        //save overall min and max distances for clamping the range between 0.0 and 1.0 later
                        if (pixel.dist > maxDist) {
                            maxDist = pixel.dist;
                        }

                        if (pixel.dist < minDist) {
                            minDist = pixel.dist;
                        }

                    }
                }
            }

            //initialize vectors
            depthMap = std::vector<uint8_t>();

            normalMap = std::vector<uint8_t>();

            //decrease minDist, so no pixel with a value that was set above has 0 as value and we know which pixels are invalid
            if (minDist - 0.01 >= 0.0) {
                minDist = minDist - 0.01f;
            }

            //transform distance values to depth values between 0.0 and 1.0 and save depth image data
            for (std::vector<IdAndDepth> vec : pixelToDepth) {
                for (IdAndDepth pixel : vec) {

                    if (pixel.id != std::numeric_limits<uint32_t>::max()) {
                        //pixel value was set in the procedure above. At least one pointcloud point could be projected to the pixel

                        //calculate depth value between 0.0 an 1.0
                        float color = (pixel.dist - minDist) / (maxDist - minDist);

                        if (color <= 1.0 && color >= 0.0) {

                            //convert to uint38_t
                            uint32_t convertedColor;
                            if (color >= 1.0) {
                                convertedColor = std::numeric_limits<uint8_t>::max();
                            }
                            else {
                                convertedColor = floor(color * 256);
                            }

                            //save depth to depht map
                            depthMap.push_back(convertedColor);

                            //save normal to normal map
                            float normalX = pixel.normal.x;
                            float normalY = pixel.normal.y;
                            float normalZ = pixel.normal.z;

                            //shift to 0-1 range
                            normalX = (normalX * 0.5) + 0.5;
                            normalY = (normalY * 0.5) + 0.5;
                            normalZ = (normalZ * 0.5) + 0.5;

                            //convert to uint_8
                            uint8_t intX = glm::packUnorm1x8(normalX);
                            uint8_t intY = glm::packUnorm1x8(normalY);
                            uint8_t intZ = glm::packUnorm1x8(normalZ);
                            //r = x
                            normalMap.push_back(intX);
                            //g = y
                            normalMap.push_back(intY);
                            //b = z
                            normalMap.push_back(intZ);


                        }
                        else {
                            //invalid value. set pixel to black
                            depthMap.push_back(0);
                            //r = x
                            normalMap.push_back(0);
                            //g = y
                            normalMap.push_back(0);
                            //b = z
                            normalMap.push_back(0);
                        }
                    }
                    else {
                        //no pixel value was set above. No pointcloud point could be projected to the pixel. Set color to black/0
                        depthMap.push_back(0);
                        //r = x
                        normalMap.push_back(0);
                        //g = y
                        normalMap.push_back(0);
                        //b = z
                        normalMap.push_back(0);
                    }

                }
            }
        }
    }
}

bool PointCloud::extractXYZmmForK4A(std::vector<std::array<float, 3>>& xyz_mm,
    const glm::mat4& T_varjoCam_to_k4aDepth,
    float zMin_m, float zMax_m) const
{
    xyz_mm.clear();

    std::vector<varjo_PointCloudPoint> local;
    {
        std::lock_guard<std::mutex> lock(m_pointsMutex);
        LOG_INFO("Owned point count: %zu", m_pointsOwned.size());

        if (m_pointsOwned.empty()) return false;
        local = m_pointsOwned; // safe owned copy
    }

    xyz_mm.reserve(local.size());

    size_t seen = 0, kept = 0, negz = 0, outrange = 0;



    for (const auto& p : local) {
        
        
        const uint32_t conf = (p.indexConfidence & 0xFFu);
        if (conf < 1) continue;

        const vec2 posXY = glm::unpackHalf2x16(p.positionXY);
        const vec2 posZrad = glm::unpackHalf2x16(p.positionZradius);

        vec4 vVarjo(posXY.x, posXY.y, posZrad.x, 1.0f);
        vec4 vK4A = T_varjoCam_to_k4aDepth * vVarjo;
        if (vK4A.w == 0.0f) continue;

        vec3 q(vK4A.x / vK4A.w, vK4A.y / vK4A.w, vK4A.z / vK4A.w); // meters

        ++seen;
        if (q.z <= 0.0f) { ++negz; continue; }
        if (q.z < zMin_m || q.z > zMax_m) { ++outrange; continue; }
        if (q.z <= 0.0f || q.z < zMin_m || q.z > zMax_m) continue;


        xyz_mm.push_back(std::array<float, 3>{ q.x * 1000.0f, q.y * 1000.0f, q.z * 1000.0f });
        ++kept;
    }
    LOG_INFO("[extract] seen=%zu kept=%zu negz=%zu outrange=%zu", seen, kept, negz, outrange);

    return !xyz_mm.empty();
}

void PointCloud::createDepthAndNormalMapFromPoints(const glm::ivec2 resolution, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, glm::vec3* points, int pointcount, std::vector<uint32_t>& depthMap, float& minDist, float& maxDist, std::vector<uint8_t>& normalMap)
{
    if (pointcount > 0) {

        maxDist = std::numeric_limits<float>::min();
        minDist = std::numeric_limits<float>::max();

        //build intrinsic matrix
        const glm::mat3x3 offset = glm::translate(glm::mat3(1.0f), glm::vec2((intrinsics.principalPointX - 0.5) * -2.0, (0.5 - intrinsics.principalPointY) * -2.0));
        const glm::mat3x3 scale = glm::scale(glm::mat3(1.0f), glm::vec2(intrinsics.focalLengthX, intrinsics.focalLengthY));
        glm::mat3x3 intrinsicsMat = offset * scale;


        //get camera center point direction (in varjo camera space negative z goes forward, so in this case it would just be (0,0,-1))
        glm::vec3 center(0, 0, -1.0);
        glm::vec3 centerNormed = glm::normalize(center); //not necessary, but included for completion

        //get extrinsic matrix
        glm::mat4x4 extrinsicsGLM = VarjoExamples::fromVarjoMatrix(extrinsics);
        std::vector<IdentifiedPoint3D> pointCameraCoordinates;


        for (int i = 0; i < pointcount; i++) {

            glm::vec3 point = points[i];

            //convert to camera coordinates
            glm::vec4 vec4P(point, 1);
            glm::vec4 cameraP = extrinsicsGLM * vec4P;
            glm::vec3 vec3P = glm::vec3(cameraP.x / cameraP.w, cameraP.y / cameraP.w, cameraP.z / cameraP.w);

            //compute dot product with camera vector to sort out points behind the camera/that can't be in view
            float d = glm::dot(centerNormed, vec3P);
            if (d >= 0) {
                //keep point
                pointCameraCoordinates.push_back(IdentifiedPoint3D(i, vec3P));
            }
        }


        //project the rest of the points onto the camera plane and check if resulting u v values are within the width and height specified by the image resolution
        //set value of resulting pixel to distance of the point from camera (clamped between minimal and maximal distance)
        //if multiple values result in the same pixel, then keep value of closest point (~like a z buffer)

        //structure to save per pixel depth values
        std::vector<std::vector<IdAndDepth>> pixelToDepth(resolution.y, std::vector <IdAndDepth>(resolution.x));

        for (IdentifiedPoint3D p : pointCameraCoordinates) {

            //project 3D point coordinates on the camera plane
            glm::vec3 projP = intrinsicsMat * p.coord;

            //use -z coordinate because coordinate system has negative z direction as forward axis
            glm::vec2 projP2D = glm::vec2(projP.x / -projP.z, projP.y / -projP.z);

            //calculate corresponding pixel coordinate
            int x;
            int y;
            x = glm::floor(((projP2D.x + 1.0) * 0.5 * resolution.x) - 0.5);
            y = glm::floor(((1.0 - projP2D.y) * 0.5 * resolution.y) - 0.5);

            if (x >= 0 && x < resolution.x && y >= 0 && y < resolution.y) {
                //if point is within our defined width and height, keep value;

                //compute distance to camera (in camera coordinate system the camera is in the origin)
                float dist = glm::distance(glm::vec3(0, 0, 0), p.coord);

                //save overall min and max distances for clamping the range between 0.0 and 1.0 later
                if (dist > maxDist) {
                    maxDist = dist;
                }

                if (dist < minDist) {
                    minDist = dist;
                }

                //save distance value in array at pixel location
                if (pixelToDepth[y][x].id != std::numeric_limits<uint32_t>::max()) {
                    //there is already a point projected to this pixel, select closest of both
                    float currentDist = pixelToDepth[y][x].dist;

                    if (dist < currentDist) {
                        //save new depth value
                        pixelToDepth[y][x] = IdAndDepth(p.id, dist);
                    }
                    else {
                        //keep old depth and id
                    }
                }
                else {
                    //save id to pixel
                    pixelToDepth[y][x] = IdAndDepth(p.id, dist);
                }
            }
        }

        //initialize vectors
        depthMap = std::vector<uint32_t>();

        normalMap = std::vector<uint8_t>();

        //decrease minDist, so no pixel with a value that was set above has 0 as value and we know which pixels are invalid
        if (minDist - 0.01 >= 0.0) {
            minDist = minDist - 0.01f;
        }

        //transform distance values to depth values between 0.0 and 1.0 and save depth image data
        for (std::vector<IdAndDepth> vec : pixelToDepth) {
            for (IdAndDepth pixel : vec) {

                if (pixel.id != std::numeric_limits<uint32_t>::max()) {
                    //pixel value was set in the procedure above. At least one pointcloud point could be projected to the pixel

                    //calculate depth value between 0.0 an 1.0
                    float color = (pixel.dist - minDist) / (maxDist - minDist);

                    if (color <= 1.0 && color >= 0.0) {

                        //convert to int32_t
                        uint32_t convertedColor;
                        if (color >= 1.0) {
                            convertedColor = std::numeric_limits<uint32_t>::max();
                        }
                        else {
                            convertedColor = floor(color * 4294967296);
                        }

                        //save depth to depht map
                        depthMap.push_back(convertedColor);

                        //save normal to normal map
                        float normalX = pixel.normal.x;
                        float normalY = pixel.normal.y;
                        float normalZ = pixel.normal.z;

                        //shift to 0-1 range
                        normalX = (normalX * 0.5) + 0.5;
                        normalY = (normalY * 0.5) + 0.5;
                        normalZ = (normalZ * 0.5) + 0.5;

                        //convert to uint_8
                        uint8_t intX = glm::packUnorm1x8(normalX); //convertToUint8(normalX);
                        uint8_t intY = glm::packUnorm1x8(normalY);
                        uint8_t intZ = glm::packUnorm1x8(normalZ);
                        //r = x
                        normalMap.push_back(intX);
                        //g = y
                        normalMap.push_back(intY);
                        //b = z
                        normalMap.push_back(intZ);


                    }
                    else {
                        //invalid value. set pixel to black
                        depthMap.push_back(0);
                        //r = x
                        normalMap.push_back(128);
                        //g = y
                        normalMap.push_back(128);
                        //b = z
                        normalMap.push_back(255);
                    }
                }
                else {
                    //no pixel value was set above. No pointcloud point could be projected to the pixel. Set color to black/0
                    depthMap.push_back(2147483648);
                    //r = x
                    normalMap.push_back(128);
                    //g = y
                    normalMap.push_back(128);
                    //b = z
                    normalMap.push_back(255);
                }

            }
        }

        //maybe improve sparse depth map using these approaches:
        //https://github.com/kujason/ip_basic or https://github.com/BerensRWU/DenseMap

        //calculate normals in new depth values with:
        //https://stackoverflow.com/questions/34644101/calculate-surface-normals-from-depth-image-using-neighboring-pixels-cross-produc
    }

}
