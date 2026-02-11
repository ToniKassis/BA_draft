

#pragma once
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include "Globals.hpp"
#include <thread>
#include <mutex>


#define MRSHAREDMEMORY "MRSharedMemory"
#define MRMUTEXSEMAPHORE "MRMutexSemaphore"
#define MREMPTYSEMAPHORE "MREmptySemaphore"
#define MRSTOREDSEMAPHORE "MRStoredSemaphore"

#define DEPTHSHAREDMEMORY "DepthSharedMemory"
#define DEPTHMUTEXSEMAPHORE "DepthMutexSemaphore"
#define DEPTHEMPTYSEMAPHORE "DepthEmptySemaphore"
#define DEPTHSTOREDSEMAPHORE "DepthStoredSemaphore"

using namespace boost::interprocess;


//! Class for sharing data with other processes through shared memory
class IPCConnectorClient
{
public:
    struct Intrinsics {
        double principalPointX;            //!< Camera principal point X.
        double principalPointY;            //!< Camera principal point Y.
        double focalLengthX;               //!< Camera focal length X.
        double focalLengthY;               //!< Camera focal length Y.
        double distortionCoefficients[6];  //!< Intrinsics model coefficients. For omnidir: 2 radial, skew, xi, 2 tangential.
    public:
        Intrinsics(double ppX, double ppY, double focLX, double focLY, double* distCoeff)
            :principalPointX(ppX), principalPointY(ppY), focalLengthX(focLX), focalLengthY(focLY)
        {
            std::copy(distCoeff, distCoeff + 6, distortionCoefficients);
        };
        Intrinsics(const Intrinsics& other)
            :principalPointX(other.principalPointX), principalPointY(other.principalPointY), focalLengthX(other.focalLengthX), focalLengthY(other.focalLengthY)
        {
            std::copy(other.distortionCoefficients, other.distortionCoefficients + 6, distortionCoefficients);
        };
        Intrinsics() { };
    };


    struct VSTFrameInfo {
        int width_l;
        int height_l;
        int channels_l;
        Intrinsics intrinsics_l;
        double extrinsics_l[16];
        double HMDPose_l[16];
        float minDepth_l;
        float maxDepth_l;
        int width_r;
        int height_r;
        int channels_r;
        Intrinsics intrinsics_r;
        double extrinsics_r[16];
        double HMDPose_r[16];
        float minDepth_r;
        float maxDepth_r;
    public:
        VSTFrameInfo(int width_left, int height_left, int channels_left, Intrinsics intrinsics_left, double* extrinsics_left, double* HMDPose_left, float minDepth_left, float maxDepth_left, int width_right, int height_right, int channels_right, Intrinsics intrinsics_right, double* extrinsics_right, double* HMDPose_right, float minDepth_right, float maxDepth_right)
            :width_l(width_left), height_l(height_left), channels_l(channels_left), intrinsics_l(intrinsics_left), minDepth_l(minDepth_left), maxDepth_l(maxDepth_left), width_r(width_right), height_r(height_right), channels_r(channels_right), intrinsics_r(intrinsics_right), minDepth_r(minDepth_right), maxDepth_r(maxDepth_right)
        {
            //copy left extrinsic matrix
            std::copy(extrinsics_left, extrinsics_left + 16, extrinsics_l);

            //copy right extrinsic matrix
            std::copy(extrinsics_right, extrinsics_right + 16, extrinsics_r);

            //copy left HMD pose matrix
            std::copy(HMDPose_left, HMDPose_left + 16, HMDPose_l);

            //copy right HMD pose matrix
            std::copy(HMDPose_right, HMDPose_right + 16, HMDPose_r);
        };
        VSTFrameInfo(const VSTFrameInfo& other)
            :width_l(other.width_l), height_l(other.height_l), channels_l(other.channels_l), intrinsics_l(other.intrinsics_l), minDepth_l(other.minDepth_l), maxDepth_l(other.maxDepth_l), width_r(other.width_r), height_r(other.height_r), channels_r(other.channels_r), intrinsics_r(other.intrinsics_r), minDepth_r(other.minDepth_r), maxDepth_r(other.maxDepth_r)
        {
            //copy left extrinsic matrix
            std::copy(other.extrinsics_l, other.extrinsics_l + 16, extrinsics_l);

            //copy right extrinsic matrix
            std::copy(other.extrinsics_r, other.extrinsics_r + 16, extrinsics_r);

            //copy left HMD pose matrix
            std::copy(other.HMDPose_l, other.HMDPose_l + 16, HMDPose_l);

            //copy right HMD pose matrix
            std::copy(other.HMDPose_r, other.HMDPose_r + 16, HMDPose_r);
        }
        VSTFrameInfo() {};
    };

     /**
     * Data point which belongs to the 3D reconstruction. Contains the following fields in a packed format:
     *
     * Index: globally unique identifier for the point.
     * Confidence: Integer confidence value for the point. Points with confidence of 0 are to be considered removed.
     * Normal: Unit normal in world coordinates.
     * Color: RGB color of the point.
     * Position: position relative to the HMD tracking origin in meters.
     * Radius: Radius of the point in meters.
     */
    struct PointCloudPoint {
        uint32_t indexConfidence;  //!< (index << 8) | (confidence & 0xFF);
        uint32_t normalXY;         //!< float16 / float16
        uint32_t normalZcolorR;    //!< float16 / float16
        uint32_t colorBG;          //!< float16 / float16
        uint32_t positionXY;       //!< float16 / float16
        uint32_t positionZradius;  //!< float16 / float16
    };

    //! VST frame data for both eyes
    struct VSTFrame {
        VSTFrameInfo info;
        std::vector<uint8_t> data_l;
        std::vector<uint8_t> data_r;
        std::vector<uint32_t> depthData_l;
        std::vector<uint32_t> depthData_r;
    public:
        VSTFrame() {};
    };

    struct PointCloudSnapshotContent {
        std::vector<PointCloudPoint> points;
        int32_t pointCount;
        int timeStamp;
    };


    //! Construct IPCConnector
    IPCConnectorClient();

    //! Destruct IPCConnector. Cleans up running threads and shared memory.
    ~IPCConnectorClient();

    // Disable copy, move and assign
    IPCConnectorClient(const IPCConnectorClient& other) = delete;
    IPCConnectorClient(const IPCConnectorClient&& other) = delete;
    IPCConnectorClient& operator=(const IPCConnectorClient& other) = delete;
    IPCConnectorClient& operator=(const IPCConnectorClient&& other) = delete;

    void SetVSTFrameImport(bool status);

    void SetPointCloudDataImport(bool status);

    bool getColorFrame(int ch, std::pair<int, int>& resolution, int& channels, std::vector<uint8_t>& data, double& principalPointX, double& principalPointY, double& focalLengthX, double& focalLengthY, double* extrinsics, double* HMDPose);

    bool getPointCloudPoints(PointCloudSnapshotContent& content);

private:

    void importVSTFrame();

    void importPointCloudData();
private:

    bool importVSTData = false;
    bool importPointCloud = false;

    std::thread importVSTThread;

    std::thread importPointCloudThread;

    PointCloudSnapshotContent m_LatestPointCloudSnapshotContent;

    VSTFrame m_LatestVSTFrame;

    mutable std::recursive_mutex pointCloudMutex;

    mutable std::recursive_mutex VSTFrameMutex;
};
