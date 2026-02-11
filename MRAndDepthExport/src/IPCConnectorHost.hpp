

#pragma once
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include "Globals.hpp"
#include <thread>
#include <mutex>
#include <Varjo_mr_experimental.h>


#define MRSHAREDMEMORY "MRSharedMemory"
#define MRMUTEXSEMAPHORE "MRMutexSemaphore"
#define MREMPTYSEMAPHORE "MREmptySemaphore"
#define MRSTOREDSEMAPHORE "MRStoredSemaphore"

#define DEPTHSHAREDMEMORY "DepthSharedMemory"
#define DEPTHMUTEXSEMAPHORE "DepthMutexSemaphore"
#define DEPTHEMPTYSEMAPHORE "DepthEmptySemaphore"
#define DEPTHSTOREDSEMAPHORE "DepthStoredSemaphore"

//set size to roughly the image size times two: 2000 Pixels * 2000 Pixels * 2 * 4 Channels * 1 Byte per Channel = 32.000.000 Byte
//add depth image size: 1200 PixelsW * 1200 PixelsH * 2 Frames * 1 Channel * 8 Byte per Channel = 23.040.000 Byte
//add normal map size: 1200 PixelsW * 1200 PixelsH * 2 Frames * 3 Channel * 1 Byte per Channel = 8.640.000 Byte
//Sum: 32.000.000 + 23.040.000 + 8.640.000 = 63.680.000
#define MRSHAREDMEMORYSIZE 636800000

//set size to 5000000 points times the data needed per point: 5000000 Points * 6 Values per Point * 4 Byte per Value = 120000000
#define DEPTHSHAREDMEMORYSIZE 1200000000

using namespace boost::interprocess;
using namespace VarjoExamples;


/**
 * Exports VST Frames and point cloud data to shared memory.
 *
 * This class utilized the boost interprocess header to achieve this.
 * Tow shared memory regions are created for exporting each data.
 * Each memory region has semaphores to control access to the regionand prevent multiple processes accessing the same region.
 * The mutex semaphore assures that only one process accesses the memory region at any given time.
 * The stored semaphore signals the client, that an object is stored in the shared memory.
 * The empty semaphore signals to the host, that the shared memory region is empty.
 * Each data type we receive has its own thread that manages the data import, concurrent to the game thread.
 * The latest data is set with the defined methods. To prevent simultaneous access to the variable storing the latest data, we employ one mutex per thread.
 *
 */
class IPCConnectorHost
{
public:

    /**
     * The camera intrinsics as they are stored in shared memory.
     */
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

    /**
     * The VSTFrameInfo class as it is stored in shared memory
     * Contains important information for processing the data arrays of the frames.
     * The data provided can be separated in left frame and right frame data, marked by _l or _r.
     */
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
        VSTFrameInfo(int width_left, int height_left, int channels_left, Intrinsics intrinsics_left, double* extrinsics_left, double* HMDPose_left,float minDepth_left, float maxDepth_left, int width_right, int height_right, int channels_right, Intrinsics intrinsics_right, double* extrinsics_right, double* HMDPose_right, float minDepth_right, float maxDepth_right)
            :width_l(width_left), height_l(height_left), channels_l(channels_left), intrinsics_l(intrinsics_left),minDepth_l(minDepth_left),maxDepth_l(maxDepth_left), width_r(width_right), height_r(height_right), channels_r(channels_right), intrinsics_r(intrinsics_right),minDepth_r(minDepth_right),maxDepth_r(maxDepth_right)
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
            :width_l(other.width_l), height_l(other.height_l), channels_l(other.channels_l), intrinsics_l(other.intrinsics_l),minDepth_l(other.minDepth_l),maxDepth_l(other.maxDepth_l), width_r(other.width_r), height_r(other.height_r), channels_r(other.channels_r), intrinsics_r(other.intrinsics_r), minDepth_r(other.minDepth_r),maxDepth_r(other.maxDepth_r)
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
     * VST frame data for both eyes.
     * Contains color data arrays, depth data arrays and normal maps of each left and right frame. Also includes frame info.
     */
    struct VSTFrame {
        VSTFrameInfo info;
        std::vector<uint8_t> data_l;
        std::vector<uint8_t> data_r;
        std::vector<uint32_t> depthData_l;
        std::vector<uint32_t> depthData_r;
        std::vector<uint8_t> normalMap_l;
        std::vector<uint8_t> normalMap_r;
    public:
        VSTFrame() {};
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
    public:
        PointCloudPoint(const varjo_PointCloudPoint& other)
            :indexConfidence(other.indexConfidence), normalXY(other.normalXY), normalZcolorR(other.normalZcolorR), colorBG(other.colorBG), positionXY(other.positionXY), positionZradius(other.positionZradius)
        {}
        PointCloudPoint() {};
    };


    /**
     * Construct IPCConnectorHost
     */
    IPCConnectorHost();

    /**
     * Destruct IPCConnectorHost. Cleans up running threads and shared memory.
     */
    ~IPCConnectorHost();

    /// Disable copy, move and assign
    IPCConnectorHost(const IPCConnectorHost& other) = delete;
    IPCConnectorHost(const IPCConnectorHost&& other) = delete;
    IPCConnectorHost& operator=(const IPCConnectorHost& other) = delete;
    IPCConnectorHost& operator=(const IPCConnectorHost&& other) = delete;

    /**
     * Enables or disables export of VST frames based on 'status'
     * Start or ends exportVSTThread.
     *
     * @param status - Indicator if disabling or enabling VST frame export.
     */
    void SetVSTFrameExport(bool status);

    /**
     * Enables or disables export of pointcloud data based on 'status'
     * Start or ends exportPointCloudThread.
     *
     * @param status - Indicator if disabling or enabling pointcloud data export.
     */
    void SetPointCloudDataExport(bool status);

    /**
     * Updates the data of the latest VST camera image for a given channel. This frame is then exported.
     * 
     * @param ch - The channel of the frame to update (0 for left VST image, 1 for right VST image).
     * @param resolution - The resolution of the image as ivec2 (first value width, second height)
     * @param format - The varjo color format of the image. Only varjo_TextureFormat_R8G8B8A8_UNORM is supported.
     * @param color_channels - The color channels of the new frame data.
     * @param data - Data vector containing image in bytes.
     * @param intrinsics - Intrinsic camera paramters of the camera that took the frame.
     * @param extrinsics - Extrinsic camera paramters of the camera that took the frame.
     * @param HMDPose - HMD Pose when the image was created.
     * @param depthData - Data vector containing the depth for each color image pixel.
     * @param minDepth - The minimum depth in the depth image.
     * @param maxDepth - The maximum depth in the depth image.
     * @param normalMap - Data vector containing the normal for each color image pixel.
     */
    void updateColorFrameWithDepthAndNormals(int ch, const glm::ivec2& resolution, varjo_TextureFormat format, int color_channels, const std::vector<uint8_t>& data, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, varjo_Matrix HMDPose, const std::vector<uint32_t>& depthData, float minDepth, float maxDepth, const std::vector<uint8_t>& normalMap);

    /**
     * Updates the pointcloud data to the latest pointcloud snapshot.
     * 
     * @param content - The varjo_PointCloudSnapshotContent of the latest pointcloud snapshot.
     * 
     */
    void updatePointCloudSnapshotContent(const varjo_PointCloudSnapshotContent& content);

private:
    /**
     * Initialized and creates memory segments and semaphores for VST and pointcloud export.
     */
    void initHostMemorySegments();

    /**
     * Method that exports the VST frames.
     * Executed by exportVSTThread as long as exportVSTData is true.
     * First tries to open VST memory segments, then opens semaphores and writes the latest VST frame in shared memory if the shared memory is empty.
     * Aborts export process if any boost error is thrown.
     *
     */
    void exportVSTFrame();

    /**
     * Method that exports the pointcloud data.
     * Executed by exportsPointCloudThread as long as exportPointCloud is true.
     * First tries to open Pointcloud memory segments, then opens semaphores and writes the latest pointcloud snapshot in shared memory if the shared memory is empty.
     * Aborts export process if any boost error is thrown.
     */
    void exportPointCloudData();
private:

    ///Indicator if VST data is currently exported.
    bool exportVSTData = false;

    ///Indicator if pointcloud data is currently exported.
    bool exportPointCloud = false;

    ///Thread that exports VST frames.
    std::thread exportVSTThread;

    ///Thread that exports pointcloud arrays.
    std::thread exportPointCloudThread;

    ///Contains the latest VST frame for export to shared memory.
    VSTFrame m_latestVSTFrame;

    ///Contains the latest pointcloud snapshot for export to shared memory.
    varjo_PointCloudSnapshotContent m_latestPointCloudContent;

    ///Mutex for avoiding concurrent access to m_LatestVSTFrame from thread and in updateColorFrameWithDepthAndNormals.
    mutable std::mutex VSTFrameMutex;
    
    ///Mutex for avoiding concurrent access to m_LatestPointCloudSnapshotContent from thread and in updatePointCloudSnapshotContent.
    mutable std::mutex pointCloudMutex;
};
