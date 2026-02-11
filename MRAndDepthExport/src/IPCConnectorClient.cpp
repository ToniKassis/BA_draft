


#include "IPCConnectorClient.hpp"

#include <string>
#include <algorithm>
#include <vector>

IPCConnectorClient::IPCConnectorClient()
{
}

IPCConnectorClient::~IPCConnectorClient()
{
    //wait for threads to end
    importVSTData = false;
    //wait for thread to join, before memory segments get destroyed
    if (importVSTThread.joinable()) {
        importVSTThread.join();
    }

    importPointCloud = false;
    //wait for thread to join, before memory segments get destroyed
    if (importPointCloudThread.joinable()) {
        importPointCloudThread.join();
    }
}

void IPCConnectorClient::importVSTFrame()
{
    try {
        //open semaphores
        //semaphore that handels that only one process at a time can access our 
        named_semaphore VSTMutexSemaphore(open_only_t(), MRMUTEXSEMAPHORE);
        //semaphore that holds the number of items that have been emptied from shared memory
        named_semaphore VSTEmptySemaphore(open_only_t(), MREMPTYSEMAPHORE);
        //semaphore that holds the number of items currently stored in the shared memory
        named_semaphore VSTStoredSemaphore(open_only_t(), MRSTOREDSEMAPHORE);
        std::cout << "VST Semaphores opened" << std::endl;

        //open VST Memory Segment
        managed_shared_memory VSTMemorySegment(open_only_t(), MRSHAREDMEMORY);
        std::cout << "VST Shared Memory Segment opened" << std::endl;

        while (importVSTData) {

            if (VSTStoredSemaphore.try_wait()) {
                if (VSTMutexSemaphore.try_wait()) {

                    //Find frame info object
                    VSTFrameInfo* info = VSTMemorySegment.find<VSTFrameInfo>("VSTFrameInfo").first;

                    //Find the vector with the left eye frame data using the c-string name
                    std::pair<uint8_t*, std::size_t> VSTArray_l = VSTMemorySegment.find<uint8_t>("VSTDataArray_l");

                    //Find the vector with the right eye frame data using the c-string name
                    std::pair<uint8_t*, std::size_t> VSTArray_r = VSTMemorySegment.find<uint8_t>("VSTDataArray_r");

                    //check if we found something
                    if (info != 0 && VSTArray_l.first != 0 && VSTArray_r.first != 0) {

                        //Prevent acces on m_LatestVSTFrame from different thread
                        std::lock_guard<std::recursive_mutex> streamLock(VSTFrameMutex);

                        //update current frame

                        //copy frame info
                        m_LatestVSTFrame.info = VSTFrameInfo(*info);

                        //copy left data
                        if (VSTArray_l.second > 0) {
                            //copy vector at this point, to prevent long intervals between frames due to further processing of the received data.
                            m_LatestVSTFrame.data_l = std::vector<uint8_t>(VSTArray_l.first, VSTArray_l.first + VSTArray_l.second);
                        }
                        else {
                            //keep old data?
                        }

                        //copy right data
                        if (VSTArray_r.second > 0) {
                            //copy vector at this point, to prevent long intervals between frames due to further processing of the received data.
                            m_LatestVSTFrame.data_r = std::vector<uint8_t>(VSTArray_r.first, VSTArray_r.first + VSTArray_r.second);
                        }
                        else {
                            //keep old data?
                        }
                    }

                    //When done, destroy the vectors from the segment
                    VSTMemorySegment.destroy<uint8_t>("VSTDataArray_l");
                    VSTMemorySegment.destroy<uint8_t>("VSTDataArray_r");

                    VSTMemorySegment.destroy<VSTFrameInfo>("VSTFrameInfo");

                    VSTMemorySegment.destroy<uint32_t>("DepthDataArray_l");
                    VSTMemorySegment.destroy<uint32_t>("DepthDataArray_r");

                    VSTMutexSemaphore.post();
                    VSTEmptySemaphore.post();
                }
                else {
                    //post Stored Semaphore again, so next loop we can check for both conditions
                    VSTStoredSemaphore.post();
                }
            }
        }
    }
    catch (boost::interprocess::interprocess_exception e) {
        importVSTData = false;
        std::cout << "Error when importing VST Frame. Stopping import. Exception: " << e.what() << std::endl;
    }
}

void IPCConnectorClient::importPointCloudData()
{
    try {
        //open semaphores
        //semaphore that handels that only one process at a time can access our 
        named_semaphore PointCloudMutexSemaphore(open_only_t(), DEPTHMUTEXSEMAPHORE);
        //semaphore that holds the number of items that have been emptied from shared memory
        named_semaphore PointCloudEmptySemaphore(open_only_t(), DEPTHEMPTYSEMAPHORE);
        //semaphore that holds the number of items currently stored in the shared memory
        named_semaphore PointCloudStoredSemaphore(open_only_t(), DEPTHSTOREDSEMAPHORE);
        std::cout << "PointCloud Semaphores opened" << std::endl;

        //open VST Memory Segment
        managed_shared_memory PointCloudMemorySegment(open_only_t(), DEPTHSHAREDMEMORY);
        std::cout << "PointCloud Shared Memory Segment opened" << std::endl;

        while (importPointCloud) {
            if (PointCloudStoredSemaphore.try_wait()) {
                if (PointCloudMutexSemaphore.try_wait()) {

                    //Find the vector with the left eye frame data using the c-string name
                    std::pair<PointCloudPoint*, std::size_t> PointCloudArray = PointCloudMemorySegment.find<PointCloudPoint>("PointCloudDataArray");

                    //check if we found something
                    if (PointCloudArray.first != 0) {

                        //Prevent acces on m_LatestPointCloudSnapshotContent from different thread
                        std::lock_guard<std::recursive_mutex> streamLock(pointCloudMutex);

                        // update snapshot content

                        //copy vector at this point, to prevent long intervals between point cloud data due to further processing of the received data.
                        if (PointCloudArray.second > 0) {
                            m_LatestPointCloudSnapshotContent.points = std::vector<PointCloudPoint>(PointCloudArray.first, PointCloudArray.first + PointCloudArray.second);
                        }
                        else {
                            //keep old points?
                        }

                        m_LatestPointCloudSnapshotContent.pointCount = PointCloudArray.second;
                        m_LatestPointCloudSnapshotContent.timeStamp = 0;
                    }

                    //When done, destroy the vector from the segment
                    PointCloudMemorySegment.destroy<PointCloudPoint>("PointCloudDataArray");

                    PointCloudMutexSemaphore.post();
                    PointCloudEmptySemaphore.post();
                }
                else {
                    //post Stored Semaphore again, so next loop we can check for both conditions
                    PointCloudStoredSemaphore.post();
                }
            }
        }
    }
    catch (boost::interprocess::interprocess_exception e) {
        importPointCloud = false;
        std::cout << "Error when importing Point Cloud data. Stopping import. Exception: " << e.what() << std::endl;
    }
}

void IPCConnectorClient::SetVSTFrameImport(bool status)
{
    if (status && !importVSTData) {
        //start importing vst data

        //set vst import bool
        importVSTData = status;

        //start VST import thread
        importVSTThread = std::thread(&IPCConnectorClient::importVSTFrame, this);
    }
    else if (!status && importVSTData) {
        //stop importing vst data

        //set vst import bool
        importVSTData = status;

        //join thread if joinable
        if (importVSTThread.joinable()) {
            importVSTThread.join();
        }
    }
}

void IPCConnectorClient::SetPointCloudDataImport(bool status)
{
    if (status && !importPointCloud) {
        //start importing pointcloud data

        //set pointcloud import bool
        importPointCloud = status;

        //start pointcloud import thread
        importPointCloudThread = std::thread(&IPCConnectorClient::importPointCloudData, this);
    }
    else if (!status && importPointCloud) {
        //stop importing pointcloud data

        //set pointcloud import bool
        importPointCloud = status;

        //join thread if joinable
        if (importPointCloudThread.joinable()) {
            importPointCloudThread.join();
        }
    }
}

bool IPCConnectorClient::getColorFrame(int ch, std::pair<int, int>& resolution, int& channels, std::vector<uint8_t>& data, double& principalPointX, double& principalPointY, double& focalLengthX, double& focalLengthY, double* extrinsics, double* HMDPose)
{
    if (!importVSTData) {
        return false;
    }

    //Prevent acces on m_LatestVSTFrame from different thread
    std::lock_guard<std::recursive_mutex> VSTLock(VSTFrameMutex);

    if (m_LatestVSTFrame.data_l.empty() || m_LatestVSTFrame.data_r.empty()) {
        return false;
    }

    if (ch == 0) {
        //return left image
        resolution = std::pair<int, int>(m_LatestVSTFrame.info.width_l, m_LatestVSTFrame.info.height_l);
        channels = m_LatestVSTFrame.info.channels_l;
        data = m_LatestVSTFrame.data_l;
        principalPointX = m_LatestVSTFrame.info.intrinsics_l.principalPointX;
        principalPointY = m_LatestVSTFrame.info.intrinsics_l.principalPointY;
        focalLengthX = m_LatestVSTFrame.info.intrinsics_l.focalLengthX;
        focalLengthY = m_LatestVSTFrame.info.intrinsics_l.focalLengthY;
        std::copy(m_LatestVSTFrame.info.extrinsics_l, m_LatestVSTFrame.info.extrinsics_l + 16, extrinsics);
        std::copy(m_LatestVSTFrame.info.HMDPose_l, m_LatestVSTFrame.info.HMDPose_l + 16, HMDPose);
        return true;
    }
    else if (ch == 1) {
        //return right image
        resolution = std::pair<int, int>(m_LatestVSTFrame.info.width_r, m_LatestVSTFrame.info.height_r);
        channels = m_LatestVSTFrame.info.channels_r;
        data = m_LatestVSTFrame.data_r;
        principalPointX = m_LatestVSTFrame.info.intrinsics_r.principalPointX;
        principalPointY = m_LatestVSTFrame.info.intrinsics_r.principalPointY;
        focalLengthX = m_LatestVSTFrame.info.intrinsics_r.focalLengthX;
        focalLengthY = m_LatestVSTFrame.info.intrinsics_r.focalLengthY;
        std::copy(m_LatestVSTFrame.info.extrinsics_r, m_LatestVSTFrame.info.extrinsics_r + 16, extrinsics);
        std::copy(m_LatestVSTFrame.info.HMDPose_r, m_LatestVSTFrame.info.HMDPose_r + 16, HMDPose);
        return true;
    }
    return false;
}

bool IPCConnectorClient::getPointCloudPoints(PointCloudSnapshotContent& content)
{
    if (!importPointCloud) {
        return false;
    }

    //Prevent acces on m_LatestPointCloudSnapshotContent from different thread
    std::lock_guard<std::recursive_mutex> streamLock(pointCloudMutex);

    if (m_LatestPointCloudSnapshotContent.points.empty()) {
        return false;
    }

    content = m_LatestPointCloudSnapshotContent;
    return true;
}
