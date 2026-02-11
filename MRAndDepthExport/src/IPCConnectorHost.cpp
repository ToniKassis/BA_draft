


#include "IPCConnectorHost.hpp"

#include <string>
#include <algorithm>
#include <vector>

IPCConnectorHost::IPCConnectorHost()
{
    //free memory segments in case they were open before the application started.
    shared_memory_object::remove(MRSHAREDMEMORY);
    shared_memory_object::remove(DEPTHSHAREDMEMORY);
    
    //remove semaphores
    named_semaphore::remove(MRMUTEXSEMAPHORE);
    named_semaphore::remove(MREMPTYSEMAPHORE);
    named_semaphore::remove(MRSTOREDSEMAPHORE);

    named_semaphore::remove(DEPTHMUTEXSEMAPHORE);
    named_semaphore::remove(DEPTHEMPTYSEMAPHORE);
    named_semaphore::remove(DEPTHSTOREDSEMAPHORE);

    //create memory segments and semaphores
    initHostMemorySegments();
}

IPCConnectorHost::~IPCConnectorHost()
{
    //wait for threads to end

    exportVSTData = false;
    //wait for thread to join, before memory segments get destroyed
    if (exportVSTThread.joinable()) {
        exportVSTThread.join();
    }

    exportPointCloud = false;
    //wait for thread to join, before memory segments get destroyed
    if (exportPointCloudThread.joinable()) {
        exportPointCloudThread.join();
    }

    //remove semaphores
    named_semaphore::remove(MRMUTEXSEMAPHORE);
    named_semaphore::remove(MREMPTYSEMAPHORE);
    named_semaphore::remove(MRSTOREDSEMAPHORE);

    named_semaphore::remove(DEPTHMUTEXSEMAPHORE);
    named_semaphore::remove(DEPTHEMPTYSEMAPHORE);
    named_semaphore::remove(DEPTHSTOREDSEMAPHORE);

    //free memory segments
    shared_memory_object::remove(MRSHAREDMEMORY);
    shared_memory_object::remove(DEPTHSHAREDMEMORY);
}

void IPCConnectorHost::initHostMemorySegments()
{
    //init VST Frame Shared Memory Segment
    {
        try {
            managed_shared_memory VSTMemorySegment(create_only, MRSHAREDMEMORY, MRSHAREDMEMORYSIZE);
        }
        catch (boost::interprocess::interprocess_exception e) {
            LOG_INFO("VST Shared Memory Segment could not be created. Got Error: %s", e.what());
        }
       

        try {
            //create semaphores for synchronization of processes
            
            //semaphore that handels that only one process at a time can access our VSTData
            named_semaphore MRMutexSemaphore(create_only_t(), MRMUTEXSEMAPHORE, 1);

            //semaphore that holds the number of items that have been emptied from shared memory
            named_semaphore MREmptySemaphore(create_only_t(), MREMPTYSEMAPHORE, 1);

            //semaphore that holds the number of items currently stored in the shared memory
            named_semaphore MRStoredSemaphore(create_only_t(), MRSTOREDSEMAPHORE, 0);

        }
        catch (interprocess_exception e) {
            LOG_WARNING("Could not create VSTFrame Semaphores. Got Error: %s", e.what());
        }
    }

    //init PointCloudData Shared Memory Segment
    {
        try {
            managed_shared_memory(create_only, DEPTHSHAREDMEMORY, DEPTHSHAREDMEMORYSIZE);
            LOG_INFO("Point Cloud Shared Memory Segment created");
        }
        catch (boost::interprocess::interprocess_exception e) {
            LOG_INFO("Point Cloud Shared Memory Segment could not be created. Got Error: %s",e.what());
        }
        

        try {
            //create semaphores for synchronization of processes
            
            //semaphore that handels that only one process at a time can access our PointCloud Data
            named_semaphore MutexSemaphore(create_only_t(), DEPTHMUTEXSEMAPHORE, 1);

            //semaphore that holds the number of items that have been emptied from shared memory
            named_semaphore EmptySemaphore(create_only_t(), DEPTHEMPTYSEMAPHORE, 1);

            //semaphore that holds the number of items currently stored in the shared memory
            named_semaphore StoredSemaphore(create_only_t(), DEPTHSTOREDSEMAPHORE, 0);
        }
        catch (interprocess_exception e) {
            LOG_WARNING("Could not create PointCloud Semaphores. Got Error: %s", e.what());
        }
        
    }

}

void IPCConnectorHost::exportVSTFrame()
{
    //open semaphores
    named_semaphore VSTMutexSemaphore(open_only_t(), MRMUTEXSEMAPHORE);
    named_semaphore VSTEmptySemaphore(open_only_t(), MREMPTYSEMAPHORE);
    named_semaphore VSTStoredSemaphore(open_only_t(), MRSTOREDSEMAPHORE);

    //open VST Memory Segment
    managed_shared_memory VSTMemorySegment(open_only_t(), MRSHAREDMEMORY);

    while (exportVSTData) {
        
        if (VSTEmptySemaphore.try_wait()) {
            if (VSTMutexSemaphore.try_wait()) {
                
                //lock mutex for accessing latest frame
                std::lock_guard<std::mutex> streamLock(VSTFrameMutex);

                if (!m_latestVSTFrame.data_l.empty() && !m_latestVSTFrame.data_r.empty())
                {

                    //check if buffers plus a little safety buffer are not bigger than allocated memory
                    if (sizeof(m_latestVSTFrame.data_l) + sizeof(m_latestVSTFrame.data_r)+ 200  + sizeof(m_latestVSTFrame.depthData_l) + sizeof(m_latestVSTFrame.depthData_r) + sizeof(m_latestVSTFrame.normalMap_l) + sizeof(m_latestVSTFrame.normalMap_r) < MRSHAREDMEMORYSIZE) {
                    
                        //Construct a VSTFrameInfo Object in shared memory, which holds relevant frame info
                        try {
                            VSTMemorySegment.construct<VSTFrameInfo>("VSTFrameInfo")(m_latestVSTFrame.info);
                        }
                        catch (interprocess_exception e) {
                            LOG_INFO("Got Error: %s", e.what());
                        }

                        //Construct array with left eye color frame data
                        uint8_t* VSTArray_l = VSTMemorySegment.construct<uint8_t>("VSTDataArray_l")[m_latestVSTFrame.info.width_l * m_latestVSTFrame.info.height_l * m_latestVSTFrame.info.channels_l]();
                        std::copy(m_latestVSTFrame.data_l.begin(), m_latestVSTFrame.data_l.end(), VSTArray_l);

                        //Construct array with right eye color frame data
                        uint8_t* VSTArray_r = VSTMemorySegment.construct<uint8_t>("VSTDataArray_r")[m_latestVSTFrame.info.width_r * m_latestVSTFrame.info.height_r * m_latestVSTFrame.info.channels_r]();
                        std::copy(m_latestVSTFrame.data_r.begin(), m_latestVSTFrame.data_r.end(), VSTArray_r);

                        if (!m_latestVSTFrame.depthData_l.empty()) {
                            //Construct array with left eye depth data
                            uint32_t* DepthArray_l = VSTMemorySegment.construct<uint32_t>("DepthDataArray_l")[m_latestVSTFrame.info.width_l * m_latestVSTFrame.info.height_l]();
                            std::copy(m_latestVSTFrame.depthData_l.begin(), m_latestVSTFrame.depthData_l.end(), DepthArray_l);
                        }
                        if(!m_latestVSTFrame.depthData_r.empty()){
                            //Construct array with right eye depth data
                            uint32_t* DepthArray_r = VSTMemorySegment.construct<uint32_t>("DepthDataArray_r")[m_latestVSTFrame.info.width_r * m_latestVSTFrame.info.height_r]();
                            std::copy(m_latestVSTFrame.depthData_r.begin(), m_latestVSTFrame.depthData_r.end(), DepthArray_r);
                        }
                        if (!m_latestVSTFrame.normalMap_l.empty()) {
                            //Construct array with left eye normal data
                            uint8_t* NormalArray_l = VSTMemorySegment.construct<uint8_t>("NormalDataArray_l")[m_latestVSTFrame.info.width_l * m_latestVSTFrame.info.height_l * 3]();
                            std::copy(m_latestVSTFrame.normalMap_l.begin(), m_latestVSTFrame.normalMap_l.end(), NormalArray_l);
                        }
                        if (!m_latestVSTFrame.normalMap_r.empty()) {
                            //Construct array with right eye normal data
                            uint8_t* NormalArray_r = VSTMemorySegment.construct<uint8_t>("NormalDataArray_r")[m_latestVSTFrame.info.width_r * m_latestVSTFrame.info.height_r * 3]();
                            std::copy(m_latestVSTFrame.normalMap_r.begin(), m_latestVSTFrame.normalMap_r.end(), NormalArray_r);
                        }

                        //notify client that data is available
                        VSTMutexSemaphore.post();
                        VSTStoredSemaphore.post();
                    }
                    else {
                        LOG_WARNING("Pixel Buffers too big for allocated memory. Skipping frame...");
                        VSTMutexSemaphore.post();
                        VSTEmptySemaphore.post();
                    }
                }
                else {
                    //since no frame is available, increase VSTEmptySemaphore again so there is no deadlock in the next loop
                    VSTMutexSemaphore.post();
                    VSTEmptySemaphore.post();
                }
            }
            else {
                //post Empty Semaphore again, so next loop we can check for both conditions
                VSTEmptySemaphore.post();
            }
        }
        

    }
}

void IPCConnectorHost::exportPointCloudData()
{
    //open semaphores
    named_semaphore PointCloudMutexSemaphore(open_only_t(), DEPTHMUTEXSEMAPHORE);
    named_semaphore PointCloudEmptySemaphore(open_only_t(), DEPTHEMPTYSEMAPHORE);
    named_semaphore PointCloudStoredSemaphore(open_only_t(), DEPTHSTOREDSEMAPHORE);

    //open point cloud memory Segment
    managed_shared_memory PointCloudMemorySegment(open_only_t(), DEPTHSHAREDMEMORY);

    while (exportPointCloud) {
        if (PointCloudEmptySemaphore.try_wait()) {

            if (PointCloudMutexSemaphore.try_wait()) {

                //lock mutex for accessing latest pointcloud data
                std::lock_guard<std::mutex> streamLock(pointCloudMutex);

                //avoid writing empty array to shared memory
                if (m_latestPointCloudContent.pointCount > 0) {

                    //if snapshot contains too much points than allocated memory for, skip snapshot
                    if (m_latestPointCloudContent.pointCount < 5000000) {

                        //compute size of pointcloud array
                        varjo_PointCloudPoint* pointCloudPointArray = m_latestPointCloudContent.points;

                        varjo_PointCloudPoint* pointCloudArrayEnd = pointCloudPointArray + m_latestPointCloudContent.pointCount;

                        //Construct array with pointcloud snapshot points
                        PointCloudPoint* PointCloudArray = PointCloudMemorySegment.construct<PointCloudPoint>("PointCloudDataArray")[m_latestPointCloudContent.pointCount]();
                        
                        //add pointcloud points to array in shared memory
                        for (int i = 0; i < m_latestPointCloudContent.pointCount; i++) {
                            PointCloudArray[i] = m_latestPointCloudContent.points[i];
                        }

                        //notify client that data is available
                        PointCloudMutexSemaphore.post();
                        PointCloudStoredSemaphore.post();
                    }
                    else {
                        LOG_WARNING("Point cloud Snapshot point count too big for allocated memory. Skipping snapshot export...");
                        PointCloudMutexSemaphore.post();
                        PointCloudEmptySemaphore.post();
                    }
                }
                else {
                    //since no data is available, increase PointCloudStoredSemaphore and try again next loop
                    PointCloudMutexSemaphore.post();
                    PointCloudEmptySemaphore.post();
                }
            }
            else {
                //post Empty Semaphore again, so next loop we can check for both conditions
                PointCloudEmptySemaphore.post();
            }
        }
    }
}

void IPCConnectorHost::SetVSTFrameExport(bool status)
{
    if (status && !exportVSTData) {
        //start exporting vst data
        
        //set vst export bool
        exportVSTData = status;

        //start VST export thread
        exportVSTThread = std::thread(&IPCConnectorHost::exportVSTFrame, this);
        LOG_INFO("Started exporting vst data");
    }
    else if (!status && exportVSTData) {
        //stop exporting vst data
        
        //set vst export bool
        exportVSTData = status;

        //join thread if joinable
        if (exportVSTThread.joinable()) {
            exportVSTThread.join();
        }
        LOG_INFO("Stopped exporting vst data");
    }
}

void IPCConnectorHost::SetPointCloudDataExport(bool status)
{
    if (status && !exportPointCloud) {
        //start exporting pointcloud data
        
        //set pointcloud export bool
        exportPointCloud = status;

        //start pointcloud export thread
        exportPointCloudThread = std::thread(&IPCConnectorHost::exportPointCloudData, this);
        LOG_INFO("Started exporting pointcloud data");
    }
    else if (!status && exportPointCloud) {
        //stop exporting pointcloud data

        //set pointcloud export bool
        exportPointCloud = status;

        //join thread if joinable
        if (exportPointCloudThread.joinable()) {
            exportPointCloudThread.join();
        }
        
    }
}

void IPCConnectorHost::updateColorFrameWithDepthAndNormals(int ch, const glm::ivec2& resolution, varjo_TextureFormat format, int color_channels, const std::vector<uint8_t>& data, varjo_CameraIntrinsics intrinsics, varjo_Matrix extrinsics, varjo_Matrix HMDPose, const std::vector<uint32_t>& depthData, float minDepth, float maxDepth, const std::vector<uint8_t>& normalMap)
{
    //update frame to export here
    if (format == varjo_TextureFormat_R8G8B8A8_UNORM) {
        //only update if format is R8G8B8A8_UNORM

        //lock mutex for accessing latest frame
        std::lock_guard<std::mutex> streamLock(VSTFrameMutex);

        if (ch == 0) {
            //left channel

            //update frame info
            m_latestVSTFrame.info.width_l = resolution.x;

            m_latestVSTFrame.info.height_l = resolution.y;

            m_latestVSTFrame.info.channels_l = color_channels;

            m_latestVSTFrame.info.intrinsics_l = Intrinsics(intrinsics.principalPointX, intrinsics.principalPointY, intrinsics.focalLengthX, intrinsics.focalLengthY, intrinsics.distortionCoefficients);

            std::copy(extrinsics.value, extrinsics.value + 16, m_latestVSTFrame.info.extrinsics_l);

            std::copy(HMDPose.value, HMDPose.value + 16, m_latestVSTFrame.info.HMDPose_l);

            //update color frame
            m_latestVSTFrame.data_l = data;

            //update depth data
            m_latestVSTFrame.depthData_l = depthData;

            m_latestVSTFrame.info.minDepth_l = minDepth;

            m_latestVSTFrame.info.maxDepth_l = maxDepth;

            //update normal data
            m_latestVSTFrame.normalMap_l = normalMap;

        }
        else if (ch == 1) {
            //right channel

            //update frame info
            m_latestVSTFrame.info.width_r = resolution.x;

            m_latestVSTFrame.info.height_r = resolution.y;

            m_latestVSTFrame.info.channels_r = color_channels;

            m_latestVSTFrame.info.intrinsics_r = Intrinsics(intrinsics.principalPointX, intrinsics.principalPointY, intrinsics.focalLengthX, intrinsics.focalLengthY, intrinsics.distortionCoefficients);

            std::copy(extrinsics.value, extrinsics.value + 16, m_latestVSTFrame.info.extrinsics_r);

            std::copy(HMDPose.value, HMDPose.value + 16, m_latestVSTFrame.info.HMDPose_r);

            //update color frame
            m_latestVSTFrame.data_r = data;

            //update depth data
            m_latestVSTFrame.depthData_r = depthData;

            m_latestVSTFrame.info.minDepth_r = minDepth;

            m_latestVSTFrame.info.maxDepth_r = maxDepth;

            //update normal data
            m_latestVSTFrame.normalMap_r = normalMap;
        }
    }
}

void IPCConnectorHost::updatePointCloudSnapshotContent(const varjo_PointCloudSnapshotContent& content)
{
    //lock mutex for accessing latest pointcloud snapshot
    std::lock_guard<std::mutex> streamLock(pointCloudMutex);

    //update snapshot
    m_latestPointCloudContent = content;
}
