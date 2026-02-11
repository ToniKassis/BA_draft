

#pragma once
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include "Globals.hpp"
#include <thread>
#include <mutex>

#define INSTRUMENTSHAREDMEMORY "InstrumentSharedMemory"
#define INSTRUMENTMUTEXSEMAPHORE "InstrumentMutexSemaphore"
#define INSTRUMENTEMPTYSEMAPHORE "InstrumentEmptySemaphore"
#define INSTRUMENTSTOREDSEMAPHORE "InstrumentStoredSemaphore"

using namespace boost::interprocess;


//! Class for sharing data with other processes through shared memory
class IPCConnectorClientInstrument
{
public:
    typedef allocator<char, segment_manager_t>                           char_allocator;
    typedef basic_string<char, std::char_traits<char>, char_allocator>   char_string;
    typedef allocator<void, segment_manager_t>                           void_allocator;

    struct Instrument {
        char_string name;

        //center point
        float centerX;
        float centerY;
        float centerZ;

        //upper left point
        float upperLeftX;
        float upperLeftY;
        float upperLeftZ;

        //upper right point
        float upperRightX;
        float upperRightY;
        float upperRightZ;

        //lower left point
        float lowerLeftX;
        float lowerLeftY;
        float lowerLeftZ;

        //lower right point
        float lowerRightX;
        float lowerRightY;
        float lowerRightZ;

        float cameraPosX;
        float cameraPosY;
        float cameraPosZ;
    };

    typedef allocator <Instrument, managed_shared_memory::segment_manager> instrument_allocator;
    typedef vector<Instrument, instrument_allocator> InstrumentDataVector;

    struct InstrumentLocal {
        char* name;

        //center point
        glm::vec3 center;

        //upper left point
        glm::vec3 upperLeft;

        //upper right point
        glm::vec3 upperRight;

        //lower left point
        glm::vec3 lowerLeft;

        //lower right point
        glm::vec3 lowerRight;

        //camera position when frame was taken
        glm::vec3 camPos;

    public:
        InstrumentLocal(const char* nameIn, float centerXIn, float centerYIn, float centerZIn,float upperLeftXIn, float upperLeftYIn, float upperLeftZIn, float upperRightXIn, float upperRightYIn, float upperRightZIn, float lowerLeftXIn, float lowerLeftYIn, float lowerLeftZIn, float lowerRightXIn, float lowerRightYIn, float lowerRightZIn, float camPosXIn, float camPosYIn, float camPosZIn)
            :name(nameIn), center(glm::vec3(centerXIn, centerYIn, centerZIn)),upperLeft(glm::vec3(upperLeftXIn, upperLeftYIn, upperLeftZIn)), upperRight(glm::vec3(upperRightXIn, upperRightYIn, upperRightZIn)), lowerLeft(glm::vec3(lowerLeftXIn, lowerLeftYIn, lowerLeftZIn)), lowerRight(glm::vec3(lowerRightXIn, lowerRightYIn, lowerRightZIn), camPos(glm::vec3(camPosXIn, camPosYIn, camPosZIn)
        {}
    };

    //! Construct IPCConnector
    IPCConnectorClientInstrument();

    //! Destruct IPCConnector. Cleans up running threads and shared memory.
    ~IPCConnectorClientInstrument();

    // Disable copy, move and assign
    IPCConnectorClientInstrument(const IPCConnectorClientInstrument& other) = delete;
    IPCConnectorClientInstrument(const IPCConnectorClientInstrument&& other) = delete;
    IPCConnectorClientInstrument& operator=(const IPCConnectorClientInstrument& other) = delete;
    IPCConnectorClientInstrument& operator=(const IPCConnectorClientInstrument&& other) = delete;

    void SetInstrumentDataImport(bool status);

    bool getInstrumentData(std::vector<char*>& names, std::vector<glm::vec3>& centers, std::vector<glm::vec3>& upperLefts, std::vector<glm::vec3>& upperRights, std::vector<glm::vec3>& lowerLefts, std::vector<glm::vec3>& lowerRights, std::vector<glm::vec3>& camPositions);

private:

    void importInstrumentData();

    bool importInstrument = false;

    std::thread importInstrumentThread;

    std::vector<InstrumentLocal> m_LatestInstrumentData;

    mutable std::mutex instrumentMutex;
};
