


#include "IPCConnectorClientInstrument.hpp"

#include <string>
#include <algorithm>
#include <vector>

IPCConnectorClientInstrument::IPCConnectorClientInstrument()
{
}

IPCConnectorClientInstrument::~IPCConnectorClientInstrument()
{
    //wait for threads to end
    importInstrument = false;
    //wait for thread to join, before memory segments get destroyed
    if (importInstrumentThread.joinable()) {
        importInstrumentThread.join();
    }
}

void IPCConnectorClientInstrument::importInstrumentData()
{
    try {
        //open semaphores
        //semaphore that handels that only one process at a time can access our 
        named_semaphore InstrumentMutexSemaphore(open_only_t(), INSTRUMENTMUTEXSEMAPHORE);
        //semaphore that holds the number of items that have been emptied from shared memory
        named_semaphore InstrumentEmptySemaphore(open_only_t(), INSTRUMENTEMPTYSEMAPHORE);
        //semaphore that holds the number of items currently stored in the shared memory
        named_semaphore InstrumentStoredSemaphore(open_only_t(), INSTRUMENTSTOREDSEMAPHORE);
        std::cout << "Instrument Semaphores opened" << std::endl;

        //open instrument Memory Segment
        managed_shared_memory InstrumentMemorySegment(open_only_t(), InstrumentSHAREDMEMORY);
        std::cout << "Instrument Shared Memory Segment opened" << std::endl;

        while (importInstrument) {

            if (InstrumentStoredSemaphore.try_wait()) {
                if (InstrumentMutexSemaphore.try_wait()) {


                    //Find the vector with the right eye frame data using the c-string name
                    InstrumentDataVector* InstrumentVector = InstrumentMemorySegment.find<InstrumentDataVector>("InstrumentDataVector").first;

                    //check if we found something
                    if (InstrumentVector != 0) {

                        //Prevent acces on m_LatestInstrumentFrame from different thread
                        std::lock_guard<std::recursive_mutex> streamLock(instrumentMutex);


                        //copy instrument data
                        if (!InstrumentVector->empty()) {
                            //copy vector at this point, to prevent long intervals between frames due to further processing of the received data.
                            for(int i = 0; i < InstrumentVector->size(); i++)
                            m_LatestInstrumentData.push_back(InstrumentLocal(InstrumentVector[i].name, InstrumentVector[i].centerX, InstrumentVector[i].centerY, InstrumentVector[i].centerZ, InstrumentVector[i].upperLeftX, InstrumentVector[i].upperLeftY, InstrumentVector[i].upperLeftZ, InstrumentVector[i].upperRightX, InstrumentVector[i].upperRightY, InstrumentVector[i].upperRightZ, InstrumentVector[i].lowerLeftX, InstrumentVector[i].lowerLeftY, InstrumentVector[i].lowerLeftZ, InstrumentVector[i].lowerRightX, InstrumentVector[i].lowerRightY, InstrumentVector[i].lowerRightZ, InstrumentVector[i].cameraPosX, InstrumentVector[i].cameraPosY, InstrumentVector[i].cameraPosZ))
                        }
                        else {
                            //keep old data?
                        }
                    }

                    //When done, destroy the vectors from the segment
                    InstrumentMemorySegment.destroy<InstrumentDataVector>("InstrumentDataVector");

                    InstrumentMutexSemaphore.post();
                    InstrumentEmptySemaphore.post();
                }
                else {
                    //post Stored Semaphore again, so next loop we can check for both conditions
                    InstrumentStoredSemaphore.post();
                }
            }
        }
    }
    catch (boost::interprocess::interprocess_exception e) {
        importInstrument = false;
        std::cout << "Error when importing instrument data. Stopping import. Exception: " << e.what() << std::endl;
    }
}

void IPCConnectorClientInstrument::SetInstrumentDataImport(bool status)
{
    if (status && !importInstrument) {
        //start importing instrument data

        //set instrument import bool
        importInstrument = status;

        //start instrument import thread
        importInstrumentThread = std::thread(&IPCConnectorClientInstrument::importInstrumentData, this);
    }
    else if (!status && importInstrument) {
        //stop importing instrument data

        //set instrument import bool
        importInstrument = status;

        //join thread if joinable
        if (importInstrumentThread.joinable()) {
            importInstrumentThread.join();
        }
    }
}

bool IPCConnectorClientInstrument::getInstrumentData(std::vector<char*>& names, std::vector<glm::vec3>& centers, std::vector<glm::vec3>& upperLefts, std::vector<glm::vec3>& upperRights, std::vector<glm::vec3>& lowerLefts, std::vector<glm::vec3>& lowerRights, std::vector<glm::vec3>&camPositions) {
    if (!importInstrument) {
        return false;
    }
    //Prevent acces on m_LatestPointCloudSnapshotContent from different thread
    std::lock_guard<std::mutex> streamLock(instrumentMutex);

    if (m_LatestInstrumentData.empty()) {
        return false;
    }

    names.clear();
    centers.clear();
    upperLefts.clear();
    upperRights.clear();
    lowerLefts.clear();
    lowerRights.clear();
    camPositions.clear();

    //return data in some kind of structure
    for (InstrumentLocal instrument : m_LatestInstrumentData) {
        names.push_back(instrument.name);
        centers.push_back(instrument.center);
        upperLefts.push_back(instrument.upperLeft);
        upperRights.push_back(instrument.upperRight);
        lowerLefts.push_back(instrument.lowerLeft);
        lowerRights.push_back(instrument.lowerRight);
        camPositions.push_back(instrument.camPos);
    }

    return true;
}
