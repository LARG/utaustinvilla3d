#include "SharedMemory.h"

#include <memory/FrameInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>

//#include <iostream>
//using namespace std;

const int SharedMemory::MEMORY_SIZE = 2000000;
const char SharedMemory::SHARED_MEMORY_NAME[] = "VillaSharedMemory";
const char SharedMemory::MEM_MAP_NAME[] = "VillaMemMap";

SharedMemory::SharedMemory(bool server):
    server_(server)
{
    if (server_) {
        // remove the shared memory
        boost::interprocess::shared_memory_object::remove(SHARED_MEMORY_NAME);
        mem_segment_ = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,SHARED_MEMORY_NAME,MEMORY_SIZE);
        VoidAllocator alloc_inst(mem_segment_->get_segment_manager());
        blocks_ = mem_segment_->construct<SharedMemMap>(MEM_MAP_NAME)(std::less<SharedString>(),alloc_inst);
    }
    else {
//  std::cerr << "BAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl << endl << endl;
        mem_segment_ = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only,SHARED_MEMORY_NAME);
//  std::cerr << "BAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl << endl << endl;
        blocks_ = mem_segment_->find<SharedMemMap>(MEM_MAP_NAME).first;
//  std::cerr << "BAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl << endl << endl;
    }
}

SharedMemory::~SharedMemory() {
    if (server_) {
        mem_segment_->destroy<SharedMemMap>(MEM_MAP_NAME);
        boost::interprocess::shared_memory_object::remove(SHARED_MEMORY_NAME);
    }
    delete mem_segment_;
}

MemoryBlock* SharedMemory::getBlockPtr(const std::string &name) {
    SharedString temp(name.c_str(),CharAllocator(mem_segment_->get_segment_manager()));
    SharedMemMap::const_iterator it = blocks_->find(temp);
    if (it == blocks_->end()) // the block isn't in our map
        return NULL;
    else
        return (*it).second.get();
}

const MemoryBlock* SharedMemory::getBlockPtr(const std::string &name) const {
    SharedString temp(name.c_str(),CharAllocator(mem_segment_->get_segment_manager()));
    SharedMemMap::const_iterator it = blocks_->find(temp);
    if (it == blocks_->end()) // the block isn't in our map
        return NULL;
    else
        return (*it).second.get();
}

void SharedMemory::getBlockNames(std::vector<std::string> &module_names, bool only_log) const {
    for (SharedMemMap::const_iterator it = blocks_->begin(); it != blocks_->end(); it++) {
        if ((!only_log) || (*it).second->log_block)
            module_names.push_back(std::string((*it).first.c_str()));
    }
}


//void SharedMemory::lock() {
//  mem_mutex_->lock();
//}

//void SharedMemory::unlock() {
//  mem_mutex_->unlock();
//}
