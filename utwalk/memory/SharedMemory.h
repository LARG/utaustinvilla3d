#ifndef SHAREDMEMORY_85G7089E
#define SHAREDMEMORY_85G7089E

#include <iostream>
#include <cstdarg>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/containers/string.hpp>
#include "AbstractMemory.h"

typedef boost::interprocess::allocator<void,boost::interprocess::managed_shared_memory::segment_manager> VoidAllocator;
typedef boost::interprocess::allocator<char,boost::interprocess::managed_shared_memory::segment_manager> CharAllocator;
typedef boost::interprocess::basic_string<char,std::char_traits<char>,CharAllocator> SharedString;
typedef boost::interprocess::offset_ptr<MemoryBlock> MemoryBlockPtr;
typedef boost::interprocess::map<SharedString,MemoryBlockPtr,std::less<SharedString>,boost::interprocess::allocator<std::pair<const SharedString, MemoryBlockPtr>,boost::interprocess::managed_shared_memory::segment_manager> > SharedMemMap;

class SharedMemory : public AbstractMemory {
public:
    SharedMemory(bool server = false);
    virtual ~SharedMemory();
    MemoryBlock* getBlockPtr(const std::string &name);
    const MemoryBlock* getBlockPtr(const std::string &name) const;
    void getBlockNames(std::vector<std::string> &module_names, bool only_log) const;

protected:
    SharedMemMap *blocks_;
    bool server_;
    boost::interprocess::managed_shared_memory *mem_segment_;

    const static int MEMORY_SIZE;
    const static char MEM_MAP_NAME[];
    const static char SHARED_MEMORY_NAME[];

public:
    // addBlock is in the header so that the templating is handled correctly
    template <class T>
    bool addBlock(const std::string &name, T *block) {
        if (dynamic_cast<MemoryBlock*>(block) == NULL) {
            std::cerr << "SharedMemory::addBlock - Block " << name << " is not of type MemoryBlock" << std::endl;
        }
        // check if the block already exists
        MemoryBlock *found_block = getBlockPtr(name);
        if (found_block != NULL) {
            std::cerr << "SharedMemory::addBlock - Block " << name << " already exists, trying to set it equal though" << std::endl;
            *found_block = *block;
            delete block;
            return false;
        }
        // create a copy of the block in shared memory
        MemoryBlockPtr shared_block = mem_segment_->construct<T>(name.c_str())(*block);
        // remove the original block
        delete block;

        // try to add the block
        SharedString key(name.c_str(),CharAllocator(mem_segment_->get_segment_manager()));
        std::pair<SharedMemMap::iterator,bool> res = blocks_->insert(std::pair<const SharedString,MemoryBlockPtr>(key,shared_block));

        // check if the block was inserted
        if (!res.second) {
            std::cerr << "SharedMemory::addBlock - ERROR ADDING BLOCK " << name << " already exists, Deleting given block" << std::endl;
            mem_segment_->destroy<T>(name.c_str());
        }
        return res.second;
    };
};

#endif /* end of include guard: SHAREDMEMORY_85G7089E */
