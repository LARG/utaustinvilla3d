#ifndef MEMORY_3Z94SCHB
#define MEMORY_3Z94SCHB

#include <vector>
#include <string>
#include "MemoryBlock.h"
#include "SharedMemory.h"
#include "PrivateMemory.h"
#include "Lock.h"

#define LOCK_MOTION_NAME "motion"
#define LOCK_VISION_NAME "vision"
#define LOCK_VISION_MOTION_NAME "visionmotion"

struct MemoryHeader {
    std::vector<std::string> block_names;
};

//void cleanLocks();

class Memory {
public:
    enum MemoryLockType {
        LOCK_MOTION,
        LOCK_VISION,
        LOCK_VISION_MOTION
    };

    Memory(bool use_shared_memory, bool server = false);
    Memory(const Memory&);
    ~Memory();

    void setBlockLogging(const std::string &name,bool log_block);
    MemoryBlock* getBlockPtrByName(const std::string &name);
    const MemoryBlock* getBlockPtrByName(const std::string &name) const;
    bool addBlockByName(const std::string &name);

    template <class T>
    void getBlockByName(T *&ptr, const std::string &name) {
        MemoryBlock *temp = getBlockPtr(name);
        if (temp == NULL)
            std::cerr << "Memory::getBlockByName: Error: couldn't get block for name " << name << std::endl;
        //ptr = dynamic_cast<T*>(temp);
        ptr = (T*)(temp);
        if (ptr == NULL)
            std::cerr << "Memory::getBlockByName: Error: got block with unexpected type " << name << std::endl;
    }

    template <class T>
    void getOrAddBlockByName(T *&ptr, const std::string &name) {
        MemoryBlock *temp = getBlockPtr(name);
        if (temp == NULL) {
            bool res = addBlockByName(name);
            if (!res)
                std::cerr << "Memory::getOrAddBlockByName: ERROR: failed to add block for name: " << name << std::endl;
        }
        getBlockByName(ptr,name);
    }

    void getBlockNames(std::vector<std::string> &module_names, bool only_log) const;
private:
    void createLocks(bool clean_locks);

    void lock(MemoryLockType);
    void notify_lock(MemoryLockType);
    void wait_lock(MemoryLockType);
    bool timed_wait_lock(MemoryLockType,long wait_time_in_ms);
    void unlock(MemoryLockType);

private:
    template <class T>
    bool addBlock(const std::string &name,T *block) {
        if (use_shared_memory_)
            return shared_memory_->addBlock(name,block);
        else
            return private_memory_->addBlock(name,block);
    };

    MemoryBlock* getBlockPtr(const std::string &name);
    const MemoryBlock* getBlockPtr(const std::string &name) const;

private:
    bool use_shared_memory_;
    SharedMemory *shared_memory_;
    PrivateMemory *private_memory_;

    Lock *motion_lock_;
    Lock *vision_lock_;
    Lock *vision_motion_lock_;

    Lock &getLock(MemoryLockType);
};

#endif /* end of include guard: MEMORY_3Z94SCHB */
