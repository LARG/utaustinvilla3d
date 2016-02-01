#include "Memory.h"

#include "BodyModelBlock.h"
#include "FrameInfoBlock.h"
#include "GraphableBlock.h"
#include "JointBlock.h"
#include "JointCommandBlock.h"
#include "SensorBlock.h"
#include "SimEffectorBlock.h"
#include "WalkRequestBlock.h"
#include "WalkEngineBlock.h"
#include "OdometryBlock.h"


#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

//void cleanLocks(){
//cleanLock(LOCK_MOTION_NAME);
//cleanLock(LOCK_VISION_NAME);
//cleanLock(LOCK_VISION_MOTION_NAME);
//}

Memory::Memory(bool use_shared_memory, bool server):
    use_shared_memory_(use_shared_memory),
    motion_lock_(NULL),
    vision_lock_(NULL),
    vision_motion_lock_(NULL)
{
    if (use_shared_memory_) {
        shared_memory_ = new SharedMemory(server);
        private_memory_ = NULL;
    } else {
        shared_memory_ = NULL;
        private_memory_ = new PrivateMemory();
    }
}

Memory::Memory(const Memory &old):
    motion_lock_(NULL),
    vision_lock_(NULL),
    vision_motion_lock_(NULL)
{
    use_shared_memory_ = old.use_shared_memory_;
    if (use_shared_memory_) {
        std::cout << "CAN'T HANDLE COPYING SHARED MEMORY AT THIS TIME" << std::endl;
        exit(1);
    }
    private_memory_ = new PrivateMemory(*old.private_memory_);
    shared_memory_ = NULL;
}

Memory::~Memory() {
    if (shared_memory_ != NULL) {
        delete shared_memory_;
        shared_memory_ = NULL;
    }
    if (private_memory_ != NULL) {
        delete private_memory_;
        private_memory_ = NULL;
    }
    // delete the locks
    if (motion_lock_ != NULL) {
        delete motion_lock_;
        motion_lock_ = NULL;
    }
    if (vision_lock_ != NULL) {
        delete vision_lock_;
        vision_lock_ = NULL;
    }
    if (vision_motion_lock_ != NULL) {
        delete vision_motion_lock_;
        vision_motion_lock_ = NULL;
    }
}


MemoryBlock* Memory::getBlockPtr(const std::string &name) {
    if (use_shared_memory_)
        return shared_memory_->getBlockPtr(name);
    else
        return private_memory_->getBlockPtr(name);
}

const MemoryBlock* Memory::getBlockPtr(const std::string &name) const {
    if (use_shared_memory_)
        return shared_memory_->getBlockPtr(name);
    else
        return private_memory_->getBlockPtr(name);
}

void Memory::getBlockNames(std::vector<std::string> &module_names, bool only_log) const {
    if (use_shared_memory_)
        shared_memory_->getBlockNames(module_names,only_log);
    else
        private_memory_->getBlockNames(module_names,only_log);
}

void Memory::createLocks(bool clean_locks) {
    if (clean_locks) {
        cleanLock(LOCK_MOTION_NAME);
        cleanLock(LOCK_VISION_NAME);
        cleanLock(LOCK_VISION_MOTION_NAME);
    }
    assert(motion_lock_ == NULL);
    motion_lock_ = new Lock(LOCK_MOTION_NAME);
    assert(vision_lock_ == NULL);
    vision_lock_ = new Lock(LOCK_VISION_NAME);
    assert(vision_motion_lock_ == NULL);
    vision_motion_lock_ = new Lock(LOCK_VISION_MOTION_NAME);
}

void Memory::lock(MemoryLockType lock_type) {
    Lock &lock = getLock(lock_type);
    lock.lock();
}

void Memory::notify_lock(MemoryLockType lock_type) {
    Lock &lock = getLock(lock_type);
    lock.notify_one();
}

void Memory::wait_lock(MemoryLockType lock_type) {
    Lock &lock = getLock(lock_type);
    lock.wait();
}

bool Memory::timed_wait_lock(MemoryLockType lock_type, long wait_time_in_ms) {
    boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(wait_time_in_ms);
    Lock &lock = getLock(lock_type);
    return lock.timed_wait(timeout);
}

void Memory::unlock(MemoryLockType lock_type) {
    Lock &lock = getLock(lock_type);
    lock.unlock();
}

void Memory::setBlockLogging(const std::string &name, bool log_block) {
    getBlockPtr(name)->log_block = log_block;
}

MemoryBlock* Memory::getBlockPtrByName(const std::string &name) {
    return getBlockPtr(name);
}

const MemoryBlock* Memory::getBlockPtrByName(const std::string &name) const {
    return getBlockPtr(name);
}


Lock& Memory::getLock(MemoryLockType lock_type) {
    Lock *lock = NULL;
    switch (lock_type) {
    case LOCK_MOTION:
        lock = motion_lock_;
        break;
    case LOCK_VISION:
        lock = vision_lock_;
        break;
    case LOCK_VISION_MOTION:
        lock = vision_motion_lock_;
        break;
    }
    assert(lock != NULL);
    return *lock;
}

bool Memory::addBlockByName(const std::string &name) {
    if (name == "frame_info")
        return addBlock(name,new FrameInfoBlock(0,0,MEMORY_ROBOT));
    if (name == "vision_frame_info")
        return addBlock(name,new FrameInfoBlock(0,0,MEMORY_ROBOT));
    else if (name == "body_model")
        return addBlock(name,new BodyModelBlock());
    else if (name == "graphable")
        return addBlock(name,new GraphableBlock());
    // joints
    else if (name == "raw_joint_angles")
        return addBlock(name,new JointBlock());
    else if (name == "processed_joint_angles")
        return addBlock(name,new JointBlock());
    // commands
    else if (name == "raw_joint_commands")
        return addBlock(name,new JointCommandBlock());
    else if (name == "processed_joint_commands")
        return addBlock(name,new JointCommandBlock());
    // sensors
    else if (name == "raw_sensors")
        return addBlock(name,new SensorBlock());
    else if (name == "processed_sensors")
        return addBlock(name,new SensorBlock());
    // Sim
    else if (name == "sim_effectors")
        return addBlock(name, new SimEffectorBlock());
    // motion
    else if (name == "walk_engine")
        return addBlock(name,new WalkEngineBlock());
    else if (name == "walk_request")
        return addBlock(name,new WalkRequestBlock());
    // odometry for localization (includes kick, walk, fall info)
    else if (name == "odometry")
        return addBlock(name,new OdometryBlock());
    else {
        std::cerr << "Memory::addBlockByName: Error: Unknown memory block for name: " << name << std::endl << std::flush;
        return false;
    }
}
