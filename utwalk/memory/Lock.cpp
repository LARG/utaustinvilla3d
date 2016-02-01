#include "Lock.h"
#include <iostream>

void cleanLock(const std::string &name) {
    boost::interprocess::named_mutex::remove((name + SUFFIX_MUTEX).c_str());
    boost::interprocess::named_condition::remove((name + SUFFIX_CND).c_str());
}

Lock::Lock(const std::string &name):
    name_(name),
    mutex_(boost::interprocess::open_or_create,(name_ + SUFFIX_MUTEX).c_str()),
    cond_(boost::interprocess::open_or_create,(name_ + SUFFIX_CND).c_str()),
    lock_(mutex_,boost::interprocess::defer_lock)
{
}

void Lock::lock() {
    //std::cout << "LOCKING " << name_ << std::endl;
    lock_.lock();
    //std::cout << "DONE LOCKING " << name_ << std::endl;
}

void Lock::notify_one() {
    //std::cout << "NOTIFYING " << name_ << std::endl;
    cond_.notify_all();
    //std::cout << "DONE NOTIFYING " << name_ << std::endl;
}

void Lock::wait() {
    //std::cout << "WAITING " << name_ << std::endl;
    cond_.wait(lock_);
    //std::cout << "DONE WAITING " << name_ << std::endl;
}

bool Lock::timed_wait(const boost::system_time &timeout) {
    bool res;
    res = cond_.timed_wait(lock_,timeout);
    return res;
}

void Lock::unlock() {
    //std::cout << "UNLOCKING " << name_ << std::endl;
    lock_.unlock();
    //std::cout << "DONE UNLOCKING " << name_ << std::endl;
}
