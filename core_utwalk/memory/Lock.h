#ifndef LOCK_CH9LJB3R
#define LOCK_CH9LJB3R

#include <string>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread/thread_time.hpp>

void cleanLock(const std::string &name);

#define SUFFIX_MUTEX "mtx"
#define SUFFIX_CND "cnd"

class Lock {
public:
    Lock(const std::string &name);

    void lock();
    void notify_one();
    void wait();
    bool timed_wait(const boost::system_time &timeout);
    void unlock();

private:
    std::string name_;
    boost::interprocess::named_mutex mutex_;
    boost::interprocess::named_condition cond_;
    boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock_;
};

#endif /* end of include guard: LOCK_CH9LJB3R */
