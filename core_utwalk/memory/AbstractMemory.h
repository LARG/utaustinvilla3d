#ifndef ABSTRACTMEMORY_JK0RIQTZ
#define ABSTRACTMEMORY_JK0RIQTZ

#include <vector>
#include <string>
#include "MemoryBlock.h"

class AbstractMemory {
public:
    AbstractMemory() {};
    ~AbstractMemory() {};
    //virtual bool addBlock(const std::string &name,MemoryBlock *block) = 0;
    virtual MemoryBlock* getBlockPtr(const std::string &name) = 0;
    virtual const MemoryBlock* getBlockPtr(const std::string &name) const = 0;
    virtual void getBlockNames(std::vector<std::string> &module_names, bool only_log) const = 0;
};

#endif /* end of include guard: ABSTRACTMEMORY_JK0RIQTZ */
