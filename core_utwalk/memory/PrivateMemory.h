#ifndef PRIVATEMEMORY_DMIY3W1X
#define PRIVATEMEMORY_DMIY3W1X

#include <vector>
#include <map>
#include <string>
#include "MemoryBlock.h"
#include "AbstractMemory.h"

typedef std::map<const std::string,MemoryBlock*> MemMap;

class PrivateMemory : public AbstractMemory {
public:
    PrivateMemory();
    virtual ~PrivateMemory();
    bool addBlock(const std::string &name,MemoryBlock *block);
    MemoryBlock* getBlockPtr(const std::string &name);
    const MemoryBlock* getBlockPtr(const std::string &name) const;
    void getBlockNames(std::vector<std::string> &module_names,bool only_log) const;

protected:
    MemMap blocks_;
};

#endif /* end of include guard: PRIVATEMEMORY_DMIY3W1X */
