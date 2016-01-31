#ifndef MEMORYBLOCK_4MKDDAED
#define MEMORYBLOCK_4MKDDAED

#include <cstring>

struct MemoryBlockHeader {
    unsigned int version;
    unsigned int size;
};

class MemoryBlock {
public:
    MemoryBlock();
    virtual ~MemoryBlock() {}

    MemoryBlockHeader header;
    bool log_block;

};

#endif /* end of include guard: MEMORYBLOCK_4MKDDAED */
