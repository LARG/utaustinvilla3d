#ifndef FRAMEINFO_KJS7E8UX
#define FRAMEINFO_KJS7E8UX

#include "MemoryBlock.h"
#include <iostream>

enum MemorySource {
    MEMORY_ROBOT,
    MEMORY_SIM
};

struct FrameInfoBlock : public MemoryBlock {
public:
    FrameInfoBlock(unsigned int frame_id, double seconds_since_start, MemorySource source):
        frame_id(frame_id),
        start_time(-1),
        seconds_since_start(seconds_since_start),
        source(source)
    {
        header.version = 0;
        header.size = sizeof(FrameInfoBlock);
        log_block = true;
    }
    unsigned int frame_id;
    double start_time;
    double seconds_since_start;
    MemorySource source;
};

#endif /* end of include guard: FRAMEINFO_KJS7E8UX */
