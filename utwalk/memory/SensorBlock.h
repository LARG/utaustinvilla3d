#ifndef SENSORBLOCK_
#define SENSORBLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct SensorBlock : public MemoryBlock {
public:
    SensorBlock()  {
        header.version = 0;
        header.size = sizeof(SensorBlock);
    }

    float values_[NUM_SENSORS];
};

#endif
