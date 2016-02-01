#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

#include <Module.h>
#include "InertialFilter.h"

class SensorBlock;
class FrameInfoBlock;

class SensorModule: public Module {
public:
    virtual ~SensorModule();

    void specifyMemoryBlocks();

    void processSensors();

private:
    SensorBlock *raw_sensors_;
    SensorBlock *sensors_;
    FrameInfoBlock *frame_info_;

    InertialFilter inertial_filter_;
};

#endif /* end of include guard: SENSOR_MODULE */
