#include "SensorModule.h"

#include <common/RobotInfo.h>

#include <memory/SensorBlock.h>
#include <memory/FrameInfoBlock.h>

SensorModule::~SensorModule() {};

void SensorModule::specifyMemoryBlocks() {
    requiresMemoryBlock("raw_sensors");
    requiresMemoryBlock("processed_sensors");
    requiresMemoryBlock("frame_info");
    getMemoryBlock(raw_sensors_,"raw_sensors");
    getMemoryBlock(sensors_,"processed_sensors");
    getMemoryBlock(frame_info_,"frame_info");

    inertial_filter_.init(frame_info_->source == MEMORY_SIM);
}

void SensorModule::processSensors() {
    for (int i=0; i<NUM_SENSORS; i++) {
        sensors_->values_[i] = raw_sensors_->values_[i];
    }

    inertial_filter_.setInertialData(raw_sensors_->values_[accelX],raw_sensors_->values_[accelY],raw_sensors_->values_[accelZ],raw_sensors_->values_[gyroX],raw_sensors_->values_[gyroY]);
    inertial_filter_.processFrame();
    //std::cout << "Raw  " << sensors_->values_[angleX] << " " << sensors_->values_[angleY] << std::endl;
    sensors_->values_[angleX] = inertial_filter_.getRoll();
    sensors_->values_[angleY] = inertial_filter_.getTilt();
    //sensors_->values_[angleX] += inertial_filter_.getRoll();
    //sensors_->values_[angleY] += inertial_filter_.getTilt();
    //sensors_->values_[angleX] /= 2.0;
    //sensors_->values_[angleY] /= 2.0;
    //std::cout << "Filt " << sensors_->values_[angleX] << " " << sensors_->values_[angleY] << std::endl;
}
