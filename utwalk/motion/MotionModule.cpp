#include "MotionModule.h"

#include <common/RobotInfo.h>

MotionModule::~MotionModule() {};

void MotionModule::specifyMemoryBlocks() {
    requiresMemoryBlock("body_model");
    requiresMemoryBlock("processed_joint_angles");
    requiresMemoryBlock("processed_joint_commands");
    requiresMemoryBlock("processed_sensors");
    requiresMemoryBlock("walk_request");


    getMemoryBlock(body_model_,"body_model");
    getMemoryBlock(joint_angles_,"processed_joint_angles");
    getMemoryBlock(commands_,"processed_joint_commands");
    getMemoryBlock(sensors_,"processed_sensors");
    getMemoryBlock(walk_request_,"walk_request");

    ut_walk_engine_.init(memory_);
}

void MotionModule::processFrame() {
    ut_walk_engine_.processFrame();
}

