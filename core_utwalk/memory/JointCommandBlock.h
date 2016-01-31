#ifndef JOINT_COMMAND_BLOCK_
#define JOINT_COMMAND_BLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct JointCommandBlock : public MemoryBlock {
public:
    JointCommandBlock()  {
        header.version = 1;
        header.size = sizeof(JointCommandBlock);
        angle_time_ = 1000;
        stiffness_time_ = 1000;
        for (int i=0; i<NUM_JOINTS; i++) {
            angles_[i] = 0;
            stiffness_[i] = 0;
        }
    }

    void setPoseRad(float src[NUM_JOINTS]) {
        for (int i=0; i<NUM_JOINTS; i++) {
            angles_[i]=src[i];
        }
    }

    void setPoseDeg(float src[NUM_JOINTS]) {
        for (int i=0; i<NUM_JOINTS; i++) angles_[i]=DEG_T_RAD*src[i];
    }

    bool send_stiffness_;
    float angles_[NUM_JOINTS];
    float stiffness_[NUM_JOINTS];
    float angle_time_;
    float stiffness_time_;
};

#endif
