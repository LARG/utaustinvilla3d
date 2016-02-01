#ifndef WALKENGINEBLOCK_CTX0HMI2
#define WALKENGINEBLOCK_CTX0HMI2

#include "MemoryBlock.h"
#include <math/Pose2D.h>
#include <math/Pose3D.h>

struct WalkEngineBlock : public MemoryBlock {
    WalkEngineBlock() {
        header.version = 1;
        header.size = sizeof(WalkEngineBlock);
    }

    // where are we in the phase
    float phase_frac_;
    bool left_swing_;

    // what walk are we trying to execute
    float phase_length_;
    Pose2D step_size_;
    Pose2D last_step_size_;

    // current foot destinations
    float groin_;
    Pose3D left_foot_;
    Pose3D right_foot_;
};

#endif /* end of include guard: WALKENGINEBLOCK_CTX0HMI2 */
