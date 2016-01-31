#ifndef SIMEFFECTORBLOCK_H_
#define SIMEFFECTORBLOCK_H_

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct SimEffector {
public:
    SimEffector() {
        scale_ = 1.0;

        current_error_= 0;
        previous_error_= 0;
        cumulative_error_=0;

        current_angle_ = 0;
        target_angle_ = 0;

        k1_ = 0.15;
        k2_ = 0.0;
        k3_ = 0.01;

    }

    void updateErrors() {
        previous_error_ = current_error_;
        current_error_ = target_angle_ - current_angle_;
        cumulative_error_ += current_error_;
    }

    void updateAngles(const double &cangle, const double &tangle) {
        current_angle_ = cangle;
        target_angle_ = tangle;
    }



    // Constants for PID control
    double k1_, k2_, k3_;

    // Error terms
    double current_error_;// corr. k1
    double cumulative_error_;// corr. k2
    double previous_error_;// corr. k3

    double current_angle_;
    double target_angle_;

    float scale_;
};

struct SimEffectorBlock : public MemoryBlock {
public:
    SimEffectorBlock()  {
        header.version = 0;
        header.size = sizeof(SimEffectorBlock);
    }

    SimEffector effector_[NUM_JOINTS];
};

#endif
