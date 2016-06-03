#ifndef WALKREQUESTBLOCK_RE8SDRLN
#define WALKREQUESTBLOCK_RE8SDRLN

#include <math/Pose2D.h>
#include "MemoryBlock.h"

struct WalkRequestBlock : public MemoryBlock {
public:
    enum Motion {
        WALK,
        STAND,
        NONE,
        NUM_OF_MOTIONS
    };

    enum ParamSet {
        PARAMS_NONE,
        PARAMS_DEFAULT,
        PARAMS_POSITIONING,
        PARAMS_APPROACH_BALL
    };

    WalkRequestBlock():
        motion_(STAND),
        paramSet_(PARAMS_DEFAULT),
        speed_(),
        percentage_speed_(true),
        pedantic_walk_(false)
    {
        header.version = 0;
        header.size = sizeof(WalkRequestBlock);
    }

    Motion motion_; // what type of motion, just walk or stand for now, maybe add kicks from walking start
    ParamSet paramSet_; // What parameter set to use.
    Pose2D speed_; // the speed of the walk
    bool percentage_speed_; // true if speed is percentage rather than absolute vel
    bool pedantic_walk_; // true disables the step size stabilization.  "Set it when precision is indispensable"
};

#endif /* end of include guard: WALKREQUESTBLOCK_RE8SDRLN */
