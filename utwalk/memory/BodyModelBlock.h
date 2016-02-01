#ifndef BODY_MODEL_BLOCK_
#define BODY_MODEL_BLOCK_

#include <common/RobotInfo.h>
#include <math/Pose3D.h>
#include <math/Vector3.h>

#include "MemoryBlock.h"

struct BodyModelBlock : public MemoryBlock {
public:
    BodyModelBlock()  {
        header.version = 3;
        header.size = sizeof(BodyModelBlock);
    }

    // Body relative to origin
    Pose3D rel_parts_[BodyPart::NUM_PARTS];

    // Body translated and relative to ground
    Pose3D abs_parts_[BodyPart::NUM_PARTS];

    Vector3<float> center_of_mass_;
};

#endif
