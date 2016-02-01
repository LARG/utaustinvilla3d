#ifndef ODOMETRY_
#define ODOMETRY_

#include "MemoryBlock.h"

// DANIEL ADDED
#include <math/Geometry.h>

// all info needed by localization about robot motions
// includes kicking, walking, falling

struct OdometryBlock : public MemoryBlock {
public:
    OdometryBlock()
    {
        header.version = 0;
        header.size = sizeof(OdometryBlock);

        displacement = Point2D(0,0);
        angDisplacement = 0;

        didKick = false;

        fallen = false;
        getUp = 0;
    }

    // walking
    Point2D displacement;
    AngRad angDisplacement;

    // kicking
    bool didKick;
    float kickVelocity;
    float kickHeading;

    // falling
    bool fallen;
    int getUp;
};

#endif
