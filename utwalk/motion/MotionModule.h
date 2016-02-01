#ifndef MOTION_MODULE_H
#define MOTION_MODULE_H

#include <Module.h>

#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <motion/UTWalkEngine.h>


class MotionModule: public Module {
public:
    virtual ~MotionModule();

    void specifyMemoryBlocks();

    void processFrame();

    inline double getMaxXSpeed()   {
        return ut_walk_engine_.getMaxXSpeed();
    }
    inline double getMaxYSpeed()   {
        return ut_walk_engine_.getMaxYSpeed();
    }
    inline double getMaxRotSpeed() {
        return ut_walk_engine_.getMaxRotSpeed();
    }

    inline double getStepSizeTranslationX() {
        return ut_walk_engine_.getStepSizeTranslationX();
    }
    inline double getStepSizeTranslationY() {
        return ut_walk_engine_.getStepSizeTranslationY();
    }
    inline double getStepSizeRotation() {
        return ut_walk_engine_.getStepSizeRotation();
    }
    inline double getPhaseLength() {
        return ut_walk_engine_.getPhaseLength();
    }

private:
    BodyModelBlock* body_model_;
    JointCommandBlock *commands_;
    JointBlock *joint_angles_;
    SensorBlock *sensors_;
    WalkRequestBlock *walk_request_;

    UTWalkEngine ut_walk_engine_;;
};

#endif /* end of include guard: SENSOR_MODULE */
