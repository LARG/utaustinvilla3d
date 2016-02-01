#ifndef KINEMATICS_MODULE_H
#define KINEMATICS_MODULE_H

#include <Module.h>

#include <common/MassCalibration.h>
#include <common/RobotDimensions.h>

struct SensorBlock;
struct JointBlock;
struct BodyModelBlock;

class KinematicsModule: public Module {
public:
    virtual ~KinematicsModule();

    void specifyMemoryBlocks();

    void calculatePose();

private:
    // memory blocks
    SensorBlock* sensors_;
    JointBlock* joints_;
    BodyModelBlock* body_model_;

    // dimensions and mass info
    MassCalibration mass_calibration_;
    RobotDimensions dimensions_;
};

#endif /* end of include guard: SENSOR_MODULE */
