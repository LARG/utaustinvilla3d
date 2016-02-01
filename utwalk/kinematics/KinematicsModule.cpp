#include "KinematicsModule.h"

#include <kinematics/ForwardKinematics.h>
#include <common/RobotInfo.h>

#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/SensorBlock.h>

KinematicsModule::~KinematicsModule() {};

void KinematicsModule::specifyMemoryBlocks() {
    requiresMemoryBlock("processed_sensors");
    requiresMemoryBlock("processed_joint_angles");
    requiresMemoryBlock("body_model");
    //frame_info_ = (FrameInfo*)getMemoryBlock("frame_info");

    getMemoryBlock(sensors_,"processed_sensors");
    getMemoryBlock(joints_,"processed_joint_angles");

    getMemoryBlock(body_model_,"body_model");
}

void KinematicsModule::calculatePose() {
    ForwardKinematics::calculateRelativePose(joints_->values_, sensors_->values_[angleX], sensors_->values_[angleY], body_model_->rel_parts_, dimensions_);
    ForwardKinematics::calculateAbsolutePose(body_model_->rel_parts_, body_model_->abs_parts_);
    ForwardKinematics::calculateCoM(body_model_->abs_parts_,body_model_->center_of_mass_,mass_calibration_);

    /*Pose3D unrotated_parts[BodyPart::NUM_PARTS];
    ForwardKinematics::calculateRelativePose(joints_->values_, 0, 0, unrotated_parts, dimensions_);
    std::cout << sensors_->values_[angleX] << " " << unrotated_parts[BodyPart::left_bottom_foot].rotation.getYAngle() << std::endl;*/
}
