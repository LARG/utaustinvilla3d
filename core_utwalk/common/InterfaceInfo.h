#ifndef INTERFACEINFO_R3BSIHAC
#define INTERFACEINFO_R3BSIHAC

const int robot_joint_signs[NUM_JOINTS] = {
    1, // HeadYaw
    -1, // HeadPitch

    1, // LHipYawPitch
    -1, // LHipRoll
    1, // LHipPitch
    1, // LKneePitch
    1, // LAnklePitch
    -1, // LAnkleRoll

    1, // RHipYawPitch
    1, // RHipRoll
    1, // RHipPitch
    1, // RKneePitch
    1, // RAnklePitch
    1, // RAnkleRoll

    -1, // LShoulderPitch
    1, // LShoulderRoll
    1, // LElbowYaw
    1, // LElbowRoll

    -1, // RShoulderPitch
    -1, // RShoulderRoll
    -1, // RElbowYaw
    -1 // RElbowRoll
};

const int spark_joint_signs[NUM_JOINTS] = {
    1, // HeadYaw
    1, // HeadPitch

    1, // LHipYawPitch
    -1, // LHipRoll
    -1, // LHipPitch
    -1, // LKneePitch
    -1, // LAnklePitch
    -1, // LAnkleRoll

    1, // RHipYawPitch
    1, // RHipRoll
    -1, // RHipPitch
    -1, // RKneePitch
    -1, // RAnklePitch
    1, // RAnkleRoll // CHANGED BY SAM to match what Bhuman expects

    1, // LShoulderPitch
    1, // LShoulderRoll
    1, // LElbowYaw
    1, // LElbowRoll

    1, // RShoulderPitch
    -1, // RShoulderRoll
    -1, // RElbowYaw
    -1 // RElbowRoll
};

#endif
