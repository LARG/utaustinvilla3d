#ifndef ROBOTINFO_R3BSIHAC
#define ROBOTINFO_R3BSIHAC

#include <string>
#include <cmath>

#ifndef RAD_T_DEG
#define RAD_T_DEG  180.0/ M_PI
#endif
#ifndef DEG_T_RAD
#define DEG_T_RAD  M_PI/180.0
#endif

#define FOVx DEG_T_RAD * 46.4 //0.8   // From Todd's calculations
//DEG_T_RAD*60.0 // From Webots (i think)

#define FOVy DEG_T_RAD * 34.8 //45.0 // From demo_camera.cpp

#define SIGMA 1.0

#define IMAGE_X 640 //160 //640
#define IMAGE_Y 480 //120 //480

// to modify our size checks automatically based on image resolution
const int IMAGE_FACTOR = (float)IMAGE_X/160.0;
const int IMAGE_FACTOR2 = IMAGE_FACTOR*IMAGE_FACTOR;

#define IMAGE_SIZE IMAGE_X*IMAGE_Y
#define RAW_IMAGE_SIZE IMAGE_SIZE*2

#define SIM_IMAGE_X 640
#define SIM_IMAGE_Y 480

enum Sensor {
    gyroX,
    gyroY,
    gyroZ,
    accelX,
    accelY,
    accelZ,
    battery,
    fsrLFL,
    fsrLFR,
    fsrLRL,
    fsrLRR,
    fsrRFL,
    fsrRFR,
    fsrRRL,
    fsrRRR,
    ultrasoundL,
    ultrasoundR,
    angleX,
    angleY,
    NUM_SENSORS
};

enum Joint {
    HeadYaw = 0,
    HeadPitch = 1,

    LHipYawPitch = 2,
    LHipRoll = 3,
    LHipPitch = 4,
    LKneePitch = 5,
    LAnklePitch = 6,
    LAnkleRoll = 7,

    RHipYawPitch = 8,
    RHipRoll = 9,
    RHipPitch = 10,
    RKneePitch = 11,
    RAnklePitch = 12,
    RAnkleRoll = 13,

    LShoulderPitch = 14,
    LShoulderRoll = 15,
    LElbowYaw = 16,
    LElbowRoll = 17,

    RShoulderPitch = 18,
    RShoulderRoll = 19,
    RElbowYaw = 20,
    RElbowRoll = 21,

    LToePitch = 22,
    RToePitch = 23,

    NUM_JOINTS = 24
};

enum jointsAltern {
    HeadPan,
    HeadTilt,

    LShoulderRotator,
    LShoulderAbductor,
    LElbowBend,
    LElbowTwist,

    LHipYawPitchA,
    LHipRollA,
    LHipPitchA,
    LKneePitchA,
    LAnklePitchA,
    LAnkleRollA,

    RHipYawPitchA,
    RHipRollA,
    RHipPitchA,
    RKneePitchA,
    RAnklePitchA,
    RAnkleRollA,

    RShoulderRotator,
    RShoulderAbductor,
    RElbowBend,
    RElbowTwist
};

typedef float Joints[NUM_JOINTS];

struct JointRequest
{
    Joints angles;
    int hardness[NUM_JOINTS];
};

// Returns the joint name
static const char* getJointName(Joint joint)
{
    switch(joint)
    {
    case HeadYaw:
        return "HeadYaw";
    case HeadPitch:
        return "HeadPitch";
    case LShoulderPitch:
        return "LShoulderPitch";
    case LShoulderRoll:
        return "LShoulderRoll";
    case LElbowYaw:
        return "LElbowYaw";
    case LElbowRoll:
        return "LElbowRoll";
    case RShoulderPitch:
        return "RShoulderPitch";
    case RShoulderRoll:
        return "RShoulderRoll";
    case RElbowYaw:
        return "RElbowYaw";
    case RElbowRoll:
        return "RElbowRoll";
    case LHipYawPitch:
        return "LHipYawPitch";
    case LHipRoll:
        return "LHipRoll";
    case LHipPitch:
        return "LHipPitch";
    case LKneePitch:
        return "LKneePitch";
    case LAnklePitch:
        return "LAnklePitch";
    case LAnkleRoll:
        return "LAnkleRoll";
    case RHipYawPitch:
        return "RHipYawPitch";
    case RHipRoll:
        return "RHipRoll";
    case RHipPitch:
        return "RHipPitch";
    case RKneePitch:
        return "RKneePitch";
    case RAnklePitch:
        return "RAnklePitch";
    case RAnkleRoll:
        return "RAnkleRoll";
    case LToePitch:
        return "LToePitch";
    case RToePitch:
        return "RToePitch";
    default:
        return "unknown";
    }
}

// Returns the string location in fast access for the sensor
static const char* getSensorString(Sensor s)
{
    switch(s)
    {
    case gyroX:
        return "InertialSensor/GyrX";
    case gyroY:
        return "InertialSensor/GyrY";
    case gyroZ:
        return "InertialSensor/GyrZ";
    case accelX:
        return "InertialSensor/AccX";
    case accelY:
        return "InertialSensor/AccY";
    case accelZ:
        return "InertialSensor/AccZ";
    case angleX:
        return "InertialSensor/AngleX";
    case angleY:
        return "InertialSensor/AngleY";
    case fsrLFL:
        return "LFoot/FSR/FrontLeft";
    case fsrLFR:
        return "LFoot/FSR/FrontRight";
    case fsrLRL:
        return "LFoot/FSR/RearLeft";
    case fsrLRR:
        return "LFoot/FSR/RearRight";
    case fsrRFL:
        return "RFoot/FSR/FrontLeft";
    case fsrRFR:
        return "RFoot/FSR/FrontRight";
    case fsrRRL:
        return "RFoot/FSR/RearLeft";
    case fsrRRR:
        return "RFoot/FSR/RearRight";
    case ultrasoundL:
        return "US/Left";
    case ultrasoundR:
        return "US/Right";
    case battery:
        return "Battery/Charge";
    default:
        return "unknown";
    }
}

// joint limits
const float minJointLimits[NUM_JOINTS] = {
    DEG_T_RAD * (-110.0 + SIGMA), //(-120.0 + SIGMA), // HeadYaw // Todd:  doc says 120, get an error if we go over 60
    DEG_T_RAD * (-45.0 + SIGMA), // HeadPitch

    DEG_T_RAD * (-90.0 + SIGMA), // LHipYawPitch
    DEG_T_RAD * (-25.0 + SIGMA), // LHipRoll
    DEG_T_RAD * (-100.0 + SIGMA), // LHipPitch
    DEG_T_RAD * (0.0 + SIGMA), // LKneePitch
    DEG_T_RAD * (-75.0 + SIGMA), // LAnklePitch
    DEG_T_RAD * (-45.0 + SIGMA), // LAnkleRoll

    DEG_T_RAD * (-90.0 + SIGMA), // RHipYawPitch
    DEG_T_RAD * (-45.0 + SIGMA), // RHipRoll
    DEG_T_RAD * (-100.0 + SIGMA), // RHipPitch
    DEG_T_RAD * (0.0 + SIGMA), // RKneePitch
    DEG_T_RAD * (-75.0 + SIGMA), // RAnklePitch
    DEG_T_RAD * (-25.0 + SIGMA), // RAnkleRoll

    DEG_T_RAD * (-115.0 + SIGMA), //(-120.0 + SIGMA), // LShoulderPitch
    DEG_T_RAD * (0.0 + SIGMA), // LShoulderRoll
    DEG_T_RAD * (-120.0 + SIGMA), // LElbowYaw
    DEG_T_RAD * (-90.0 + SIGMA), // LElbowRoll


    DEG_T_RAD * (-115.0 + SIGMA), //(-120.0 + SIGMA), // seems to be 115 // RShoulderPitch
    DEG_T_RAD * (-95.0 + SIGMA), // RShoulderRoll
    DEG_T_RAD * (-120.0 + SIGMA), // RElbowYaw
    DEG_T_RAD * (0.0 + SIGMA), // RElbowRoll

    DEG_T_RAD * (-1.0 + SIGMA), // LToePitch
    DEG_T_RAD * (-1.0 + SIGMA) // RToePitch
};


const float maxJointLimits[NUM_JOINTS] = {
    DEG_T_RAD * (110.0 - SIGMA), //(120.0 - SIGMA) // HeadYaw,
    DEG_T_RAD * (45.0 - SIGMA), // HeadPitch

    DEG_T_RAD * (0.0 - SIGMA), // LHipYawPitch
    DEG_T_RAD * (45.0 - SIGMA), // LHipRoll
    DEG_T_RAD * (25.0 - SIGMA), // LHipPitch
    DEG_T_RAD * (130.0 - SIGMA), // LKneePitch
    DEG_T_RAD * (45.0 - SIGMA), // LAnklePitch
    DEG_T_RAD * (25.0 - SIGMA), // LAnkleRoll

    DEG_T_RAD * (0.0 - SIGMA), // RHipYawPitch
    DEG_T_RAD * (25.0 - SIGMA), // RHipRoll
    DEG_T_RAD * (25.0 - SIGMA), // RHipPitch
    DEG_T_RAD * (130.0 - SIGMA), // RKneePitch
    DEG_T_RAD * (45.0 - SIGMA), // RAnklePitch
    DEG_T_RAD * (45.0 - SIGMA), // RAnkleRoll

    DEG_T_RAD * (115.0 - SIGMA), //(120.0 - SIGMA) // LShoulderPitch
    DEG_T_RAD * (95.0 - SIGMA), // LShoulderRoll
    DEG_T_RAD * (120.0 - SIGMA), // LElbowYaw
    DEG_T_RAD * (0.0 - SIGMA), // LElbowRoll

    DEG_T_RAD * (115.0 - SIGMA), //(120.0 - SIGMA), // RShoulderPitch
    DEG_T_RAD * (0.0 - SIGMA), // RShoulderRoll
    DEG_T_RAD * (120.0 - SIGMA), // RElbowYaw
    DEG_T_RAD * (90.0 - SIGMA), // RElbowRoll

    DEG_T_RAD * (70.0 - SIGMA), // LToePitch
    DEG_T_RAD * (70.0 - SIGMA) // RToePitch
};

#define LEFT_LEG_START LHipYawPitch
#define RIGHT_LEG_START RHipYawPitch
#define LEFT_ARM_START LShoulderPitch
#define RIGHT_ARM_START RShoulderPitch
#define HEAD_START HeadYaw

//#define LEFT_LEG_END LAnkleRoll
//#define RIGHT_LEG_END RAnkleRoll
//#define LEFT_ARM_END LElbowYaw
//#define RIGHT_ARM_END RElbowYaw

#define NUM_LEG_JOINTS 6
#define NUM_ARM_JOINTS 4
#define NUM_HEAD_JOINTS 2

// All the body parts that have weight
namespace BodyPart {
enum Part {
    neck,
    head,
    left_shoulder,
    left_bicep,
    left_elbow,
    left_forearm,
    right_shoulder,
    right_bicep,
    right_elbow,
    right_forearm,
    left_pelvis,
    left_hip,
    left_thigh,
    left_tibia,
    left_ankle,
    left_foot, // rotated at ankle (used for weight)
    left_bottom_foot, // translated (used for pose)
    right_pelvis,
    right_hip,
    right_thigh,
    right_tibia,
    right_ankle,
    right_foot,
    right_bottom_foot,
    torso,
    NUM_PARTS
};
}

// Location of key body parts
namespace BodyFrame {
enum Part {
    origin,
    head,
    left_shoulder,
    left_elbow,
    left_wrist,
    right_shoulder,
    right_elbow,
    right_wrist,
    left_hip,
    left_knee,
    left_ankle,
    left_foot,
    right_hip,
    right_knee,
    right_ankle,
    right_foot,
    left_foot_front_left,
    left_foot_front_right,
    left_foot_rear_left,
    left_foot_rear_right,
    right_foot_front_left,
    right_foot_front_right,
    right_foot_rear_left,
    right_foot_rear_right,
    NUM_POINTS,
};
}

#endif /* end of include guard: ROBOTINFO_R3BSIHAC */
