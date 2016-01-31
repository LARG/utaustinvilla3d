#ifndef UTWALKENGINE_IX44V51P
#define UTWALKENGINE_IX44V51P

#include <kinematics/InverseKinematics.h>
#include <common/RobotDimensions.h>
#include <common/PIDController.h>
#include <common/RingBufferWithSum.h>
#include <motion/WalkEngineParameters.h>

#include <memory/Memory.h>
#include <memory/BodyModelBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GraphableBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkEngineBlock.h>
#include <memory/WalkRequestBlock.h>

#include <map>

using namespace std;


class UTWalkEngine {
public:
    UTWalkEngine ();
    virtual ~UTWalkEngine ();

    void init(Memory *mem);
    void processFrame();

    inline double getMaxXSpeed()   {
        return params_.max_step_size_.translation.x;
    }
    inline double getMaxYSpeed()   {
        return params_.max_step_size_.translation.y;
    }
    inline double getMaxRotSpeed() {
        return params_.max_step_size_.rotation;
    }

    inline double getStepSizeTranslationX() {
        return walk_engine_->step_size_.translation.x;
    }
    inline double getStepSizeTranslationY() {
        return walk_engine_->step_size_.translation.y;
    }
    inline double getStepSizeRotation() {
        return walk_engine_->step_size_.rotation;
    }
    inline double getPhaseLength() {
        return walk_engine_->phase_length_;
    }

private:
    BodyModelBlock *body_model_;
    FrameInfoBlock *frame_info_;
    GraphableBlock *graph_;
    JointCommandBlock *commands_;
    JointBlock *joint_angles_;
    SensorBlock *sensors_;
    WalkEngineBlock *walk_engine_;
    WalkRequestBlock *walk_request_;

    InverseKinematics inverse_kinematics_;
    RobotDimensions robot_dimensions_;

    WalkEngineParameters params_;
    map<WalkRequestBlock::ParamSet, WalkEngineParameters> paramSets;

    WalkRequestBlock::Motion last_frame_motion_;
    float last_time_;

    void setWalkParameters(WalkRequestBlock::ParamSet paramSet);
    void setWalkParameters(const WalkEngineParameters &params);

    void processWalkRequest();

    float calcShiftFrac(float t);
    float calcHeightFrac(float t);
    float calcSwingFwd(float t);
    void calcFwdFracs(float t, float &stance_fwd, float &swing_fwd);

    float calcLeftSide(float t, bool left_swing, float step_side_size);
    float calcRightSide(float t, bool left_swing, float step_side_size);
    float calcOffsetFracTowards(float t, bool towards_swing);
    float calcOffsetFracAway(float t, bool away_swing);

    float calcGroinAngle(float t, bool left_swing, float step_turn_size);
    float calcSwingAnkleFrac(float t);

    void commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float groin, float theta_tilt, float theta_roll, const Vector3<float> &com_offset, bool graph_data);

    void setArms(float *command_angles, const Vector2<float> &arm_offset);
    void setHead(float *command_angles);

    float phase_start_;
    float phase_end_;

    void initStiffness();
    void initWalk();
    void stand();
    void walk();
    void switchPhase();
    void resetTime();

    void calcFeetTargets(Pose3D &left_target, Pose3D &right_target, float &groin, float t, bool left_swing, const Pose2D &step_size);

    void calcDesiredCOM();

    PIDController pid_tilt_;
    PIDController pid_roll_;
    float tilt_offset_;
    float roll_offset_;

    PIDController pid_com_x_;
    PIDController pid_com_y_;
    PIDController pid_com_z_;
    Vector3<float> desired_com_;
    Vector3<float> com_offset_;

    PIDController pid_arm_x_;
    PIDController pid_arm_y_;
    Vector2<float> arm_offset_;

    PIDController pid_step_size_x_;
    PIDController pid_step_size_y_;
    PIDController pid_step_size_rot_;

    RingBufferWithSum<Vector3<float>,20> com_errors_;

    void applyHTWKClosedLoop(float angles[], float tilt, float roll);
};

#endif /* end of include guard: UTWALKENGINE_IX44V51P */
