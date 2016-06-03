#include "UTWalkEngine.h"

#include <kinematics/ForwardKinematics.h>
#include <math/Geometry.h>

extern int agentBodyType;

UTWalkEngine::UTWalkEngine()
{
    paramSets[WalkRequestBlock::PARAMS_DEFAULT] = WalkEngineParameters();
    paramSets[WalkRequestBlock::PARAMS_POSITIONING] = WalkEngineParameters("pos_");
    paramSets[WalkRequestBlock::PARAMS_APPROACH_BALL] = WalkEngineParameters("app_");
}

UTWalkEngine::~UTWalkEngine() {
}

void UTWalkEngine::init(Memory *mem) {
    mem->getBlockByName(body_model_,"body_model");
    mem->getBlockByName(frame_info_,"frame_info");
    mem->getBlockByName(graph_,"graphable");
    mem->getBlockByName(commands_,"processed_joint_commands");
    mem->getBlockByName(joint_angles_,"processed_joint_angles");
    mem->getBlockByName(sensors_,"processed_sensors");
    mem->getBlockByName(walk_engine_,"walk_engine");
    mem->getBlockByName(walk_request_,"walk_request");

    last_time_ = frame_info_->seconds_since_start;

    setWalkParameters(WalkRequestBlock::PARAMS_DEFAULT);

    initWalk();
}

void UTWalkEngine::setWalkParameters(WalkRequestBlock::ParamSet paramSet)
{
    /*
     * [victor] Attempt to transition between two parameter sets.
     * If transitions are not smooth (i.e. robot falls a lot), some code
     * should be added here to try to make transitions not as abrupt.
     * Ideas include:
     *    - Keep some sort of history and refuse to transition if there's
     *      a lot of thrashing between parameter sets.
     *    - Only transition when agent speed is within acceptable ranges?
     *    - Don't transition in the middle of a phase?
     *    - Don't transition with lots of error?
     */

    /*
     * [patmac] The above problem mentioned by victor is generally taken care of
     * by using overlapping layered learning during walk engine parameter
     * optimiztion.
     */

    static WalkRequestBlock::ParamSet last_param_set = WalkRequestBlock::PARAMS_NONE;

    if (paramSet == WalkRequestBlock::PARAMS_NONE)
        return;

    if (last_param_set != paramSet)
    {
        setWalkParameters(paramSets[paramSet]);
        last_param_set = paramSet;
    }
}

void UTWalkEngine::setWalkParameters(const WalkEngineParameters &params) {
    params_ = params;

    if (frame_info_->source == MEMORY_SIM) {
        //params_.com_measurement_delay_ = 0.08; // in seconds
        params_.correction_fwd_.rotation = DEG_T_RAD * 0;
    }

    walk_engine_->phase_length_ = params_.phase_length_;
    // tilt pids
    pid_tilt_.setParams(params_.pid_tilt_);
    pid_roll_.setParams(params_.pid_roll_);
    // com pids
    pid_com_x_.setParams(params_.pid_com_x_);
    pid_com_y_.setParams(params_.pid_com_y_);
    pid_com_z_.setParams(params_.pid_com_z_);
    // arm pids
    pid_arm_x_.setParams(params_.pid_arm_x_);
    pid_arm_y_.setParams(params_.pid_arm_y_);
    // step pids
    pid_step_size_x_.setParams(params_.pid_step_size_x_);
    pid_step_size_y_.setParams(params_.pid_step_size_y_);
    pid_step_size_rot_.setParams(params_.pid_step_size_rot_);
}

void UTWalkEngine::processFrame() {
    // copy current joint angles over into commands
    for (int i = 0; i < NUM_JOINTS; i++)
        commands_->angles_[i] = joint_angles_->values_[i];

    tilt_offset_ = pid_tilt_.update(sensors_->values_[angleY],0);
    roll_offset_ = pid_roll_.update(sensors_->values_[angleX],0);
    calcDesiredCOM();

    // do the correct motion
    switch (walk_request_->motion_) {
    case WalkRequestBlock::STAND:
        stand();
        break;
    case WalkRequestBlock::WALK:
        setWalkParameters(walk_request_->paramSet_);
        processWalkRequest();
        walk();
        break;
    case WalkRequestBlock::NONE:
        break;
    default:
        std::cerr << "Error: Unknown walk request type" << std::endl << std::flush;
        exit(1);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
        joint_angles_->values_[i] = commands_->angles_[i];

    // store some information about this frame
    last_frame_motion_ = walk_request_->motion_;
    last_time_ = frame_info_->seconds_since_start;

    ////////////////////////////////////////////////////
    graph_->addData("phase_frac",walk_engine_->phase_frac_);
    // actual com
    graph_->addData("com.x",body_model_->center_of_mass_.x);
    graph_->addData("com.y",body_model_->center_of_mass_.y);
    graph_->addData("com.z",body_model_->center_of_mass_.z);
    graph_->addData("angleX",sensors_->values_[angleX]);
    graph_->addData("angleY",sensors_->values_[angleX]);
    // commanded com
    Pose3D rel_parts[BodyPart::NUM_PARTS];
    Pose3D abs_parts[BodyPart::NUM_PARTS];
    RobotDimensions robot_dimensions;
    MassCalibration mass_calibration;
    Vector3<float> commanded_com;
    float commanded_angleX = sensors_->values_[angleX];
    float commanded_angleY = sensors_->values_[angleY];

    ForwardKinematics::calculateRelativePose(commands_->angles_,commanded_angleX,commanded_angleY,rel_parts,robot_dimensions);
    ForwardKinematics::calculateAbsolutePose(rel_parts,abs_parts);
    ForwardKinematics::calculateCoM(abs_parts,commanded_com,mass_calibration);
    graph_->addData("commanded_com.x",commanded_com.x);
    graph_->addData("commanded_com.y",commanded_com.y);
    graph_->addData("commanded_com.z",commanded_com.z);

    ////////////////////////////////////////////////////
}

void UTWalkEngine::calcDesiredCOM() {
    // desired com
    float angles[NUM_JOINTS];
    Pose3D left_target;
    Pose3D right_target;
    float groin;

    // convert the time using the measurement delay
    float t = walk_engine_->phase_frac_ - params_.com_measurement_delay_ / walk_engine_->phase_length_;  // divison to convert from time to phase time
    bool left_swing = walk_engine_->left_swing_;

    if (t < phase_start_) {
        t = phase_end_ - (phase_start_ - t);
        left_swing = !left_swing;
    }

    // calculate the joint targets
    calcFeetTargets(left_target,right_target,groin,t,left_swing,walk_engine_->step_size_); // TODO step size needs to change?
    //commandLegsRelativeToTorso(angles,left_target,right_target,groin,0,0,Vector3<float>(0,0,0),false);//params_.default_com_pos_,false);
    commandLegsRelativeToTorso(angles,left_target,right_target,groin,0,0,params_.default_com_pos_,false);
    setArms(angles,Vector2<float>(0,0));
    setHead(angles);

    // do the forward kinematics
    Pose3D rel_parts[BodyPart::NUM_PARTS];
    Pose3D abs_parts[BodyPart::NUM_PARTS];
    RobotDimensions robot_dimensions;
    MassCalibration mass_calibration;
    float desired_angleX = 0;
    float desired_angleY = 0;
    ForwardKinematics::calculateRelativePose(angles,desired_angleX,desired_angleY,rel_parts,robot_dimensions);
    ForwardKinematics::calculateAbsolutePose(rel_parts,abs_parts);
    ForwardKinematics::calculateCoM(abs_parts,desired_com_,mass_calibration);

    //graph desired com
    graph_->addData("desired_com.x",desired_com_.x);
    graph_->addData("desired_com.y",desired_com_.y);
    graph_->addData("desired_com.z",desired_com_.z);

    // update the pids
    // TODO I'm not sure if this is the correct way to do things
    com_offset_.x = pid_com_x_.update(body_model_->center_of_mass_.x,0);
    com_offset_.y = pid_com_y_.update(body_model_->center_of_mass_.y,0);
    com_offset_.z = pid_com_z_.update(body_model_->center_of_mass_.z,0);

    // update arm pids
    arm_offset_.x = pid_arm_x_.update(body_model_->center_of_mass_.x,desired_com_.x);
    arm_offset_.y = pid_arm_y_.update(body_model_->center_of_mass_.y,desired_com_.y);
    //arm_offset_.x = 0;
    //arm_offset_.y = 0;
    // update the error between the desired and actual coms
    com_errors_.add(body_model_->center_of_mass_ - desired_com_);
    graph_->addData("avg_com_error.x",com_errors_.getAverage().x);
    graph_->addData("avg_com_error.y",com_errors_.getAverage().y);
    graph_->addData("avg_com_error.z",com_errors_.getAverage().z);
    graph_->addData("com_error.x",com_errors_[0].x);
    graph_->addData("com_error.y",com_errors_[0].y);
    graph_->addData("com_error.z",com_errors_[0].z);
}

void UTWalkEngine::processWalkRequest() {
    if (walk_request_->percentage_speed_) {
        // convert from percentage speeds to step sizes
        walk_engine_->step_size_.translation.x = walk_request_->speed_.translation.x * params_.max_step_size_.translation.x;
        walk_engine_->step_size_.translation.y = walk_request_->speed_.translation.y * params_.max_step_size_.translation.y;
        walk_engine_->step_size_.rotation = walk_request_->speed_.rotation * params_.max_step_size_.rotation;
    } else {
        // convert from speeds to step sizes
        walk_engine_->step_size_.translation.x = walk_request_->speed_.translation.x / walk_engine_->phase_length_;
        walk_engine_->step_size_.translation.y = walk_request_->speed_.translation.y / walk_engine_->phase_length_;
        walk_engine_->step_size_.rotation = walk_request_->speed_.rotation / walk_engine_->phase_length_;
    }

    // if the avg com error is too high, reduce the step size
    Vector3<float> avg_com_error_ = com_errors_.getAverage();
    float x_error_frac = (fabs(avg_com_error_.x) - params_.max_normal_com_error_.x) / (params_.max_acceptable_com_error_.x - params_.max_normal_com_error_.x);
    float y_error_frac = (fabs(avg_com_error_.y) - params_.max_normal_com_error_.y) / (params_.max_acceptable_com_error_.y - params_.max_normal_com_error_.y);
    float error_frac = max(x_error_frac,y_error_frac);
    error_frac = max(0.0,error_frac); // never less than 0
    error_frac = min(1.0,error_frac); // never greater than 1.0
    float slow_frac = 1.0 - (2.0) * (error_frac / (1.0 + error_frac));

    if (slow_frac < 0.999) {
//    std::cout << "TOO MUCH COM ERROR, slowing to " << slow_frac << " of desired speed" << std::endl;
        walk_engine_->step_size_.translation.x *= slow_frac;
        walk_engine_->step_size_.translation.y *= slow_frac;
        walk_engine_->step_size_.rotation *= slow_frac;
    }

    // use the pids to interpolate changes in the commands
    walk_engine_->step_size_.translation.x = walk_engine_->last_step_size_.translation.x + pid_step_size_x_.update(walk_engine_->last_step_size_.translation.x,walk_engine_->step_size_.translation.x);
    walk_engine_->step_size_.translation.y = walk_engine_->last_step_size_.translation.y + pid_step_size_y_.update(walk_engine_->last_step_size_.translation.y,walk_engine_->step_size_.translation.y);
    walk_engine_->step_size_.rotation = walk_engine_->last_step_size_.rotation + pid_step_size_rot_.update(walk_engine_->last_step_size_.rotation,walk_engine_->step_size_.rotation);


    // crop the step sizes to the maximums
    walk_engine_->step_size_.translation.x = crop(walk_engine_->step_size_.translation.x,-params_.max_step_size_.translation.x,params_.max_step_size_.translation.x);
    walk_engine_->step_size_.translation.y = crop(walk_engine_->step_size_.translation.y,-params_.max_step_size_.translation.y,params_.max_step_size_.translation.y);
    walk_engine_->step_size_.rotation = crop(walk_engine_->step_size_.rotation,-params_.max_step_size_.rotation,params_.max_step_size_.rotation);

    walk_engine_->last_step_size_ = walk_engine_->step_size_;

    // apply a correction to the walk based on drift
    float fwd_frac = walk_engine_->step_size_.translation.x / params_.max_step_size_.translation.x;
    walk_engine_->step_size_.translation.x += fwd_frac * params_.correction_fwd_.translation.x;
    walk_engine_->step_size_.translation.y += fwd_frac * params_.correction_fwd_.translation.y;
    walk_engine_->step_size_.rotation += fwd_frac * params_.correction_fwd_.rotation;
}

void UTWalkEngine::initStiffness() {
    // initialize the stiffnesses
    commands_->send_stiffness_ = true;
    commands_->stiffness_time_ = 10;
    for (int i = 0; i < NUM_JOINTS; i++)
        commands_->stiffness_[i] = 0.75;
    commands_->stiffness_[HeadPitch] = 1.0;
    commands_->stiffness_[HeadYaw] = 1.0;
}

void UTWalkEngine::commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float groin, float theta_tilt, float theta_roll, const Vector3<float> &com_offset, bool graph_data) {
    RotationMatrix rot;
    RotationMatrix foot_rotation;

    rot.rotateY(theta_tilt);
    rot.rotateX(theta_roll);
    foot_rotation.rotateY(theta_tilt);
    foot_rotation.rotateX(theta_roll);

    left_target.translation = rot * (com_offset + left_target.translation);
    right_target.translation = rot * (com_offset + right_target.translation);

    left_target.rotation = left_target.rotation * foot_rotation;
    right_target.rotation = right_target.rotation * foot_rotation;

    ///////////////////////////////////////////
    if (graph_data) {
        graph_->addData("left_target.x",left_target.translation.x);
        graph_->addData("left_target.y",left_target.translation.y);
        graph_->addData("left_target.z",left_target.translation.z);
        graph_->addData("right_target.x",right_target.translation.x);
        graph_->addData("right_target.y",right_target.translation.y);
        graph_->addData("right_target.z",right_target.translation.z);
    }
    ///////////////////////////////////////////

    inverse_kinematics_.calcLegJoints(left_target,command_angles,groin,true,robot_dimensions_);
    inverse_kinematics_.calcLegJoints(right_target,command_angles,groin,false,robot_dimensions_);
    command_angles[LHipYawPitch] = groin;
    command_angles[RHipYawPitch] = groin;

    walk_engine_->left_foot_ = left_target;
    walk_engine_->right_foot_ = right_target;
    walk_engine_->groin_ = groin;
}

void UTWalkEngine::stand() {
    if ((last_frame_motion_ != WalkRequestBlock::STAND) && (last_frame_motion_ != WalkRequestBlock::WALK)) {
        initStiffness();
    }

    commandLegsRelativeToTorso(commands_->angles_,Pose3D(0,params_.foot_separation_ * 0.5,-params_.walk_height_),Pose3D(0,-params_.foot_separation_ * 0.5,-params_.walk_height_),0,tilt_offset_,roll_offset_,com_offset_,true);

    setArms(commands_->angles_,arm_offset_);
    setHead(commands_->angles_);

    commands_->angle_time_ = 1000;
    // SAM CHANGED FOR ODOMETRY
    walk_engine_->last_step_size_ = Pose2D();
    walk_engine_->step_size_ = Pose2D();
}

void UTWalkEngine::initWalk() {
    phase_start_ = -0.5;
    phase_end_ = 0.5;

    walk_engine_->phase_frac_ = phase_start_;
    walk_engine_->left_swing_ = true;
    walk_engine_->last_step_size_.translation.x = 0;
    walk_engine_->last_step_size_.translation.y = 0;
    walk_engine_->last_step_size_.rotation = 0;
    initStiffness();
}

void UTWalkEngine::switchPhase() {
    resetTime();
    walk_engine_->left_swing_ = !walk_engine_->left_swing_;
}

void UTWalkEngine::resetTime() {
    double inner = 6 - cosh(params_.k_ * walk_engine_->phase_frac_ * 0.5);
    if (inner >= 1.0)
        phase_start_ = acosh(inner) / (params_.k_ * 0.5);
    else
        phase_start_ = 0.5;
    phase_start_ = -fabs(phase_start_);
    walk_engine_->phase_frac_ = phase_start_;
    phase_end_ = 0.5 * (phase_end_ - phase_start_);
}


float UTWalkEngine::calcOffsetFracTowards(float t, bool towards_swing) {
    if (towards_swing) {
        if (t + 0.5 < params_.fraction_still_)
            return 0;
        else if (t + 0.5 < params_.fraction_still_ + params_.fraction_moving_)
            return 0.5 * (1.0 + cos(M_PI * (t - params_.fraction_still_) / params_.fraction_moving_));
        else
            return 1;
    } else {
        if (t < 0)
            return 0.25 * (1.0 - cos(M_PI * t / 0.5));
        else
            return 0;
    }
}

float UTWalkEngine::calcOffsetFracAway(float t, bool away_swing) {
    if (away_swing) {
        if (t + 0.5 < params_.fraction_still_)
            return -1;
        else if (t + 0.5 < params_.fraction_still_ + params_.fraction_moving_)
            return 0.5 * (-1.0 + cos(M_PI * (t - params_.fraction_still_) / params_.fraction_moving_));
        else
            return 0;
    } else {
        if (t < 0)
            return 0;
        else
            return 0.25 * (cos(M_PI * t / 0.5) - 1);
    }
}

float UTWalkEngine::calcLeftSide(float t, bool left_swing, float step_side_size) {
    float val = 0;
    float feetSeparation = 100;
    if (agentBodyType == 3) {
        feetSeparation = 75;
    }
    if (left_swing)
        val = feetSeparation + calcRightSide(t,left_swing,step_side_size);
    else
        val = feetSeparation/2.0 + params_.shift_amount_ * calcShiftFrac(t);
    if (step_side_size > 0)
        val += step_side_size * calcOffsetFracTowards(t,left_swing);
    else
        val += step_side_size * calcOffsetFracAway(t,left_swing);
    return val;
}

float UTWalkEngine::calcRightSide(float t, bool left_swing, float step_side_size) {
    float val = 0;
    float feetSeparation = 100;
    if (agentBodyType == 3) {
        feetSeparation = 75;
    }
    if (left_swing)
        val = -feetSeparation/2.0 - params_.shift_amount_ * calcShiftFrac(t);
    else
        val = calcLeftSide(t,left_swing,step_side_size) - feetSeparation;
    if (step_side_size > 0)
        val += step_side_size * calcOffsetFracAway(t,!left_swing);
    else
        val += step_side_size * calcOffsetFracTowards(t,!left_swing);
    return val;
}

void UTWalkEngine::calcFeetTargets(Pose3D &left_target, Pose3D &right_target, float &groin, float t, bool left_swing, const Pose2D &step_size) {
    float step_height = params_.step_height_ * calcHeightFrac(t);

    float left = calcLeftSide(t,left_swing,step_size.translation.y);
    float right = calcRightSide(t,left_swing,step_size.translation.y);


    float stance_step_fwd;
    float swing_step_fwd;
    calcFwdFracs(t,stance_step_fwd,swing_step_fwd);
    float fwd_offset =  params_.fwd_offset_ - step_size.translation.x * params_.fwd_offset_factor_; // TODO fix this better?

    groin = calcGroinAngle(t,left_swing,step_size.rotation);

    left_target.translation.y = left;
    right_target.translation.y = right;

    Pose3D *swing_target = &right_target;
    Pose3D *stance_target = &left_target;
    if (left_swing) {
        swing_target = &left_target;
        stance_target = &right_target;
    }

    swing_target->translation.x = swing_step_fwd * step_size.translation.x + fwd_offset;
    stance_target->translation.x = stance_step_fwd * step_size.translation.x + fwd_offset;
    swing_target->translation.z = step_height - params_.walk_height_;
    stance_target->translation.z = -params_.walk_height_;


    // add an angle offset to the swing foot, to make sure it does not hit toe first
    float swing_ankle = params_.swing_ankle_offset_ * calcSwingAnkleFrac(t);
    swing_target->rotation.rotateY(swing_ankle);
}


void UTWalkEngine::walk() {
    if (last_frame_motion_ != WalkRequestBlock::WALK) {
        initWalk();
    }

    walk_engine_->phase_frac_ += ((frame_info_->seconds_since_start - last_time_) / walk_engine_->phase_length_);
    if (walk_engine_->phase_frac_ > phase_end_) {
        switchPhase();
    }

    Pose3D left_target;
    Pose3D right_target;
    float groin;

    calcFeetTargets(left_target,right_target,groin,walk_engine_->phase_frac_,walk_engine_->left_swing_,walk_engine_->step_size_);
    float theta_tilt = pid_tilt_.update(sensors_->values_[angleY],0);
    float theta_roll = pid_roll_.update(sensors_->values_[angleX],0);
    commandLegsRelativeToTorso(commands_->angles_,left_target,right_target,groin,theta_tilt,theta_roll,com_offset_,true);

    // TOES!
    float t = walk_engine_->phase_frac_;
    if (walk_engine_->left_swing_)
        t += 1.0;
    commands_->angles_[LToePitch] = params_.toeConstOffset + params_.toeAmplitude * cos(M_PI * (t + params_.toePhaseOffset));
    commands_->angles_[RToePitch] = params_.toeConstOffset + params_.toeAmplitude * cos(M_PI * (t + 1.0 + params_.toePhaseOffset));
    // ankles to counterbalance toes
    commands_->angles_[LAnklePitch] += params_.ankleConstOffset + params_.ankleAmplitude * cos(M_PI * (t + params_.anklePhaseOffset));
    commands_->angles_[RAnklePitch] += params_.ankleConstOffset + params_.ankleAmplitude * cos(M_PI * (t + 1.0 + params_.anklePhaseOffset));


    setArms(commands_->angles_,arm_offset_);
    setHead(commands_->angles_);

    applyHTWKClosedLoop(commands_->angles_,sensors_->values_[angleY],sensors_->values_[angleX]);

    commands_->angle_time_ = 10;
}

void UTWalkEngine::applyHTWKClosedLoop(float angles[], float bodyTilt, float bodyRoll) {
    angles[LHipPitch] += params_.balanceHipPitch * bodyTilt;
    angles[RHipPitch] += params_.balanceHipPitch * bodyTilt;
    angles[LKneePitch] += params_.balanceKneePitch * bodyTilt;
    angles[RKneePitch] += params_.balanceKneePitch * bodyTilt;
    angles[LHipRoll] += params_.balanceHipRoll * bodyRoll;
    angles[RHipRoll] += params_.balanceHipRoll * bodyRoll;
    angles[LAnkleRoll] += params_.balanceAnkleRoll * bodyRoll;
    angles[RAnkleRoll] += params_.balanceAnkleRoll * bodyRoll;
}

float UTWalkEngine::calcGroinAngle(float t, bool left_swing, float step_turn_size) {
    bool turn_left = true;
    if (step_turn_size < 0) {
        step_turn_size = -step_turn_size; // still want the same direction on the groin
        turn_left = false;
    }
    if (left_swing == turn_left) {
        if (t + 0.5 < params_.fraction_still_)
            return 0;
        else if (t + 0.5 < params_.fraction_still_ + params_.fraction_moving_)
            return -step_turn_size * 0.5 * (1 - cos(M_PI * (t + 0.5 - params_.fraction_still_) / params_.fraction_moving_));
        else
            return -step_turn_size;
    } else {
        if (t + 0.5 < params_.fraction_still_)
            return -step_turn_size;
        else if (t + 0.5 < params_.fraction_still_ + params_.fraction_moving_)
            return -step_turn_size * 0.5 * (1 + cos(M_PI * (t + 0.5 - params_.fraction_still_) / params_.fraction_moving_));
        else
            return 0;
    }
}

float UTWalkEngine::calcSwingAnkleFrac(float t) {
    float fraction_affected = 2 * (1.0 - params_.fraction_on_ground_ - params_.fraction_in_air_);
    if (t + 0.5 < 1.0 - fraction_affected)
        return 0;
    else
        return 0.5 * (1.0 - cos(M_PI * (t + 0.5 - 0.5 * fraction_affected) / (0.5 * fraction_affected)));
}

float UTWalkEngine::calcShiftFrac(float t) {
    float y = -1 + 0.5 * (cosh(params_.k_ * t * 0.5) - 1); // NOTE if you change this, you must change resetTime since it's based on setting the two pendulum positions equal
    return y;
}

float UTWalkEngine::calcHeightFrac(float t) {
    float height = 0;
    t = (t - phase_start_) / (phase_end_ - phase_start_);
    if ((t > params_.fraction_on_ground_) && (t < params_.fraction_on_ground_ + params_.fraction_in_air_)) {
        height = 0.5 * (1 - cos(2 * M_PI * (t - params_.fraction_on_ground_) / params_.fraction_in_air_));
    }
    return height;
}

inline float sigmoid(float t) {
    return 1.0 / (1.0 + exp(-t));
}

float UTWalkEngine::calcSwingFwd(float t) {
    float m = 0.125;
    if (t + 0.5 < params_.fraction_still_) {
        return -0.5 - t + params_.fraction_still_;
    } else if (t + 0.5 < params_.fraction_still_ + params_.fraction_moving_) {
        float scale = 10.0;
        t = scale * (-0.5 + (t + 0.5 - params_.fraction_still_) / params_.fraction_moving_);
        float minVal = sigmoid(-scale * 0.5);
        float maxVal = sigmoid(scale * 0.5);
        return (1 + 2 * params_.fraction_still_) * (-0.5 + (sigmoid(t) - minVal) / (maxVal - minVal)) + 0.5 + params_.fraction_still_;
    } else {
        return 1.5 - t + params_.fraction_still_;
    }
}

void UTWalkEngine::calcFwdFracs(float t, float &stance_fwd, float &swing_fwd) {
    float m = 0.125;
    // calculate swing
    swing_fwd = calcSwingFwd(t);

    // calculate stance
    stance_fwd = 0.5 - t + params_.fraction_still_;
}

void UTWalkEngine::setArms(float *command_angles, const Vector2<float> &arm_offset) {

    command_angles[LShoulderPitch] = DEG_T_RAD*-80 + DEG_T_RAD * arm_offset.x;
    command_angles[RShoulderPitch] = DEG_T_RAD*-80 + DEG_T_RAD * arm_offset.x;

    command_angles[LShoulderRoll] = DEG_T_RAD*15 + DEG_T_RAD * arm_offset.y;
    command_angles[RShoulderRoll] = DEG_T_RAD*15 - DEG_T_RAD * arm_offset.y;

    command_angles[LElbowRoll] = DEG_T_RAD*-15;
    command_angles[RElbowRoll] = DEG_T_RAD*-15;

    command_angles[LElbowYaw] = DEG_T_RAD*-70;
    command_angles[RElbowYaw] = DEG_T_RAD*-70;
}

void UTWalkEngine::setHead(float *command_angles) {
    command_angles[HeadPitch] = 0;
    command_angles[HeadYaw] = 0;
}
