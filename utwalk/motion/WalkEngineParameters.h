#ifndef WALKENGINEPARAMETERS_5GLKWI3S
#define WALKENGINEPARAMETERS_5GLKWI3S

#include <math/Pose2D.h>
#include <math/Vector3.h>
#include <string>

struct WalkEngineParameters {
    /**
     * Same as the default constructor, but append a prefix to all parameter names.
     */
    WalkEngineParameters(const std::string& prefix = "");

    // all distances refer to mm unless otherwise specified
    // angles are all in radians
    // times are all in seconds

    // max speeds
    Pose2D max_step_size_;
    // dimensions of walk
    float shift_amount_; // half width for now
    float foot_separation_;
    float walk_height_;
    float step_height_;
    // timing of moving swing foot
    float fraction_still_;
    float fraction_moving_;
    // timing of raising swing foot
    float fraction_on_ground_;
    float fraction_in_air_;
    // other
    float phase_length_; // seconds for 1 step by 1 foot
    float k_; // sqrt(g/walk_height_)
    float swing_ankle_offset_;
    float com_measurement_delay_; // in seconds
    // pid tilt and roll params
    Vector3<float> pid_tilt_;
    Vector3<float> pid_roll_;
    // pid com params
    Vector3<float> pid_com_x_;
    Vector3<float> pid_com_y_;
    Vector3<float> pid_com_z_;
    // pid arm params
    Vector3<float> pid_arm_x_;
    Vector3<float> pid_arm_y_;
    // pid step params
    Vector3<float> pid_step_size_x_;
    Vector3<float> pid_step_size_y_;
    Vector3<float> pid_step_size_rot_;

    // default com
    Vector3<float> default_com_pos_;

    // corrections to commands (for example turning right when walking forwards
    Pose2D correction_fwd_;

    // handling com error using step size
    Vector2<float> max_normal_com_error_;
    Vector2<float> max_acceptable_com_error_;

    float fwd_offset_;
    float fwd_offset_factor_;

    float balanceHipPitch;
    float balanceKneePitch;
    float balanceHipRoll;
    float balanceAnkleRoll;

    // TOES!
    float toeConstOffset;
    float toeAmplitude;
    float toePhaseOffset;
    float ankleConstOffset;
    float ankleAmplitude;
    float anklePhaseOffset;
};

#endif /* end of include guard: WALKENGINEPARAMETERS_5GLKWI3S */
