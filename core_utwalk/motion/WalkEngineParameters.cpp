#include "WalkEngineParameters.h"

#include <math/Geometry.h>

#include <map>
#include <stdio.h>
extern map<string, string> namedParams; // from main.cc

static double
getParam(const map<string, string>& namedParams, const string name)
{
    map<string, string>::const_iterator it = namedParams.find(name);
    if ( it == namedParams.end())
        throw "In \"WalkEngineParameters\": Missing parameter: " + name;

    return atof(it->second.c_str());
}

WalkEngineParameters::WalkEngineParameters(const string& prefix):
    // max speeds
//  max_step_size_(DEG_T_RAD*70,50,40),
    max_step_size_(getParam(namedParams, prefix + "utwalk_max_step_size_angle"),
                   getParam(namedParams, prefix + "utwalk_max_step_size_x"),
                   getParam(namedParams, prefix + "utwalk_max_step_size_y")),
    // dimensions of walk
//  shift_amount_(20), // ?? might be different for fast and slow walk ??
    shift_amount_(getParam(namedParams, prefix + "utwalk_shift_amount")), // ?? might be different for fast and slow walk ??
    foot_separation_(2 * 50), // fixed parameter
//  walk_height_(175),
    walk_height_(getParam(namedParams, prefix + "utwalk_walk_height")),
//  step_height_(20),
    step_height_(getParam(namedParams, prefix + "utwalk_step_height")),
    // timing of moving swing foot
//  fraction_still_(0.20),
    fraction_still_(getParam(namedParams, prefix + "utwalk_fraction_still")),
    //fraction_moving_(1.0 - 2 * fraction_still_), // fixed parameter
    fraction_moving_(getParam(namedParams, prefix + "utwalk_fraction_moving")), // fixed parameter
    // timing of raising swing foot
//  fraction_on_ground_(0.20), // probably should be the utwalk as fraction_still_
    fraction_on_ground_(getParam(namedParams, prefix + "utwalk_fraction_on_ground")), // probably should be the utwalk as fraction_still_
    //fraction_in_air_(1.0 - 2 * fraction_on_ground_),
    fraction_in_air_(getParam(namedParams, prefix + "utwalk_fraction_in_air")),
    // other
//  phase_length_(0.38),
    phase_length_(getParam(namedParams, prefix + "utwalk_phase_length")),
    k_(sqrtf(9806.65f / walk_height_)), // sqrt(g/h) // fixed parameter
    //swing_ankle_offset_(DEG_T_RAD * -5), // if open for opt., start from 0
    swing_ankle_offset_(getParam(namedParams, prefix + "utwalk_swing_ankle_offset")),
    com_measurement_delay_(0.02),
    //com_measurement_delay_(getParam(namedParams, prefix + "utwalk_com_measurement_delay")),

    // pid tilt and roll params
    //pid_tilt_(0.15,0,0.01),
    pid_tilt_(getParam(namedParams, prefix + "utwalk_pid_tilt"),0,0),
    //pid_roll_(0.2,0,0),
    pid_roll_(getParam(namedParams, prefix + "utwalk_pid_roll"),0,0),

    // pid com params
    //pid_com_x_(1.0,0,0),
    pid_com_x_(getParam(namedParams, prefix + "utwalk_pid_com_x"),0,0),
    //pid_com_y_(1.0,0,0),
    pid_com_y_(getParam(namedParams, prefix + "utwalk_pid_com_y"),0,0),
    //pid_com_z_(0.0,0,0),
    pid_com_z_(getParam(namedParams, prefix + "utwalk_pid_com_z"),0,0),

    // pid arm params
    //pid_arm_x_(1.0,0,0),
    pid_arm_x_(getParam(namedParams, prefix + "utwalk_pid_arm_x"),0,0),
    //pid_arm_y_(1.0,0,0),
    pid_arm_y_(getParam(namedParams, prefix + "utwalk_pid_arm_y"),0,0),

    // pid step params
//  pid_step_size_x_(0.03,0,0), // related to how smoothly to accelerate
    pid_step_size_x_(getParam(namedParams, prefix + "utwalk_pid_step_size_x"),0,0), // related to how smoothly to accelerate
//  pid_step_size_y_(0.03,0,0), // related to how smoothly to accelerate
    //pid_step_size_y_(getParam(namedParams, prefix + "utwalk_pid_step_size_x"),0,0), // related to how smoothly to accelerate
    pid_step_size_y_(getParam(namedParams, prefix + "utwalk_pid_step_size_y"),0,0), // related to how smoothly to accelerate
//  pid_step_size_rot_(0.03,0,0), // related to how smoothly to accelerate
    pid_step_size_rot_(getParam(namedParams, prefix + "utwalk_pid_step_size_rot"),0,0), // related to how smoothly to accelerate
    // default com
//  default_com_pos_(-15,0,0), // definitely open for opt.// <<<<<<
    default_com_pos_(getParam(namedParams, prefix + "utwalk_default_com_pos_x"),0,0), // definitely open for opt.

    // corrections to commands (for example turning right when walking forwards
    correction_fwd_(0,0,DEG_T_RAD * 0), // fixed parameter
    // handling com error using step size
//  max_normal_com_error_(7.5,7.5), // open these// <<<<<<
    max_normal_com_error_(getParam(namedParams, prefix + "utwalk_max_normal_com_error"), getParam(namedParams, prefix + "utwalk_max_normal_com_error")),
//  max_acceptable_com_error_(12.5,12.5)// open these// <<<<<<
    max_acceptable_com_error_(getParam(namedParams, prefix + "utwalk_max_acceptable_com_error"),getParam(namedParams, prefix + "utwalk_max_acceptable_com_error")),
//  fwd_offset_(2.5),
    fwd_offset_(getParam(namedParams, prefix + "utwalk_fwd_offset")),
//  fwd_offset_factor_(0.5),
    fwd_offset_factor_(getParam(namedParams, prefix + "utwalk_fwd_offset_factor")),
    //  balanceHipPitch(0),
    balanceHipPitch(getParam(namedParams,prefix + "utwalk_balance_hip_pitch")),
    // balanceKneePitch(0),
    balanceKneePitch(getParam(namedParams,prefix + "utwalk_balance_knee_pitch")),
    // balanceHipRoll(0),
    balanceHipRoll(getParam(namedParams,prefix + "utwalk_balance_hip_roll")),
    // balanceAnkleRoll(0),
    balanceAnkleRoll(getParam(namedParams,prefix + "utwalk_balance_ankle_roll")),
    // toeConstOffset(0),
    toeConstOffset(getParam(namedParams,prefix + "utwalk_toe_const_offset")),
    // toeAmplitude(0),
    toeAmplitude(getParam(namedParams,prefix + "utwalk_toe_amplitude")),
    // toePhaseOffset(0),
    toePhaseOffset(getParam(namedParams,prefix + "utwalk_toe_phase_offset")),
    // ankleConstOffset(0),
    ankleConstOffset(getParam(namedParams,prefix + "utwalk_ankle_const_offset")),
    //ankleAmplitude(0),
    ankleAmplitude(getParam(namedParams,prefix + "utwalk_ankle_amplitude")),
    //anklePhaseOffset(0)
    anklePhaseOffset(getParam(namedParams,prefix + "utwalk_ankle_phase_offset"))
{
}
