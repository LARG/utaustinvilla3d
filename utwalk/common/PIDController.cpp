#include "PIDController.h"

PIDController::PIDController():
    cp_(1.0),
    ci_(0),
    cd_(0),
    current_error_(0),
    cumulative_error_(0),
    previous_error_(0)
{
}

PIDController::PIDController(float cp, float ci, float cd):
    cp_(cp),
    ci_(ci),
    cd_(cd),
    current_error_(0),
    cumulative_error_(0),
    previous_error_(0)
{
}

float PIDController::update(float current, float target) {
    previous_error_ = current_error_;
    current_error_ = target - current;
    cumulative_error_ += current_error_;

    return cp_ * current_error_ + ci_ * cumulative_error_ + cd_ * (current_error_ - previous_error_);
}

void PIDController::setParams(float cp, float ci, float cd) {
    cp_ = cp;
    ci_ = ci;
    cd_ = cd;
}

void PIDController::setParams(const Vector3<float> &params) {
    cp_ = params.x;
    ci_ = params.y;
    cd_ = params.z;
}
