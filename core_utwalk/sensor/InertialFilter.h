#ifndef _InertialFilter_h_DEFINED
#define _InertialFilter_h_DEFINED


#include <common/NMatrix.h>

class InertialFilter {
public:
    InertialFilter();
    void init(bool in_simulation);

    // this takes the data an perfomrs the update
    // the reason I don't pass the variables to processFrame
    // is that I want the class to extendable and we may end up using
    // information from the walk engine, and I'd prefer to not change
    // the inputs to processFrame
    void processFrame();

    // Set the intertial data from the sensors
    void setInertialData(float aX, float aY, float aZ, float gX, float gy);

    // Gets for filtered and raw tilts/rolls and velocities
    float getRoll() {
        return filtRoll;
    };
    float getTilt() {
        return filtTilt;
    };

    float getRollVel() {
        return filtRollVel;
    };
    float getTiltVel() {
        return filtTiltVel;
    };

    float getRawRoll() {
        return rawRoll;
    };
    float getRawTilt() {
        return rawTilt;
    };

    //------ Variables ----- // Note: I generally don't believe in public/private
private:
    // inputs and outputs
    float x_acc;
    float y_acc;
    float z_acc;
    float x_gyro;
    float y_gyro;

    float filtRoll;
    float filtTilt;
    float filtRollVel;
    float filtTiltVel;

    float rawRoll;
    float rawTilt;

    // needed for calculation
    bool isActive;          // Filter takes 30 frames before being correct
    int framesProcessed;

    float gyro_scale;  // (deg/sec)
    float iu_height; // (m)

    float gyro_noise_rms; // (deg/sec)
    float acc_noise_rms; // (m/s^2)
    float motion_rms; // (rad/s^3)
    float gyro_drift_rms; // rad/sample

    // Gyro offsets
    int numFramesToMeanOver;
    float sum_x_gyro;
    float sum_y_gyro;
    float mean_x_gyro;
    float mean_y_gyro;

    // KF variables
    float dt;  // Time bewtween sensor readings
    NMatrix A;
    NMatrix B;
    NMatrix C;
    NMatrix Cz;
    NMatrix Cz1;

    NMatrix Q;  // Weighting Matricies
    NMatrix R;
    NMatrix L;

    NMatrix xhRoll;
    NMatrix xhTilt;
};

#endif

