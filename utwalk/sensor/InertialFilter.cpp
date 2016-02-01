#include "InertialFilter.h"

#include <math.h>
#include <iostream>
using namespace std;

#define RAD_TO_DEG  180.0/ M_PI
#define DEG_TO_RAD  M_PI/180.0

InertialFilter::InertialFilter() {
}

void InertialFilter::init(bool in_simulation) {
    x_acc = 0.0001;
    y_acc = 0.0001;
    z_acc = 0.0001;
    x_gyro = 0.0001;
    y_gyro = 0.0001;

    filtRoll=0.0;
    filtTilt=0.0;
    filtRollVel=0.0;
    filtTiltVel=0.0;

    rawRoll=0.0;
    rawTilt=0.0;

    isActive = false;
    framesProcessed = 0;

    gyro_scale = DEG_TO_RAD*0.405;  // (rad/sec)
    iu_height = 0.35;  // (m)

    gyro_noise_rms = 3.0 *  DEG_TO_RAD;  // (rad/sec)
    acc_noise_rms = 1.0;   //  (m/s^2)
    motion_rms = 1.0;      // (rad/s^3)
    gyro_drift_rms = 0.0001;  //rad/sample


    numFramesToMeanOver = 30;
    sum_x_gyro=0.0;
    sum_y_gyro=0.0;
    mean_x_gyro=0.0;
    mean_y_gyro=0.0;

    // Init KF
    dt = 0.02; // 50 fps, but this could be recorded not hardcoded
    A = NMatrix(4,4,false);
    B = NMatrix(4,2,false);
    C = NMatrix(2,4,false);
    Cz = NMatrix(1,4,false);
    Cz1 = NMatrix(1,4,false);

    A[0][0]=1;
    A[0][1]=dt;
    A[0][2]=(dt*dt)/2.0;
    A[0][3]=0;
    A[1][0]=0;
    A[1][1]=1;
    A[1][2]=dt;
    A[1][3]=0;
    A[2][0]=0;
    A[2][1]=0;
    A[2][2]=1;
    A[2][3]=0;
    A[3][0]=0;
    A[3][1]=0;
    A[3][2]=0;
    A[3][3]=1;

    B[0][0]=(dt*dt*dt)/6.0;
    B[0][1]=0;
    B[1][0]=(dt*dt)/2.0;
    B[1][1]=0;
    B[2][0]=dt;
    B[2][1]=0;
    B[3][0]=0;
    B[3][1]=1;

    C[0][0]=9.81;
    C[0][1]=0;
    C[0][2]=iu_height;
    C[0][3]=0;
    C[1][0]=0;
    C[1][1]=1;
    C[1][2]=0;
    C[1][3]=1;

    Cz[0][0]=1;
    Cz[0][1]=0;
    Cz[0][2]=0;
    Cz[0][3]=0;

    Cz1[0][0]=0;
    Cz1[0][1]=1;
    Cz1[0][2]=0;
    Cz1[0][3]=0;

    //Weighting Matrices
    NMatrix diag1(2,2,false);
    diag1[0][0]=motion_rms*motion_rms;
    diag1[1][1]=gyro_drift_rms*gyro_drift_rms;
    Q=B*diag1*B.transp();

    R = NMatrix(2,2,false);
    R[0][0]=acc_noise_rms*acc_noise_rms;
    R[1][1]=gyro_noise_rms*gyro_noise_rms;

    // L = L=dlqe(A,eye(size(A)),C,Q,R);
    // values are precoded below but they rely
    // on the constants defined above not changing
    L = NMatrix(4,2,false);
    if (in_simulation) {
        // only 50 hz
        L[0][0]=0.002208826688060;
        L[0][1]=0.014373714070774;
        L[1][0]=0.001709942995628;
        L[1][1]=0.113822914154786;
        L[2][0]=0.002303923878533;
        L[2][1]=0.355802778034262;
        L[3][0]=-0.000981957939151;
        L[3][1]=0.002359546154730;
    } else {
        // running at 100 hz
        L[0][0]= 0.000681046696253;
        L[0][1]=0.008190299932203;
        L[1][0]= 0.000495744650085;
        L[1][1]=0.059037533294317;
        L[2][0]= 0.001121001609691;
        L[2][1]=0.183895602697463;
        L[3][0]=-0.000099013137273;
        L[3][1]=0.000222817196833;
    }

    xhRoll = NMatrix(4,1,false);
    xhTilt = NMatrix(4,1,false);
}



void InertialFilter::processFrame() {
    framesProcessed++;

    // First do some work on the accelerometer data
    // float abs_acc=sqrtf(x_acc*x_acc+y_acc*y_acc+z_acc*z_acc); why
    // define and cause warning? remove thisss!!!
    rawTilt=-atan2(x_acc,z_acc);
    rawRoll=atan2(y_acc,z_acc);

    // We need to calculate the mean of the initial gyro offset
    if (framesProcessed<=numFramesToMeanOver) {
        sum_x_gyro+=x_gyro;
        sum_y_gyro+=y_gyro;

        if (framesProcessed==numFramesToMeanOver) { // we can do the filter
            isActive = true;
            mean_x_gyro = sum_x_gyro/framesProcessed;
            mean_y_gyro = sum_y_gyro/framesProcessed;
            xhRoll[0][0]=rawRoll; // start the filter at the position of the raw data
            xhTilt[0][0]=rawTilt;
        } else {
            // set the filtered to be the raw and return
            filtRoll=rawRoll;
            filtTilt=rawTilt;
            return;
        }
    }
    //-----------------------

    // Now work with the gyros
    float x_g=(x_gyro-mean_x_gyro) * gyro_scale;
    float y_g=(y_gyro-mean_y_gyro) * gyro_scale;

    //Now do KF calculations for roll axis
    NMatrix xSen(2,1,false);
    xSen[0][0] = y_acc;
    xSen[1][0] = x_g;
    xhRoll=A*xhRoll-L*(C*xhRoll - xSen);
    filtRoll=convDble(Cz*xhRoll);
    filtRollVel=convDble(Cz1*xhRoll);

    //Now do KF calculations for tilt axis
    NMatrix ySen(2,1,false);
    ySen[0][0] = -x_acc;
    ySen[1][0] = y_g;
    xhTilt=A*xhTilt-L*(C*xhTilt - ySen);
    filtTilt=convDble(Cz*xhTilt);
    filtTiltVel=convDble(Cz1*xhTilt);
}

void InertialFilter:: setInertialData(float aX, float aY, float aZ, float gX, float gy) {
    x_acc = aX;
    y_acc = aY;
    z_acc = aZ;
    x_gyro = gX;
    y_gyro = gy;
}


