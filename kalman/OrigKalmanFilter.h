#ifndef _ORIGKALMANFILTER_H
#define _ORIGKALMANFILTER_H

#include <math.h>
#include <common/NMatrix.h>

//#include "../FieldObj.h"
//#include "../Globals.h"
//#include "Parameters.h"

#define KF_CRASH 0 // Matrix dimensions error, check your code!
#define KF_NUMERICS 1 // Bad error matrix, reset
#define KF_OUTLIER 2 // Outlier rejected
#define KF_SUCCESS 3 // Success!

#define S_D_RANGE_REJECT 2

class OrigKalmanFilter {
public:
    OrigKalmanFilter();

    bool Start(short numStates, NMatrix& uncert, NMatrix& intStates);
    bool Restart();
    bool TimeUpdate(NMatrix& A, NMatrix& B, NMatrix& U, NMatrix& Q, bool mainFilterUpdate);
    bool TimeUpdateExtended(NMatrix& A, NMatrix& Xbar, NMatrix& Q);
    int MeasurementUpdate(NMatrix& C, float R, float Y, bool rejectOutliers, float outlierError, bool mainFilterAngleUpdate);
    int MeasurementUpdateExtended(NMatrix& C, float R, float Y, float Ybar, bool rejectOutliers, float outlierError, bool mainFilterAngleUpdate, bool ignoreLongRangeUpdate, float deadzoneSize, float dist, bool ambigObject, bool changeAlpha);
    void Reset();
    NMatrix GetStates();
    void SetStates(NMatrix Xbar);
    float GetState(short n);
    void SetState(short n, float x);
    void NormaliseState(short n);
    NMatrix GetErrorMatrix();
    void SetErrorMatrix(NMatrix Pbar);
    float GetCovariance(short m, short n);
    float GetVariance(short n);
    NMatrix GetXchanges();
    float GetXchange(short n);
    void CompilerError(const char* str);

    void Deadzone(float* R, float* innovation, float CPC, float eps);

    short numStates;
    NMatrix I;
    NMatrix initX;
    NMatrix initP;
    NMatrix X;
    NMatrix P;
    NMatrix Xchange;

    // ------ New Stuff for multiple models

    // Control and model Evaluation
    bool active;    // Is the model currrently in use ?
    bool activate;
    float alpha;   // The probability that the model is correct (0->1)

};

#endif// _KALMANFILTER_H
