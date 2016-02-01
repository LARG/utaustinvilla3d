//Note : only do single value updates, do not update with a matrix of values!!!

#include "OrigKalmanFilter.h"
#include <math/Geometry.h>

//#include "stdafx.h"
//#include "..\Globals.h"

#include <iostream>
using namespace std;

/*import matrix
import Common
import math*/

// Note: for examples, assume 5 states, 1 input, 1 outputs

OrigKalmanFilter::OrigKalmanFilter() {
    numStates = 0;
    I = NMatrix(1,1,false);
    initP = NMatrix(1,1,false);
    initX = NMatrix(1,1,false);
    P = NMatrix(1,1,false);
    X = NMatrix(1,1,false);
    Xchange = NMatrix(1,1,false);

    active = false;    // Is the model currrently in use ?
    activate = false;
    alpha = 1.0;
}

bool OrigKalmanFilter::Start(short n, NMatrix& uncert, NMatrix& initStates) {
    if (uncert.getm() != n || uncert.getn() != n || initStates.getm() != n || initStates.getn() != 1) {
        CompilerError("Incorrect matrix dimensions in method Start()");
        return false;
    }
    //E.g. uncert is 5x5, initStates = 5x1
    numStates = n;
    I = NMatrix(n, n, true);
    initP = uncert;
    initX = initStates;
    alpha = 1.0;
    active = false;    // Is the model currrently in use ?
    activate = false;
    return Restart();
    return true;
}

bool OrigKalmanFilter::Restart() {
    P = initP;
    X = initX;
    Xchange = NMatrix(numStates, 1, false);
    // Anything else that needs to be reset should be done here
    return true;
}

bool OrigKalmanFilter::TimeUpdate(NMatrix& A, NMatrix& B, NMatrix& U, NMatrix& Q, bool mainFilterUpdate) { // Set mainFilterUpdate to false unless X[2][0] is the orientation of the robot
    if (A.getm() != numStates || A.getn() != numStates || B.getm() != numStates || B.getn() != U.getm() || U.getn() != 1 || Q.getm() != numStates || Q.getn() != numStates) {
        CompilerError("Incorrect matrix dimensions in method TimeUpdate()");
        return false;
    }
    //E.g. A is 5x5, B is 5x1, U is 1x1, & Q is 5x5
    X = A*X + B*U;
    if (mainFilterUpdate) X[2][0] = normalizeAngle(X[2][0]);
    P = A*P*A.transp() + Q;
    Xchange = NMatrix(numStates, 1, false);
    for (int  i = 0; i < numStates; i++) Xchange[i][0] = 0;
    return true;
}

bool OrigKalmanFilter::TimeUpdateExtended(NMatrix& A, NMatrix& Xbar, NMatrix& Q) { //A = df/dx|x=x(k) where Xbar = f(x(k))
    if (A.getm() != numStates || A.getn() != numStates || Xbar.getm() != numStates || Xbar.getn() != 1 || Q.getm() != numStates || Q.getn() != numStates) {
        CompilerError("Incorrect matrix dimensions in method TimeUpdateExtended()");
        return false;
    }
    //E.g. A is 5x5, X is 5x1, & Q is 5x5
    X = Xbar;
    P = A*P*A.transp() + Q;
    Xchange = NMatrix(numStates, 1, false);
    for (int  i = 0; i < numStates; i++) Xchange[i][0] = 0;
    return true;
}

int OrigKalmanFilter::MeasurementUpdate(NMatrix& C, float R, float Y, bool rejectOutliers, float outlierSD, bool mainFilterAngleUpdate) { // Set mainFilterAngleUpdate to false unless this is an angle update operation and X[2][0] is the orientation of the robot
    if (C.getn() != numStates || C.getm() != 1) {
        CompilerError("Incorrect matrix dimensions in method MeasurementUpdate()");
        cout << "Incorrect matrix dimensions in method MeasurementUpdate()" << endl << flush;
        return KF_CRASH;
    }
    //E.g. C is 1x5
    float HX = convDble(C*X);
    float innovation = Y - HX;
    if (mainFilterAngleUpdate) innovation = normalizeAngle(innovation);
    Xchange = Xchange - X;
    float posVar = convDble(C*P*C.transp());
    if (posVar < 0.0) {
        Reset();
        posVar = convDble(C*P*C.transp());
        cout << "OrigKalmanFilter reset due to negative variance" << endl << flush;
    }
    float varPredError = posVar + R;
    if (rejectOutliers && (abs(innovation) > powf(outlierSD,2)*sqrtf(varPredError))) return KF_OUTLIER;
    NMatrix J = P*C.transp()/varPredError; //J is now X.M x Y.M e.g. 5x1
    NMatrix newP = (I - J*C)*P;

    /*
    for (int i = 0; i < numStates; i++) {
      if (newP[i][i] <= 0) {
        cout << "Numerics error at frame " << frameNumber << endl << flush;
        Reset();
        return MeasurementUpdate(C, R, Y, rejectOutliers, outlierSD, mainFilterAngleUpdate);
      }
      for (int j = i+1; j < numStates; j++)
        if (newP[i][j]*newP[i][j] > newP[i][i]*newP[j][j]) {
          cout << "Numerics error at frame " << frameNumber << endl << flush;
          Reset();
          return MeasurementUpdate(C, R, Y, rejectOutliers, outlierSD, mainFilterAngleUpdate);
        }
    }
    */

    X = X + J*innovation;
    P = newP;
    Xchange = Xchange + X;
    return KF_SUCCESS;
}

int OrigKalmanFilter::MeasurementUpdateExtended(NMatrix& C, float R, float Y, float Ybar, bool rejectOutliers, float outlierSD, bool mainFilterAngleUpdate, bool ignoreLongRangeUpdate, float deadzoneSize, float dist, bool ambigObj, bool changeAlpha) { // Set mainFilterAngleUpdate to false unless this is an angle update operation and X[2][0] is the orientation of the robot
    if (C.getn() != numStates || C.getm() != 1) {
        CompilerError("Incorrect matrix dimensions in method MeasurementUpdateExtended()");
        cout << "Incorrect matrix dimensions in method MeasurementUpdateExtended()" << endl << flush;
        return KF_CRASH;
    }
    //E.g. C is 1x5
    float innovation = Y - Ybar;
    float posVar = convDble(C*P*C.transp());

    if (mainFilterAngleUpdate) {
        innovation = normalizeAngle(innovation);
    }

//RHM 7/7/07 Shift till after detection of outliers etc.
// if (changeAlpha) {
//   if (ambigObj) {
//     alpha*=MAX(((R)/(R+innovation*innovation)),0.01); //0.1);
//   } else {
//     alpha*=(R)/(R+innovation*innovation);
//   }
// }

    //if (ambigObj) {
    //  if (mainFilterAngleUpdate) Output2Doubles((R)/(R+innovation*innovation),0.0);
    //  else Output2Doubles((R)/(R+innovation*innovation),1.0);
    //}

    if (mainFilterAngleUpdate) {
        R += square((P[0][0]+P[1][1])/(dist*dist));    // Moved from object update into here on 22/3/2007
    }

    Xchange = Xchange - X;

    if (posVar < 0.0) {
        Reset();
        posVar = convDble(C*P*C.transp());
        cout << "OrigKalmanFilter reset due to negative variance" << endl << flush;
    }
    // add in deadzone calculations: RHM 1/6/06
    Deadzone(&R, &innovation, posVar, deadzoneSize);

    float varPredError = posVar + R;
    if (ignoreLongRangeUpdate && (innovation > S_D_RANGE_REJECT*sqrtf(varPredError))) {
        cout << "Ignore Long range update" << endl << flush;
        // R = R*4;
        alpha *= 0.5; //RHM 7/7/07
        return KF_SUCCESS;
    }
    if (rejectOutliers && (powf(innovation,2) > powf(outlierSD,2)*varPredError)) {
        alpha*=0.5; //RHM 7/7/07
        return KF_OUTLIER;
    }
// RHM 7/7/07: Shifted alpha changes to here
    if (changeAlpha) {
        if (ambigObj) {
            alpha *= (R)/(R+innovation*innovation);
//	    alpha *= max(((R)/(R+innovation*innovation)),0.01); //0.1);
        } else {
            alpha *= (R)/(R+innovation*innovation);
        }
    }


    NMatrix J = P*C.transp()/varPredError; //J is now X.M x Y.M e.g. 5x1
//  if (!mainFilterAngleUpdate) J[2][0] = 0;
    NMatrix Xbar = X;
    NMatrix newP = (I - J*C)*P;

    /*
    for (int i = 0; i < numStates; i++) {
      if (newP[i][i] <= 0) {
        cout << "Numerics error at frame " << frameNumber << endl << flush;
        Reset();
        return MeasurementUpdateExtended(C, R, Y, Ybar, rejectOutliers, outlierSD, mainFilterAngleUpdate, ignoreLongRangeUpdate, deadzoneSize, dist, ambigObj,changeAlpha);
      }
      for (int j = i+1; j < numStates; j++)
        if (newP[i][j]*newP[i][j] > newP[i][i]*newP[j][j]) {
          cout << "Numerics error at frame " << frameNumber << ", OrigKalmanFilter reset" << endl << flush;
          Reset();
          return MeasurementUpdateExtended(C, R, Y, Ybar, rejectOutliers, outlierSD, mainFilterAngleUpdate, ignoreLongRangeUpdate, deadzoneSize, dist, ambigObj,changeAlpha);
        }
    }
    */

    X = Xbar + J*innovation;
    P = newP;
    Xchange = Xchange + X;
    return KF_SUCCESS;
}

void OrigKalmanFilter::Reset() {
    P = initP;
}

NMatrix OrigKalmanFilter::GetStates() {
    return X;
}

void OrigKalmanFilter::SetStates(NMatrix Xbar) {
    X = Xbar;
}

float OrigKalmanFilter::GetState(short n) {
    return X[n][0];
}

void OrigKalmanFilter::SetState(short n, float x) {
    X[n][0] = x;
}

void OrigKalmanFilter::NormaliseState(short n) {
    X[n][0] = normalizeAngle(X[n][0]);
}

NMatrix OrigKalmanFilter::GetErrorMatrix() {
    return P;
}

void OrigKalmanFilter::SetErrorMatrix(NMatrix Pbar) {
    P = Pbar;
}

float OrigKalmanFilter::GetCovariance(short m, short n) {
    return P[m][n];
}

float OrigKalmanFilter::GetVariance(short n) {
    return P[n][n];
}

NMatrix OrigKalmanFilter::GetXchanges() {
    return Xchange;
}

float OrigKalmanFilter::GetXchange(short n) {
    return Xchange[n][0];
}

void OrigKalmanFilter::CompilerError(const char* str) {
    cout << str << endl << flush;
}

void OrigKalmanFilter::Deadzone(float* R, float* innovation, float CPC, float eps)
{
    float invR;
// R is the covariance of the measurement (altered by this procedure)
// innovation is the prediction error (altered by this procedure)
// CPC is the variance of the predicted y value (not altered)
// eps is the value of the deadzone
// all vars except innovation must e positive for this to work

//Return if eps or CPC or R non-positive
    if ((eps<1.0e-08) || (CPC<1.0e-08) || (*R<1e-08))
        return;

    if (abs(*innovation)>eps)
    {   //RHM 5/6/06 ?extra fix to Deadzones
        invR=(abs(*innovation)/eps-1)/CPC; // adjust R when outside the deadzone to not jump too far
        //return;
    }
    else
    {
        // If we get to here, then valid parameters passed, and we
        // are within the specified accuracy

        //cout << "Deadzone  " << eps << "   " << *innovation << "\n" << flush;
        *innovation=0.0; //Set to zero to force no update of parameters
        invR=0.25/(eps*eps)-1.0/CPC;
    }

    if (invR<1.0e-08) //RHM 5/6/06 decrease from 1.0e-06
        invR=1e-08;

// only ever increase R
    if ( *R < 1.0/invR )
        *R=1.0/invR;
// R is adjusted (when in the deadzone) so that the a-posteriori variance of the
// prediction is 4*eps*eps;
// or, (if we are outside the deadzone), so that the new prediction at most just reaches
// the deadzone.
}



