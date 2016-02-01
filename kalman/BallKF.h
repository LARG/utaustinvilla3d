#ifndef _BALLKF_H
#define _BALLKF_H

#include "OrigKalmanFilter.h"
#include <common/NMatrix.h>
#include "../math/Geometry.h"
#include "../headers/Field.h"
#include "../worldmodel/WorldObject.h"
#include "../worldmodel/worldmodel.h"
#include <stdarg.h>

#define MAX_MODELS_UT 1

class TextLogger;

class FrameInfoBlock;
class LocalizationBlock;
class OdometryBlock;
class RobotStateBlock;
class WorldObjectBlock;

class BallKF {
public:
    BallKF(WorldModel* worldModel_);
    ~BallKF();

    // init
    void init();
    void initConstants();
    void initModel(int modelNumber);
    void setTextLogger(TextLogger* tf);

    WorldModel* worldModel;

    TextLogger *text_logger_;

    // memory pointers
    FrameInfoBlock *vision_frame_info_;
    LocalizationBlock *localization_mem_;
    OdometryBlock *odometry_;
    RobotStateBlock *robot_state_;
    WorldObjectBlock *world_objects_;

    float prev_time_;
    void processFrame();
    // updates
    void timeUpdate(int modelNumber, float timePassed);
    void poseMeasurementUpdate(int modelNumber);
    void ballMeasurementUpdate(int modelNumber);
    void normalBallMeasurementUpdate(int modelNumber);
    void closeBallMeasurementUpdate(int modelNumber);
    void kickUpdate(int modelNumber, float vel, SIM::AngRad bearing);
    void grabUpdate(int modelNumber);
    void teammateUpdate(SIM::Point2D ballLoc, SIM::Point2D ballSD);
    void updateBallFromKF(int modelNumber);
    SIM::Point2D getRobotPosition(int modelNumber);
    void clipPosition(int modelNumber);
    void resetFilter(int modelNumber);
    void setBall(float x, float y);

    // helper function
    float getBallDistanceVariance(float ballDistance);
    void logBallMatrix(int loglevel);
    void log(int logLevel, const char* format, ...);

    // Jacobian Methods
    NMatrix GetBallDistanceJacobian(float, float, float, float,
                                    float, float);
    NMatrix GetBallBearingJacobian(float, float, float, float, float, float);
    NMatrix GetBallXRelJacobian(float, float, float, float, float);
    NMatrix GetBallYRelJacobian(float, float, float, float, float);

    NMatrix GetXPositionJacobian();
    NMatrix GetYPositionJacobian();
    NMatrix GetThetaPositionJacobian();

    /*
      X[0][0] = posex
      X[1][0] = posey
      X[2][0] = posetheta
      X[3][0] = ballx
      X[4][0] = bally
      X[5][0] = ballxvel
      X[6][0] = ballyvel
    */

    bool ballDebug;

    // Parameters
    // Initial Filter parameters ..
    int numStates;
    NMatrix uncert;
    NMatrix initStates;

    // Variance Parameters
    float positionVar;
    float angleVar;
    float ballPosVar;
    float ballVelVar;
    float unseenBallVarFactor;
    float kickVelVar;

    float sdDist;
    float sdBearing;

    // other stuff
    bool useRelBall;
    float relBallThreshold;
    float minBallDistance;
    float pfVarianceFactor;

    // some decays and variances
    float velocityDecay;


    OrigKalmanFilter ballModel[MAX_MODELS_UT];
};


#endif //_BALLKF_H
