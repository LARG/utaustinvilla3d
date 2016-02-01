#ifndef _PLAYERKF_H
#define _PLAYERKF_H

#include "OrigKalmanFilter.h"
#include <common/NMatrix.h>
#include "../math/Geometry.h"
#include "../headers/Field.h"
#include "../worldmodel/WorldObject.h"
#include "../worldmodel/worldmodel.h"
#include <stdarg.h>

#define MAX_PLAYER_MODELS_UT NUM_AGENTS

class TextLogger;

class FrameInfoBlock;
class LocalizationBlock;
class OdometryBlock;
class RobotStateBlock;
class WorldObjectBlock;

class PlayerKF {
public:
    PlayerKF(WorldModel* worldModel_);
    ~PlayerKF();

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
    void playerMeasurementUpdate(int modelNumber);
    void normalPlayerMeasurementUpdate(int modelNumber);
    void closePlayerMeasurementUpdate(int modelNumber);
    void grabUpdate(int modelNumber);
    void teammateUpdate(SIM::Point2D playerLoc, SIM::Point2D playerSD);
    void updatePlayerFromKF(int modelNumber);
    SIM::Point2D getRobotPosition(int modelNumber);
    void clipPosition(int modelNumber);
    void resetFilter(int modelNumber);
    void setPlayer(int modelNumber, float x, float y);

    // helper function
    float getPlayerDistanceVariance(float playerDistance);
    void logPlayerMatrix(int modelNumber, int loglevel);
    void log(int logLevel, const char* format, ...);

    // Jacobian Methods
    NMatrix GetPlayerDistanceJacobian(float, float, float, float,
                                      float, float);
    NMatrix GetPlayerBearingJacobian(float, float, float, float, float, float);
    NMatrix GetPlayerXRelJacobian(float, float, float, float, float);
    NMatrix GetPlayerYRelJacobian(float, float, float, float, float);

    NMatrix GetXPositionJacobian();
    NMatrix GetYPositionJacobian();
    NMatrix GetThetaPositionJacobian();

    /*
      X[0][0] = posex
      X[1][0] = posey
      X[2][0] = posetheta
      X[3][0] = playerx
      X[4][0] = playery
      X[5][0] = playerxvel
      X[6][0] = playeryvel
    */

    bool playerDebug;

    // Parameters
    // Initial Filter parameters ..
    int numStates;
    NMatrix uncert;
    NMatrix initStates;

    // Variance Parameters
    float positionVar;
    float angleVar;
    float playerPosVar;
    float playerVelVar;
    float unseenPlayerVarFactor;

    float sdDist;
    float sdBearing;

    // other stuff
    bool useRelPlayer;
    float relPlayerThreshold;
    float minPlayerDistance;
    float pfVarianceFactor;

    // some decays and variances
    float velocityDecay;


    OrigKalmanFilter playerModel[MAX_PLAYER_MODELS_UT];
};


#endif //_PLAYERKF_H
