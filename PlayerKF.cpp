#include "PlayerKF.h"
#include <iostream>
//#include <memory/TextLogger.h>


#include <cstdlib> // abs didn't compile

/*
  X[0][0] = posex
  X[1][0] = posey
  X[2][0] = posetheta
  X[3][0] = playerx
  X[4][0] = playery
  X[5][0] = playerxvel
  X[6][0] = playeryvel
*/

PlayerKF::PlayerKF(WorldModel* worldModel_) {
    worldModel = worldModel_;
    init();
}

PlayerKF::~PlayerKF() {

}


void PlayerKF::init() {
    prev_time_ = -1;

    initConstants();
    for (int i = 0; i < MAX_PLAYER_MODELS_UT; i++) {
        initModel(i);
    }
}


/** Initialize the model for the KF */
void PlayerKF::initConstants() {

    playerDebug = false; //true;

#ifdef TOOL
    playerDebug = true;
#endif

    numStates = 7;

    uncert = NMatrix(7,7,false);
    // Standard deviation is the length
    uncert[0][0] = SIM::square(FIELD_X / 2);
    uncert[1][1] = SIM::square(FIELD_Y / 2);
    uncert[2][2] = SIM::square(2 * M_PI);
    uncert[3][3] = SIM::square(FIELD_X / 2);
    uncert[4][4] = SIM::square(FIELD_Y / 2);
    // Standard deviation of vel is 1000mm/s
    uncert[5][5] = SIM::square(1000);
    uncert[6][6] = SIM::square(1000);

    initStates = NMatrix(7,1,false);
    // Initially robot and player at (almost) the centre of the field,
    //  the robot facing up the field
    initStates[0][0] = -0.001;
    initStates[1][0] = -0.001;
    initStates[2][0] = 0;
    initStates[3][0] = 0.001;
    initStates[4][0] = 0.001;
    initStates[5][0] = 0;
    initStates[6][0] = 0;

    // when to use close player measurement update (rel x/y)
    // vs the normal distance/bearing one
    useRelPlayer = true;
    relPlayerThreshold = 500.0;

    // some decays and variances
    velocityDecay = 0.95;

    // for time update
    positionVar = 400.0;
    angleVar =    500.0;
    playerPosVar = 300.0;
    playerVelVar = 3000.0;

    // multiple player uncertainty by this if its unseen
    unseenPlayerVarFactor = 6.0;// OPEN
    minPlayerDistance = 40.0;
    pfVarianceFactor = 1000000.0;

    // for measurement update
    sdDist = 5.0; // distance measurement variance // OPEN
//  sdBearing = 5.0; (Not Used) // bearing measurement variance

}


/** Init the kf with the appropriate initial X and P. */
void PlayerKF::initModel(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // start it up!
    currModel->Start(numStates, uncert, initStates);

    if (playerDebug) {
        log(50, "Player Init Model");
        logPlayerMatrix(modelNumber, 50);
    }


}

void PlayerKF::processFrame() {
    double timePassed = worldModel->getTime() - prev_time_;
    if (prev_time_ < 0) {
        // Case for initialization
        timePassed = 1;
    }
    prev_time_ = worldModel->getTime();

    for (int modelNumber = 0; modelNumber < MAX_PLAYER_MODELS_UT; modelNumber++) {
        // player - time update
        //cout << "Time passed: " << timePassed << endl;
        timeUpdate(modelNumber, timePassed);

        // update the pose in the kf using pf estimate
        poseMeasurementUpdate(modelNumber);

        // player - measurement update
        playerMeasurementUpdate(modelNumber);

        // possibly clip position
        //player.clipPosition(0);

        // set player kf estimate into player world object
        updatePlayerFromKF(modelNumber);
    }
}

/** Perform the time update on the KalmanFilter model[modelNumber]
    - knowing that timePassed seconds have passed. */
void PlayerKF::timeUpdate(int modelNumber, float timePassed) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get current estimate and covariance
    NMatrix X = currModel->GetStates();
    NMatrix P = currModel->GetErrorMatrix();

    // new state matrix after time update
    NMatrix Xbar = NMatrix(7, 1, false);

    // get odometry
    // DANIEL CHOOSE EITHER GLOBAL OR LOCAL
    bool useGlobalAsOdometry = false;
    SIM::Point2D odom;
    SIM::AngRad odomTurn;
    if( useGlobalAsOdometry ) {
        VecPosition curPos = worldModel->getMyPosition();
        VecPosition lastPos = worldModel->getMyLastPosition();
        VecPosition diffPos = curPos - lastPos;
        SIM::AngRad curAng = worldModel->getMyAngRad();
        SIM::AngRad lastAng = worldModel->getMyLastAngRad();
        SIM::AngRad diffAng = curAng - lastAng;
        odom = SIM::Point2D(diffPos.getX(),diffPos.getY());
        odomTurn = diffAng;
    }
    else {
        odom = worldModel->getLastOdometryPos();
        odomTurn = Deg2Rad(worldModel->getLastOdometryAngDeg());
    }

    // bad odometry
    if (fabs(odomTurn) > 1.0 || fabs(odom.x) > 100 || fabs(odom.y) > 100.0) {
        odom.x = 0.0;
        odom.y = 0.0;
        odomTurn = 0.0;
    }


    // update Xbar with odometry
    // hack: don't update robot pose with odom
    //Xbar[0][0] = X[0][0] + odomForward*cosf(X[2][0]) - odomLeft*sinf(X[2][0]);
    //Xbar[1][0] = X[1][0] + odomForward*sinf(X[2][0]) + odomLeft*cosf(X[2][0]);
    //Xbar[2][0] = X[2][0] + odomTurn;
    Xbar[0][0] = X[0][0];
    Xbar[1][0] = X[1][0];
    Xbar[2][0] = X[2][0];

    // update player position by negative of odometry
    SIM::Point2D playerPos = SIM::Point2D(X[3][0], X[4][0]);
    playerPos = playerPos - odom;
    // rotate by odom turn
    playerPos.rotate(-odomTurn);

    // player position is updated by velocity
    Xbar[3][0] = playerPos.x + X[5][0] * timePassed;
    Xbar[4][0] = playerPos.y + X[6][0] * timePassed;

    // player velocity is decayed
    Xbar[5][0] = X[5][0] * velocityDecay;
    Xbar[6][0] = X[6][0] * velocityDecay;

    // time update jacobian
    // derivative of the update to xbar from x
    // note A is already 1 on the diaganol
    NMatrix A = NMatrix(7, 7, true);
    A[0][2] = 0; //-odomForward*sinf(X[2][0]) - odomLeft*cosf(X[2][0]);
    A[1][2] = 0; //odomForward*cosf(X[2][0]) - odomLeft*sinf(X[2][0]);
    A[3][5] = timePassed;
    A[4][6] = timePassed;
    A[5][5] = velocityDecay;
    A[6][6] = velocityDecay;

    // time update variance matrix: how much should variance increase each time step
    NMatrix Q = NMatrix(7, 7, false);

    Q[0][0] = positionVar;
    Q[1][1] = positionVar;
    Q[2][2] = angleVar;

    Q[3][3] = playerPosVar;
    Q[4][4] = playerPosVar;
    Q[5][5] = playerVelVar;
    Q[6][6] = playerVelVar;

    // hack: add these to player instead
    Q[3][3] += 0.1*(odom.getMagnitude() * odom.getMagnitude());
    Q[4][4] += 0.1*(odom.getMagnitude() * odom.getMagnitude());
    Q[3][3] += 0.1*(odomTurn*odomTurn);
    Q[4][4] += 0.1*(odomTurn*odomTurn);

    //Q[0][0] += 0.1*(odomLeft*odomLeft + odomForward*odomForward);
    //Q[1][1] += 0.1*(odomLeft*odomLeft + odomForward*odomForward);
    //Q[2][2] += 0.1*(odomTurn*odomTurn);

    // TODO: ricks q value fiddle?

    // increase player pos uncertainty if we haven't seen it?
    WorldObject* player = worldModel->getWorldObject(WO_OPPONENT1+modelNumber);
    if (!player->currentlySeen && (!player->haveSighting || !player->validPosition)) {
        Q[3][3] *= unseenPlayerVarFactor;
        Q[4][4] *= unseenPlayerVarFactor;
    }

    // perform the actual update
    currModel->TimeUpdateExtended(A, Xbar, Q);
    currModel->NormaliseState(2);

    if (playerDebug) {
        log(50, "Player Time Update");
        logPlayerMatrix(modelNumber, 50);
    }


}





/** Perform a measurement update on the robot pose based
    on the current estimtae from the particle filter. */
void PlayerKF::poseMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // measurement jacobians
    NMatrix Cx = NMatrix(1, 7, false);
    NMatrix Cy = NMatrix(1, 7, false);
    NMatrix Ctheta = NMatrix(1, 7, false);

    // object pointers
    //  WorldObject* self = &(current->commonMem->worldObjects[current->commonMem->WO_SELF]);

    // get new x,y,theta position from particle filter
    // hack: set these to 0
    /*
    float newX = self->loc.getX();
    float newY = self->loc.getY();
    float newTheta = self->orientation;
    */

    float newX = 0.0;
    float newY = 0.0;
    float newTheta = 0.0;

    // estimate from kf
    float estX = X[0][0];
    float estY = X[1][0];
    float estTheta = X[2][0];

    // estimate of variance our x,y,theta from particle filter
    // hack: set these to 1
    /*
    float Rx = square(self->sd.getX()) / pfVarianceFactor;
    float Ry = square(self->sd.getY()) / pfVarianceFactor;
    float Rtheta = square(self->sdOrientation) / pfVarianceFactor;
    */

    float Rx = 0.001;
    float Ry = 0.001;
    float Rtheta = 0.001;

    // get Jacobians for x, y, theta
    // TODO: what should these jacobians be?
    Cx = GetXPositionJacobian();
    Cy = GetYPositionJacobian();
    Ctheta = GetThetaPositionJacobian();

    if (playerDebug) {
        log(50, "Pose Update: newX: %5.0f, estX: %5.0f, newY: %5.0f, estY: %5.0f, newTheta: %5.0f, estTheta: %5.0f",
            newX, estX, newY, estY, newTheta, estTheta);
    }

    // x update
    // TODO: what do we put in to this call to measurement update extended?
    int updateSuccess =
        currModel->MeasurementUpdateExtended(Cx, Rx,
                newX, estX,
                false, sdDist, false, false, 0.0,
                1.0, false, false);

    if (updateSuccess == KF_CRASH) {
        cout << "X pose update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    // y update
    updateSuccess =
        currModel->MeasurementUpdateExtended(Cy, Ry,
                newY, estY,
                false, sdDist, false, false, 0.0,
                1.0, false, false);
    if (updateSuccess == KF_CRASH) {
        cout << "Y pose update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    // theta update
    updateSuccess =
        currModel->MeasurementUpdateExtended(Ctheta, Rtheta,
                newTheta, estTheta,
                false, sdBearing, true, false, 0.0,
                1.0, false, false);
    if (updateSuccess == KF_CRASH) {
        cout << "Pose theta update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    if (playerDebug) {
        log(60, "Player Pose Measurement Update");
        logPlayerMatrix(modelNumber, 60);
    }


}



/** Perform the measurement update on the KalmanFilter &(playerModel[modelNumber])
    using the worldObjects info for player/self info. */
void PlayerKF::playerMeasurementUpdate(int modelNumber) {


    // decide if player is too close (and we want to use rel x/y for update)
    //   or far enough for normal update (dist/bearing)
    WorldObject* player = worldModel->getWorldObject(WO_OPPONENT1+modelNumber);

    // no update here if we didn't see the player
    if (!player->currentlySeen && (!player->haveSighting || !player->validPosition))
        return;

    // Todd: for some reason normal update is going crazy
    // use relative x,y player measurement update only!
    closePlayerMeasurementUpdate(modelNumber);

    //cout << "Player Measurement Update: "
    //   <<  (currModel->GetStates())[3][0] << ", "
    //   <<  (currModel->GetStates())[4][0] << endl << flush;

    if (playerDebug) {
        log(50, "Player Measurement Update");
        logPlayerMatrix(modelNumber, 50);
    }
}



/** Perform normal player measurement update based on distance and bearing. */
void PlayerKF::normalPlayerMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // measurement jacobians
    NMatrix Cdist = NMatrix(1, 7, false);
    NMatrix Cbearing = NMatrix(1, 7, false);

    // object pointers
    WorldObject* player = worldModel->getWorldObject(WO_OPPONENT1+modelNumber);
    //WorldObject* self = &(current->commonMem->worldObjects[current->commonMem->WO_SELF]);

    // get estimate of global player location from our pose
    // and distance/bearing to player
    double visionDistance = player->vision.polar.getX();
    double visionBearing = Deg2Rad( player->vision.polar.getY() );
    SIM::Point2D relLoc = SIM::Point2D(visionDistance, visionBearing, POLAR);

    // hack: global loc is rel loc
    SIM::Point2D gloplayeroc = relLoc;
    //SIM::Point2D gloplayeroc = relLoc.relativeToGlobal(self->loc, self->orientation);

    // sanity check of player
    // TODO: possibly add distance/bearing sanity checks as well
    // hack: dont do this, player can be off field
    /*
    if (!GRASS.isInside(gloplayeroc)){
      //cout << "Invalid player " << gloplayeroc << endl << flush;
      return;
    }
    */

    // estimate of variance of player distance
    //  from michael's empirical estimate
    // in cm
    float Rdist = getPlayerDistanceVariance(visionDistance);

    SIM::Point2D estPlayer = SIM::Point2D(X[3][0], X[4][0]);
    float estDist = estPlayer.getMagnitude();
    float estBearing = estPlayer.getDirection();

    //float estDist = sqrtf(square(X[3][0] - X[0][0]) + square(X[4][0] - X[1][0]));
    if (estDist < minPlayerDistance) estDist = minPlayerDistance;

    // get measurement jacobian for distance
    Cdist = GetPlayerDistanceJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                      X[5][0], X[6][0]);

    // TODO: did we get a circle fit?
    bool circleFit = true;

    // estimate variance of player bearing
    float Rbearing = 0.02; // default sd of 18 deg
    if (visionDistance < 0) Rbearing = 0.04; // sd of 36 deg
    else if (visionDistance > 1000) Rbearing = 0.001; // sd of 2 deg
    else if (visionDistance > 300 && circleFit) Rbearing = 0.004; // sd of 4 deg
    else if (visionDistance > 300 && !circleFit) Rbearing = 0.02; // sd of 8 deg



    // get measurement jacobian for bearing
    Cbearing = GetPlayerBearingJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                        X[5][0], X[6][0]);

    //float estBearing = normalizeAngle(atan2(X[4][0] - X[1][0],
    //				   X[3][0] - X[0][0]) - X[2][0]);


    if (playerDebug) {
        log(50, "Player seen at distance %5.0f, expected %5.0f, bearing %5.0f, expected %5.0f, gloplayeroc: (%5.0f, %5.0f)",
            visionDistance, estDist, visionBearing, estBearing,
            gloplayeroc.x, gloplayeroc.y);
    }

    // update the model
    // distance update
    int updateSuccess =
        currModel->MeasurementUpdateExtended(Cdist, Rdist,
                abs(visionDistance), estDist,
                false, sdDist, false, false, 0.0,
                visionDistance, false, false);

    if (updateSuccess == KF_CRASH) {
        cout << "Distance update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    // bearing update
    updateSuccess =
        currModel->MeasurementUpdateExtended(Cbearing, Rbearing,
                visionBearing, estBearing,
                false, sdBearing, true, false, 0.0,
                visionDistance, false, false);
    // TODO: reject outliers flag true/false?

    if (updateSuccess == KF_CRASH) {
        cout << "Bearing update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

}


/** Perform close player measurement update based on relative x/y to player. */
void PlayerKF::closePlayerMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // object pointers
    WorldObject* player = worldModel->getWorldObject(WO_OPPONENT1+modelNumber);

    // X
    // relative x estimate
    NMatrix Cxrel = GetPlayerXRelJacobian(X[0][0], X[1][0], X[2][0],
                                          X[3][0], X[4][0]);


    double visionDistance = player->vision.polar.getX();
    double visionBearing = Deg2Rad( player->vision.polar.getY() );


    // estimate of variance of player x distance
    // in cm
    float Rxrel = getPlayerDistanceVariance(visionDistance);

    // actual value of xrel
    float xrel = -abs(visionDistance) * sinf(visionBearing);

    // projected value for xr
    float xrelbar = - (X[4][0]);


    // Y
    // relative y estimate
    NMatrix Cyrel = GetPlayerYRelJacobian(X[0][0], X[1][0], X[2][0],
                                          X[3][0], X[4][0]);

    // estimate of variance of player y distance
    // in cm
    float Ryrel = getPlayerDistanceVariance(visionDistance);

    // actual value of yrel
    float yrel = abs(visionDistance) * cosf(visionBearing);

    // projected value for yr
    float yrelbar = (X[3][0]);

    if (playerDebug) {
        log(50, "Player seen at distance %5.0f, bearing %5.0f, relX: %5.0f, relxbar %5.0f, relY %5.0f, relYbar %5.0f",
            visionDistance, visionBearing, xrel, xrelbar, yrel, yrelbar);
    }


    // update the model
    // relative x update
    int updateSuccess =
        currModel->MeasurementUpdateExtended(Cxrel, Rxrel,
                xrel, xrelbar,
                false, sdDist, false, false, 0.0,
                visionDistance, false, false);

    if (updateSuccess == KF_CRASH) {
        cout << "Relative X update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    // relative y update
    updateSuccess =
        currModel->MeasurementUpdateExtended(Cyrel, Ryrel,
                yrel, yrelbar,
                false, sdDist, false, false, 0.0,
                visionDistance, false, false);

    if (updateSuccess == KF_CRASH) {
        cout << "Relative Y update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);


}


/** Update the player when we're grabbing it. */
void PlayerKF::grabUpdate(int modelNumber) {

    // update player velocity with this
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // measurement jacobians
    NMatrix Cdist = NMatrix(1, 7, false);
    NMatrix Cbearing = NMatrix(1, 7, false);

    // object pointers
    //  WorldObject* player = &(current->commonMem->worldObjects[WO_OPPONENT1+modelNumber]);
    //WorldObject* self = &(current->commonMem->worldObjects[current->commonMem->WO_SELF]);

    // get estimate of global player location from our pose
    // and distance/bearing to player
    SIM::Point2D relLoc = SIM::Point2D(200, 0);

    // hack: global loc is rel loc
    SIM::Point2D gloplayeroc = relLoc;
    //SIM::Point2D gloplayeroc = relLoc.relativeToGlobal(self->loc, self->orientation);



    // estimate of variance of player distance
    //  from michael's empirical estimate
    // in cm
    float Rdist = getPlayerDistanceVariance(200);

    float estDist = sqrtf(SIM::square(X[3][0] - X[0][0]) + SIM::square(X[4][0] - X[1][0]));
    if (estDist < minPlayerDistance) estDist = minPlayerDistance;

    // get measurement jacobian for distance
    Cdist = GetPlayerDistanceJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                      X[5][0], X[6][0]);

    // estimate variance of player bearing
    float Rbearing = 0.004; // sd of 4 deg

    // get measurement jacobian for bearing
    Cbearing = GetPlayerBearingJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                        X[5][0], X[6][0]);

    float estBearing = SIM::normalizeAngle(atan2(X[4][0] - X[1][0],
                                           X[3][0] - X[0][0]) - X[2][0]);

    if (playerDebug) {
        log(50, "Player grab at rel: (%5.0f, %5.0f), global (%5.0f, %5.0f)",
            relLoc.x, relLoc.y, gloplayeroc.x, gloplayeroc.y);
    }


    // update the model
    // distance update
    int updateSuccess =
        currModel->MeasurementUpdateExtended(Cdist, Rdist,
                abs(200), estDist,
                false, sdDist, false, false, 0.0,
                200, false, false);

    if (updateSuccess == KF_CRASH) {
        cout << "Distance update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

    // bearing update
    updateSuccess =
        currModel->MeasurementUpdateExtended(Cbearing, Rbearing,
                200, estBearing,
                false, sdBearing, true, false, 0.0,
                200, false, false);
    // TODO: reject outliers flag true/false?

    if (updateSuccess == KF_CRASH) {
        cout << "Bearing update crashed" << endl << flush;
    }
    currModel->NormaliseState(2);

}


/** Set the player world object with the new estimates from the kf. */
void PlayerKF::updatePlayerFromKF(int modelNumber) {
    if (playerDebug) log(50, "Updating Player v2");
    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // pointer to player object
    WorldObject* player = worldModel->getWorldObject(WO_OPPONENT1+modelNumber);

    // info about robot's pose
//  WorldObject* self = &(world_objects_->objects_[robot_state_->WO_SELF]);

    // get estimate and covariance matrix from filter
    NMatrix X = currModel->GetStates();
    NMatrix P = currModel->GetErrorMatrix();

    // get player rel loc and sd
    SIM::Point2D playerRelLoc = SIM::Point2D(X[3][0], X[4][0]);
//  SIM::Point2D playerRelSD = SIM::Point2D(sqrtf(P[3][3]), sqrtf(P[4][4]));

    // changed by Sam Barrett (6/13/11)
    player->vision.polar.setX(playerRelLoc.getMagnitude());
    player->vision.polar.setY(Rad2Deg(playerRelLoc.getDirection()));
//  player->relPos = playerRelLoc;
//  player->relOrientation = playerRelLoc.getDirection();
//cerr << "WARNING: USING KF VELOCITY - MIGHT BE WRONG?" << endl;
//  // get global loc and sd from this
//  player->loc = playerRelLoc.relativeToGlobal(self->loc, self->orientation);
//  // add in our sd
//  player->sd =  playerRelSD.rotate(self->orientation);
//  // make sure this is still positive after rotation
//  player->sd.x = fabs(player->sd.x);
//  player->sd.y = fabs(player->sd.y);
//  player->sd = player->sd + self->sd;
//
//  // get rel velocity of player from KF
//  player->relVel = SIM::Point2D(X[5][0], X[6][0]);

    // get player's absolute velocity
    player->absVel = (SIM::Point2D(X[5][0], X[6][0])).rotate(worldModel->getMyAngRad());

//  // get distance and bearing to player
//  player->distance = playerRelLoc.getMagnitude();
//  player->bearing = playerRelLoc.getDirection();

    //cout << "Filtered rel player: " << playerRelLoc << " d: " << player->distance
    // << " b: " << player->bearing << " v: " << player->relVel << endl;
    //cout << "Convert to abs " << player->loc << " v: " << player->absVel << endl;

    /* hack: set these different since player is now relative, not global

    // set location and sd of player
    player->loc = SIM::Point2D(X[3][0], X[4][0]);
    player->sd = SIM::Point2D(sqrtf(P[3][3]), sqrtf(P[4][4]));

    // set absolute velocity of player
    player->absVel = SIM::Point2D(X[5][0], X[6][0]);

    // set relative distance/bearing to player
    player->distance = myloc.getDistanceTo(player->loc);
    player->bearing = myloc.getBearingTo(player->loc, myorient);

    // set relative velocity of player
    player->relVel = player->loc.globalToRelative( myloc, myorient );

    */
}


/** Get the kf's estimate of the robot location
    (for comparison with particle filter) */
SIM::Point2D PlayerKF::getRobotPosition(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get estimate and covariance matrix from filter
    NMatrix X = currModel->GetStates();

    return SIM::Point2D(X[0][0], X[1][0]);
}


/** Clip the player position if its off the field. */
void PlayerKF::clipPosition(int modelNumber) {

    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    // get matrices
    NMatrix X = currModel->GetStates();
    NMatrix P = currModel->GetErrorMatrix();
    float mult;

    float maxX = FIELD_X / 2.0 + 1;
    float maxY = FIELD_Y / 2.0 + 1;
    float minX = -maxX;
    float minY = -maxY;

    // Condition 1: y < minY
    if (X[1][0] < minY) {
        mult = (X[1][0] - minY)/P[1][1];
        X[0][0] = X[0][0] - P[0][1]*mult;
        X[1][0] = minY;
        X[2][0] = X[2][0] - P[2][1]*mult;
        X[3][0] = X[3][0] - P[3][1]*mult;
        X[4][0] = X[4][0] - P[4][1]*mult;
        X[5][0] = X[5][0] - P[5][1]*mult;
        X[6][0] = X[6][0] - P[6][1]*mult;
    }

    // Condition 2: y > maxY
    if (X[1][0] > maxY) {
        mult = (X[1][0] - maxY)/P[1][1];
        X[0][0] = X[0][0] - P[0][1]*mult;
        X[1][0] = maxY;
        X[2][0] = X[2][0] - P[2][1]*mult;
        X[3][0] = X[3][0] - P[3][1]*mult;
        X[4][0] = X[4][0] - P[4][1]*mult;
        X[5][0] = X[5][0] - P[5][1]*mult;
        X[6][0] = X[6][0] - P[6][1]*mult;
    }

    // Condition 3: x < minX
    if (X[0][0] < minX) {
        mult = (X[0][0] - minX)/P[0][0];
        X[0][0] = minX;
        X[1][0] = X[1][0] - P[1][0]*mult;
        X[2][0] = X[2][0] - P[2][0]*mult;
        X[3][0] = X[3][0] - P[3][0]*mult;
        X[4][0] = X[4][0] - P[4][0]*mult;
        X[5][0] = X[5][0] - P[5][0]*mult;
        X[6][0] = X[6][0] - P[6][0]*mult;
    }

    // Condition 4: x > maxX
    if (X[0][0] > maxX) {
        mult = (X[0][0] - maxX)/P[0][0];
        X[0][0] = maxX;
        X[1][0] = X[1][0] - P[1][0]*mult;
        X[2][0] = X[2][0] - P[2][0]*mult;
        X[3][0] = X[3][0] - P[3][0]*mult;
        X[4][0] = X[4][0] - P[4][0]*mult;
        X[5][0] = X[5][0] - P[5][0]*mult;
        X[6][0] = X[6][0] - P[6][0]*mult;
    }

    currModel->SetStates(X);
    currModel->NormaliseState(2);

}


/** Resets the Kalman Filter */
void PlayerKF::resetFilter(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    currModel->Reset();

}



/** Returns estimate of player distance variance */
float PlayerKF::getPlayerDistanceVariance(float playerDistance) {

    // convert distance from mm to cm
    playerDistance /= 10.0;

    // Michael's empirical results
    // sd - do in mm
    float Rdist = log10(playerDistance) * playerDistance / 10.0;
    Rdist = SIM::crop(Rdist, 3, 200);

    // increase dist variance if its negative
    if (playerDistance < 0) {
        Rdist *= 4;
    }
    Rdist = SIM::square(Rdist);
    if (Rdist < 0.0001) Rdist = 0.0001;

    // convert from cm^2 to mm^2
    Rdist = Rdist * 100.0;
    return Rdist;

}


void PlayerKF::logPlayerMatrix(int modelNumber, int loglevel) {

    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    NMatrix X = currModel->GetStates();

    log(loglevel, "Player Matrix: X: Pos (%5.0f, %5.0f) Vel(%5.0f, %5.0f)",
        X[3][0], X[4][0], X[5][0], X[6][0]);

    NMatrix P = currModel->GetErrorMatrix();

    log(loglevel, "Player Matrix: P: Pos (%5.0f, %5.0f) Vel(%5.0f, %5.0f)",
        P[3][3], P[4][4], P[5][5], P[6][6]);

}


///////////////
// Jacobians //
///////////////

/* not doing pose updates
NMatrix LocWM::GetDistanceJacobian(float xb, float yb, float x, float y) {
  float dist = sqrtf((x-xb)*(x-xb) + (y-yb)*(y-yb));
  if (dist == 0) dist = 0.00001;
  NMatrix C = NMatrix(1,7,false);
  C[0][0] = (x-xb)/dist;
  C[0][1] = (y-yb)/dist;
  C[0][2] = 0;
  C[0][3] = 0;
  C[0][4] = 0;
  C[0][5] = 0;
  C[0][6] = 0;
  return C;
}

NMatrix LocWM::GetAngleJacobian(float xb, float yb, float x, float y) {
  float distSqrd = (x-xb)*(x-xb) + (y-yb)*(y-yb);
  if (distSqrd == 0) distSqrd = 0.00001;
  NMatrix C = NMatrix(1,7,false);
  C[0][0] = (yb-y)/distSqrd;
  C[0][1] = (x-xb)/distSqrd;
  C[0][2] = -1;
  C[0][3] = 0;
  C[0][4] = 0;
  C[0][5] = 0;
  C[0][6] = 0;
  return C;
}
*/

NMatrix PlayerKF::GetXPositionJacobian() {

    NMatrix C = NMatrix(1,7,false);

    // TODO: what goes here!!??
    C[0][0] = 1;
    C[0][1] = 0;
    C[0][2] = 0;
    C[0][3] = 0;
    C[0][4] = 0;
    C[0][5] = 0;
    C[0][6] = 0;
    return C;

}

NMatrix PlayerKF::GetYPositionJacobian() {

    NMatrix C = NMatrix(1,7,false);

    // TODO: what goes here!!??
    C[0][0] = 0;
    C[0][1] = 1;
    C[0][2] = 0;
    C[0][3] = 0;
    C[0][4] = 0;
    C[0][5] = 0;
    C[0][6] = 0;
    return C;

}

NMatrix PlayerKF::GetThetaPositionJacobian() {

    NMatrix C = NMatrix(1,7,false);

    // TODO: what goes here!!??
    C[0][0] = 0;
    C[0][1] = 0;
    C[0][2] = 1;
    C[0][3] = 0;
    C[0][4] = 0;
    C[0][5] = 0;
    C[0][6] = 0;
    return C;

}

NMatrix PlayerKF::GetPlayerDistanceJacobian(float xb, float yb, float x, float y,
        float vx, float vy) {
    float dist = sqrtf(SIM::square(xb) + SIM::square(yb));
    if (dist == 0) dist = 0.00001;
    NMatrix C = NMatrix(1,7,false);
    C[0][0] = 0; //-(xb-x)/dist;
    C[0][1] = 0; //-(yb-y)/dist;
    C[0][2] = 0;
    C[0][3] = xb /dist; //(xb-x)/dist;
    C[0][4] = yb /dist; //(yb-y)/dist;
    C[0][5] = 0;
    C[0][6] = 0;

    // to kill warnings
    vx = 0;
    vy = 0;
    x = 0;
    y = 0;

    return C;
}

NMatrix PlayerKF::GetPlayerBearingJacobian(float xb, float yb, float x, float y,
        float vx, float vy) {
    float distSqrd = SIM::square(xb) + SIM::square(yb);
    if (distSqrd == 0) distSqrd = 0.00001;
    NMatrix C = NMatrix(1,7,false);
    C[0][0] = 0; //(yb-y)/distSqrd;
    C[0][1] = 0; //-(xb-x)/distSqrd;
    C[0][2] = 0; //-1;
    C[0][3] = -yb/distSqrd; //-C[0][0];
    C[0][4] = xb/distSqrd; //-C[0][1];
    C[0][5] = 0;
    C[0][6] = 0;

    // to kill warnings
    vx = 0;
    vy = 0;
    x = 0;
    y = 0;

    return C;
}

// FLOZ
NMatrix PlayerKF::GetPlayerXRelJacobian(float x, float y, float th, float xb, float yb ) {
    NMatrix C = NMatrix(1,7,false);
    C[0][0] = 0; //-sinf(th);
    C[0][1] = 0; //cosf(th);
    C[0][2] = 0; //cosf(th)*(xb-x)+sinf(th)*(yb-y);
    C[0][3] = 0.0 ; //sinf(th);
    C[0][4] = -1.0; //-cosf(th);
    C[0][5] = 0;
    C[0][6] = 0;

    // kill warnings
    x = 0;
    y = 0;
    th = 0;
    xb = 0;
    yb = 0;

    return C;
}

NMatrix PlayerKF::GetPlayerYRelJacobian(float x, float y, float th, float xb, float yb ) {
    NMatrix C = NMatrix(1,7,false);
    C[0][0] = 0; //-cosf(th);
    C[0][1] = 0; //-sinf(th);
    C[0][2] = 0; //-sinf(th)*(xb-x)+cosf(th)*(yb-y);
    C[0][3] = 1.0; //cosf(th);
    C[0][4] = 0.0; //sinf(th);
    C[0][5] = 0;
    C[0][6] = 0;

    // kill warnings
    x = 0;
    y = 0;
    th = 0;
    xb = 0;
    yb = 0;

    return C;
}


void PlayerKF::setPlayer(int modelNumber, float x, float y) {

    OrigKalmanFilter* currModel = &(playerModel[modelNumber]);

    NMatrix X = initStates;
    NMatrix P = uncert;

    X[3][0] = x;
    X[4][0] = y;
    X[5][0] = 0;
    X[6][0] = 0;

    P[3][3] = playerPosVar*2.0;
    P[4][4] = playerPosVar*2.0;
    P[5][5] = playerVelVar*2.0;
    P[6][6] = playerVelVar*2.0;

    currModel->Start(numStates, P, X);

}


void PlayerKF::log(int logLevel, const char* format, ...) {
//  if (text_logger_ == NULL)
//    return;
    va_list args;
    char* temp=new char[1024];
    va_start( args, format );
    int l=vsprintf( temp, format, args );

    // kill warnings
    l = l;

    va_end( args );

//  text_logger_->logFromUKF(logLevel, temp);
    delete temp;
}
