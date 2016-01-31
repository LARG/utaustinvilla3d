#include "BallKF.h"
#include <iostream>
//#include <memory/TextLogger.h>


#include <cstdlib> // abs didn't compile

/*
  X[0][0] = posex
  X[1][0] = posey
  X[2][0] = posetheta
  X[3][0] = ballx
  X[4][0] = bally
  X[5][0] = ballxvel
  X[6][0] = ballyvel
*/

BallKF::BallKF(WorldModel* worldModel_) {
    worldModel = worldModel_;
    init();
}

BallKF::~BallKF() {

}


void BallKF::init() {
    prev_time_ = -1;

    initConstants();
    initModel(0);

}


/** Initialize the model for the KF */
void BallKF::initConstants() {

    ballDebug = false; //true;

#ifdef TOOL
    ballDebug = true;
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
    // Initially robot and ball at (almost) the centre of the field,
    //  the robot facing up the field
    initStates[0][0] = -0.001;
    initStates[1][0] = -0.001;
    initStates[2][0] = 0;
    initStates[3][0] = 0.001;
    initStates[4][0] = 0.001;
    initStates[5][0] = 0;
    initStates[6][0] = 0;

    // when to use close ball measurement update (rel x/y)
    // vs the normal distance/bearing one
    useRelBall = true;
    relBallThreshold = 500.0;

    // some decays and variances
    velocityDecay = 0.95;

    // for time update
    positionVar = 400.0;
    angleVar =    500.0;
    ballPosVar = 300.0;
    ballVelVar = 3000.0;
    kickVelVar = 3000.0;

    // multiple ball uncertainty by this if its unseen
    unseenBallVarFactor = 6.0;// OPEN
    minBallDistance = 40.0;
    pfVarianceFactor = 1000000.0;

    // for measurement update
    sdDist = 5.0; // distance measurement variance // OPEN
//  sdBearing = 5.0; (Not Used) // bearing measurement variance

}


/** Init the kf with the appropriate initial X and P. */
void BallKF::initModel(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // start it up!
    currModel->Start(numStates, uncert, initStates);

    if (ballDebug) {
        log(50, "Ball Init Model");
        logBallMatrix(50);
    }


}

void BallKF::processFrame() {
    double timePassed = worldModel->getTime() - prev_time_;
    if (prev_time_ < 0) {
        // Case for initialization
        timePassed = 1;
    }
    prev_time_ = worldModel->getTime();

    // ball - time update
    //cout << "Time passed: " << timePassed << endl;
    timeUpdate(0, timePassed);

    // update the pose in the kf using pf estimate
    poseMeasurementUpdate(0);

    // ball - measurement update
    ballMeasurementUpdate(0);

    // possibly clip position
    //ball.clipPosition(0);

    // set ball kf estimate into ball world object
    updateBallFromKF(0);
}

/** Perform the time update on the KalmanFilter model[modelNumber]
    - knowing that timePassed seconds have passed. */
void BallKF::timeUpdate(int modelNumber, float timePassed) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

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

    // update ball position by negative of odometry
    SIM::Point2D ballPos = SIM::Point2D(X[3][0], X[4][0]);
    ballPos = ballPos - odom;
    // rotate by odom turn
    ballPos.rotate(-odomTurn);

    // ball position is updated by velocity
    Xbar[3][0] = ballPos.x + X[5][0] * timePassed;
    Xbar[4][0] = ballPos.y + X[6][0] * timePassed;

    // ball velocity is decayed
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

    Q[3][3] = ballPosVar;
    Q[4][4] = ballPosVar;
    Q[5][5] = ballVelVar;
    Q[6][6] = ballVelVar;

    // hack: add these to ball instead
    Q[3][3] += 0.1*(odom.getMagnitude() * odom.getMagnitude());
    Q[4][4] += 0.1*(odom.getMagnitude() * odom.getMagnitude());
    Q[3][3] += 0.1*(odomTurn*odomTurn);
    Q[4][4] += 0.1*(odomTurn*odomTurn);

    //Q[0][0] += 0.1*(odomLeft*odomLeft + odomForward*odomForward);
    //Q[1][1] += 0.1*(odomLeft*odomLeft + odomForward*odomForward);
    //Q[2][2] += 0.1*(odomTurn*odomTurn);

    // TODO: ricks q value fiddle?

    // increase ball pos uncertainty if we haven't seen it?
    if (!(worldModel->getWorldObject(WO_BALL)->currentlySeen)) {
        Q[3][3] *= unseenBallVarFactor;
        Q[4][4] *= unseenBallVarFactor;
    }

    // perform the actual update
    currModel->TimeUpdateExtended(A, Xbar, Q);
    currModel->NormaliseState(2);

    if (ballDebug) {
        log(50, "Ball Time Update");
        logBallMatrix(50);
    }


}


/** Perform a measurement update on the robot pose based
    on the current estimtae from the particle filter. */
void BallKF::poseMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

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

    if (ballDebug) {
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

    if (ballDebug) {
        log(60, "Ball Pose Measurement Update");
        logBallMatrix(60);
    }


}


/** Perform the measurement update on the KalmanFilter &(ballModel[modelNumber])
    using the worldObjects info for ball/self info. */
void BallKF::ballMeasurementUpdate(int modelNumber) {


    // decide if ball is too close (and we want to use rel x/y for update)
    //   or far enough for normal update (dist/bearing)
    WorldObject* ball = worldModel->getWorldObject(WO_BALL);

    // no update here if we didn't see the ball
    if (!ball->currentlySeen)
        return;

    // Todd: for some reason normal update is going crazy
    // use relative x,y ball measurement update only!
    closeBallMeasurementUpdate(modelNumber);

    //cout << "Ball Measurement Update: "
    //   <<  (currModel->GetStates())[3][0] << ", "
    //   <<  (currModel->GetStates())[4][0] << endl << flush;

    if (ballDebug) {
        log(50, "Ball Measurement Update");
        logBallMatrix(50);
    }
}



/** Perform normal ball measurement update based on distance and bearing. */
void BallKF::normalBallMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // measurement jacobians
    NMatrix Cdist = NMatrix(1, 7, false);
    NMatrix Cbearing = NMatrix(1, 7, false);

    // object pointers
    WorldObject* ball = worldModel->getWorldObject(WO_BALL);
    //WorldObject* self = &(current->commonMem->worldObjects[current->commonMem->WO_SELF]);

    // get estimate of global ball location from our pose
    // and distance/bearing to ball
    double visionDistance = ball->vision.polar.getX();
    double visionBearing = Deg2Rad( ball->vision.polar.getY() );
    SIM::Point2D relLoc = SIM::Point2D(visionDistance, visionBearing, POLAR);

    // hack: global loc is rel loc
    SIM::Point2D globalLoc = relLoc;
    //SIM::Point2D globalLoc = relLoc.relativeToGlobal(self->loc, self->orientation);

    // estimate of variance of ball distance
    //  from michael's empirical estimate
    // in cm
    float Rdist = getBallDistanceVariance(visionDistance);

    SIM::Point2D estBall = SIM::Point2D(X[3][0], X[4][0]);
    float estDist = estBall.getMagnitude();
    float estBearing = estBall.getDirection();

    //float estDist = sqrtf(square(X[3][0] - X[0][0]) + square(X[4][0] - X[1][0]));
    if (estDist < minBallDistance) estDist = minBallDistance;

    // get measurement jacobian for distance
    Cdist = GetBallDistanceJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                    X[5][0], X[6][0]);

    // TODO: did we get a circle fit?
    bool circleFit = true;

    // estimate variance of ball bearing
    float Rbearing = 0.02; // default sd of 18 deg
    if (visionDistance < 0) Rbearing = 0.04; // sd of 36 deg
    else if (visionDistance > 1000) Rbearing = 0.001; // sd of 2 deg
    else if (visionDistance > 300 && circleFit) Rbearing = 0.004; // sd of 4 deg
    else if (visionDistance > 300 && !circleFit) Rbearing = 0.02; // sd of 8 deg



    // get measurement jacobian for bearing
    Cbearing = GetBallBearingJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                      X[5][0], X[6][0]);

    //float estBearing = normalizeAngle(atan2(X[4][0] - X[1][0],
    //				   X[3][0] - X[0][0]) - X[2][0]);


    if (ballDebug) {
        log(50, "Ball seen at distance %5.0f, expected %5.0f, bearing %5.0f, expected %5.0f, globalLoc: (%5.0f, %5.0f)",
            visionDistance, estDist, visionBearing, estBearing,
            globalLoc.x, globalLoc.y);
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


/** Perform close ball measurement update based on relative x/y to ball. */
void BallKF::closeBallMeasurementUpdate(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // object pointers
    WorldObject* ball = worldModel->getWorldObject(WO_BALL);

    // X
    // relative x estimate
    NMatrix Cxrel = GetBallXRelJacobian(X[0][0], X[1][0], X[2][0],
                                        X[3][0], X[4][0]);


    double visionDistance = ball->vision.polar.getX();
    double visionBearing = Deg2Rad( ball->vision.polar.getY() );


    // estimate of variance of ball x distance
    // in cm
    float Rxrel = getBallDistanceVariance(visionDistance);

    // actual value of xrel
    float xrel = -abs(visionDistance) * sinf(visionBearing);

    // projected value for xr
    float xrelbar = - (X[4][0]);


    // Y
    // relative y estimate
    NMatrix Cyrel = GetBallYRelJacobian(X[0][0], X[1][0], X[2][0],
                                        X[3][0], X[4][0]);

    // estimate of variance of ball y distance
    // in cm
    float Ryrel = getBallDistanceVariance(visionDistance);

    // actual value of yrel
    float yrel = abs(visionDistance) * cosf(visionBearing);

    // projected value for yr
    float yrelbar = (X[3][0]);

    if (ballDebug) {
        log(50, "Ball seen at distance %5.0f, bearing %5.0f, relX: %5.0f, relxbar %5.0f, relY %5.0f, relYbar %5.0f",
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


/** Update the ball after a kick with the given velocity and Bearing. */
void BallKF::kickUpdate(int modelNumber, float vel, SIM::AngRad heading) {

    if (ballDebug) {
        log(50, "Kick Update Pre");
        logBallMatrix(50);
    }

    // turn this distance/bearing into x,y
    SIM::Point2D ballVel = SIM::Point2D(vel, heading, POLAR);

    // update ball velocity with this
    int updateSuccess;
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // Measurement update
    NMatrix Cx = NMatrix(1,7,false);
    Cx[0][5] = 1.0;
    updateSuccess = currModel->MeasurementUpdate(Cx, kickVelVar, ballVel.x,
                    false, 0, false);

    if (updateSuccess == KF_CRASH) return;
    else currModel->NormaliseState(2);

    NMatrix Cy = NMatrix(1,7,false);
    Cy[0][6] = 1.0;
    updateSuccess = currModel->MeasurementUpdate(Cy, kickVelVar, ballVel.y,
                    false, 0, false);

    if (updateSuccess == KF_CRASH) return;
    else currModel->NormaliseState(2);

    if (ballDebug) {
        log(50, "Updated ball with vel %5.0f, heading %5.0f",
            vel, heading);
        logBallMatrix(50);
    }

    //cout << "Updating ball with vel " << vel << " at heading "
    //   << heading << endl << flush;

}


/** Update the ball when we're grabbing it. */
void BallKF::grabUpdate(int modelNumber) {

    // update ball velocity with this
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // get current estimate
    NMatrix X = currModel->GetStates();

    // measurement jacobians
    NMatrix Cdist = NMatrix(1, 7, false);
    NMatrix Cbearing = NMatrix(1, 7, false);

    // object pointers
    //  WorldObject* ball = &(current->commonMem->worldObjects[WO_BALL]);
    //WorldObject* self = &(current->commonMem->worldObjects[current->commonMem->WO_SELF]);

    // get estimate of global ball location from our pose
    // and distance/bearing to ball
    SIM::Point2D relLoc = SIM::Point2D(200, 0);

    // hack: global loc is rel loc
    SIM::Point2D globalLoc = relLoc;
    //SIM::Point2D globalLoc = relLoc.relativeToGlobal(self->loc, self->orientation);


    // estimate of variance of ball distance
    //  from michael's empirical estimate
    // in cm
    float Rdist = getBallDistanceVariance(200);

    float estDist = sqrtf(SIM::square(X[3][0] - X[0][0]) + SIM::square(X[4][0] - X[1][0]));
    if (estDist < minBallDistance) estDist = minBallDistance;

    // get measurement jacobian for distance
    Cdist = GetBallDistanceJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                    X[5][0], X[6][0]);

    // estimate variance of ball bearing
    float Rbearing = 0.004; // sd of 4 deg

    // get measurement jacobian for bearing
    Cbearing = GetBallBearingJacobian(X[3][0], X[4][0], X[0][0], X[1][0],
                                      X[5][0], X[6][0]);

    float estBearing = SIM::normalizeAngle(atan2(X[4][0] - X[1][0],
                                           X[3][0] - X[0][0]) - X[2][0]);

    if (ballDebug) {
        log(50, "Ball grab at rel: (%5.0f, %5.0f), global (%5.0f, %5.0f)",
            relLoc.x, relLoc.y, globalLoc.x, globalLoc.y);
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


/** Set the ball world object with the new estimates from the kf. */
void BallKF::updateBallFromKF(int modelNumber) {
    if (ballDebug) log(50, "Updating Ball v2");
    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // pointer to ball object
    WorldObject* ball = worldModel->getWorldObject(WO_BALL);

    // info about robot's pose
//  WorldObject* self = &(world_objects_->objects_[robot_state_->WO_SELF]);

    // get estimate and covariance matrix from filter
    NMatrix X = currModel->GetStates();
    NMatrix P = currModel->GetErrorMatrix();

    // get ball rel loc and sd
    SIM::Point2D ballRelLoc = SIM::Point2D(X[3][0], X[4][0]);
//  SIM::Point2D ballRelSD = SIM::Point2D(sqrtf(P[3][3]), sqrtf(P[4][4]));

    // changed by Sam Barrett (6/13/11)
    ball->vision.polar.setX(ballRelLoc.getMagnitude());
    ball->vision.polar.setY(Rad2Deg(ballRelLoc.getDirection()));
//  ball->relPos = ballRelLoc;
//  ball->relOrientation = ballRelLoc.getDirection();
//cerr << "WARNING: USING KF VELOCITY - MIGHT BE WRONG?" << endl;
//  // get global loc and sd from this
//  ball->loc = ballRelLoc.relativeToGlobal(self->loc, self->orientation);
//  // add in our sd
//  ball->sd =  ballRelSD.rotate(self->orientation);
//  // make sure this is still positive after rotation
//  ball->sd.x = fabs(ball->sd.x);
//  ball->sd.y = fabs(ball->sd.y);
//  ball->sd = ball->sd + self->sd;
//
//  // get rel velocity of ball from KF
//  ball->relVel = SIM::Point2D(X[5][0], X[6][0]);

    // get ball's absolute velocity
    ball->absVel = (SIM::Point2D(X[5][0], X[6][0])).rotate(worldModel->getMyAngRad());

//  // get distance and bearing to ball
//  ball->distance = ballRelLoc.getMagnitude();
//  ball->bearing = ballRelLoc.getDirection();

    //cout << "Filtered rel ball: " << ballRelLoc << " d: " << ball->distance
    // << " b: " << ball->bearing << " v: " << ball->relVel << endl;
    //cout << "Convert to abs " << ball->loc << " v: " << ball->absVel << endl;

    /* hack: set these different since ball is now relative, not global

    // set location and sd of ball
    ball->loc = SIM::Point2D(X[3][0], X[4][0]);
    ball->sd = SIM::Point2D(sqrtf(P[3][3]), sqrtf(P[4][4]));

    // set absolute velocity of ball
    ball->absVel = SIM::Point2D(X[5][0], X[6][0]);

    // set relative distance/bearing to ball
    ball->distance = myloc.getDistanceTo(ball->loc);
    ball->bearing = myloc.getBearingTo(ball->loc, myorient);

    // set relative velocity of ball
    ball->relVel = ball->loc.globalToRelative( myloc, myorient );

    */
}


/** Get the kf's estimate of the robot location
    (for comparison with particle filter) */
SIM::Point2D BallKF::getRobotPosition(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    // get estimate and covariance matrix from filter
    NMatrix X = currModel->GetStates();

    return SIM::Point2D(X[0][0], X[1][0]);
}


/** Clip the ball position if its off the field. */
void BallKF::clipPosition(int modelNumber) {

    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

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
void BallKF::resetFilter(int modelNumber) {

    // get the model we're updating
    OrigKalmanFilter* currModel = &(ballModel[modelNumber]);

    currModel->Reset();

}



/** Returns estimate of ball distance variance */
float BallKF::getBallDistanceVariance(float ballDistance) {

    // convert distance from mm to cm
    ballDistance /= 10.0;

    // Michael's empirical results
    // sd - do in mm
    float Rdist = log10(ballDistance) * ballDistance / 10.0;
    Rdist = SIM::crop(Rdist, 3, 200);

    // increase dist variance if its negative
    if (ballDistance < 0) {
        Rdist *= 4;
    }
    Rdist = SIM::square(Rdist);
    if (Rdist < 0.0001) Rdist = 0.0001;

    // convert from cm^2 to mm^2
    Rdist = Rdist * 100.0;
    return Rdist;

}


void BallKF::logBallMatrix(int loglevel) {

    OrigKalmanFilter* currModel = &(ballModel[0]);

    NMatrix X = currModel->GetStates();

    log(loglevel, "Ball Matrix: X: Pos (%5.0f, %5.0f) Vel(%5.0f, %5.0f)",
        X[3][0], X[4][0], X[5][0], X[6][0]);

    NMatrix P = currModel->GetErrorMatrix();

    log(loglevel, "Ball Matrix: P: Pos (%5.0f, %5.0f) Vel(%5.0f, %5.0f)",
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

NMatrix BallKF::GetXPositionJacobian() {

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

NMatrix BallKF::GetYPositionJacobian() {

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

NMatrix BallKF::GetThetaPositionJacobian() {

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

NMatrix BallKF::GetBallDistanceJacobian(float xb, float yb, float x, float y,
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

NMatrix BallKF::GetBallBearingJacobian(float xb, float yb, float x, float y,
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
NMatrix BallKF::GetBallXRelJacobian(float x, float y, float th, float xb, float yb ) {
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

NMatrix BallKF::GetBallYRelJacobian(float x, float y, float th, float xb, float yb ) {
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


void BallKF::setBall(float x, float y) {

    OrigKalmanFilter* currModel = &(ballModel[0]);

    NMatrix X = initStates;
    NMatrix P = uncert;

    X[3][0] = x;
    X[4][0] = y;
    X[5][0] = 0;
    X[6][0] = 0;

    P[3][3] = ballPosVar*2.0;
    P[4][4] = ballPosVar*2.0;
    P[5][5] = ballVelVar*2.0;
    P[6][6] = ballVelVar*2.0;

    currModel->Start(numStates, P, X);

}


void BallKF::log(int logLevel, const char* format, ...) {
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
