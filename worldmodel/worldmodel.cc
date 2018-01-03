#include "worldmodel.h"
#include "../kalman/BallKF.h"
#include "../kalman/PlayerKF.h"

WorldModel::WorldModel() {

    cycle = 0;
    scoreLeft = -1;
    scoreRight = -1;
    time = -1.0;
    gameTime = -1.0;
    playMode = PM_BEFORE_KICK_OFF;
    lastPlayMode = PM_GAME_OVER;
    lastDifferentPlayMode = PM_GAME_OVER;

    uNum = 1;//dummy
    uNumSet = false;

    side = SIDE_LEFT;//dummy
    sideSet = false;

    lastSkills.push_back( SKILL_STAND ) ;//dummy
    lastSkills.push_back( SKILL_STAND ) ;//dummy

    lastOdometryPos = SIM::Point2D(0,0);
    lastOdometryAngDeg = 0;

    for (int i = 0; i < NUM_WORLD_OBJS; ++i) {
        WorldObject& obj = worldObjects[i];
        obj.id = i;
    }

    fLocalized = false;
    fFallen = false;


    // TODO: this is not correct, but anyway localization should update it
    // Is there a better solution for init them?
    myPosition.setVecPosition( 0, 0, 0 );
    myLastPosition.setVecPosition( 0, 0, 0 );
    myAngDegrees = 0;

#ifdef GROUND_TRUTH_SERVER
    myPositionGroundTruth.setVecPosition(0, 0, 0);
    myAngGroundTruth = 0;
    ballGroundTruth.setVecPosition(0, 0, 0);
#endif

    lastBallSightingTime = -100;

    lastBallSeenPosition = vector<VecPosition>(3,VecPosition(0,0,0));
    lastBallSeenTime = vector<double>(3,0);

    lastLineSightingTime = -100;

    localToGlobal = HCTMatrix();
    globalToLocal = HCTMatrix();

    fallenTeammate = vector<bool>(NUM_AGENTS);
    fallenOpponent = vector<bool>(NUM_AGENTS);
    for (int i = 0; i < NUM_AGENTS; i++) {
        fallenTeammate[i] = false;
        fallenOpponent[i] = false;
    }

    rvsend = new RVSender();
    if (rvsend->getSockFD() == -1)
    {
        perror("WorldModel");
        delete rvsend;
        rvsend = NULL;
    }

    fUseGroundTruthDataForLocalization = false;

    opponentTeamName = "";

    confident = false;

    ballKalmanFilter = new BallKF(this);
    opponentKalmanFilters = new PlayerKF(this);

    for (unsigned int i = 0; i < NUM_AGENTS; i++) {
        setTeammateLastHeardTime(i, -1);
    }
}

WorldModel::~WorldModel() {
    if (rvsend != NULL)
        delete rvsend;

    delete ballKalmanFilter;
    delete opponentKalmanFilters;
}

void WorldModel::display() {

    cout << "****************World Model******************************\n";
    std::cout << "Cycle: " << cycle << "\n";
    std::cout << "Score Left: " << scoreLeft << "\n";
    std::cout << "SCore Right: " << scoreRight << "\n";
    std::cout << "Time: " << time << "\n";
    std::cout << "Game Time: " << gameTime << "\n";
    std::cout << "Play Mode: " << playMode << "\n";

    std::cout << "uNumSet: " << uNumSet << ", uNum: " << uNum << "\n";
    std::cout << "sideSet: " << sideSet << ", side: " << side << "\n";

    for (int i = WO_BALL; i < NUM_WORLD_OBJS; ++i) {
        WorldObject& obj = worldObjects[i];
        cout << "World Object: " << WorldObjType2Str[i] << ":\n"
             << obj.pos << '\n'
             << " currently seen: " << ( obj.currentlySeen ? "YES" : "NO" ) << '\n'
             << " cycle last seen: " << obj.cycleLastSeen
             << " time last seen: " << obj.timeLastSeen << endl;
    }

    cout << "*********************************************************\n";
}

void WorldModel::updateGoalPostsAndFlags() {

    VecPosition fieldXPlusYPlus, fieldXPlusYMinus, fieldXMinusYPlus, fieldXMinusYMinus;
    if(side == SIDE_LEFT) {
        // flags
        worldObjects[FLAG_1_R].pos.setVecPosition( HALF_FIELD_X,
                HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_2_R].pos.setVecPosition(  HALF_FIELD_X,
                -HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_1_L].pos.setVecPosition( -HALF_FIELD_X,
                HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_2_L].pos.setVecPosition( -HALF_FIELD_X,
                -HALF_FIELD_Y,
                0 );
        // goalposts
        worldObjects[GOALPOST_1_R].pos.setVecPosition( HALF_FIELD_X,
                HALF_GOAL_Y,
                GOAL_Z);
        worldObjects[GOALPOST_2_R].pos.setVecPosition(  HALF_FIELD_X,
                -HALF_GOAL_Y,
                GOAL_Z);
        worldObjects[GOALPOST_1_L].pos.setVecPosition( -HALF_FIELD_X,
                HALF_GOAL_Y,
                GOAL_Z);
        worldObjects[GOALPOST_2_L].pos.setVecPosition( -HALF_FIELD_X,
                -HALF_GOAL_Y,
                GOAL_Z);

    }
    else { //side == SIDE_RIGHT
        // flags
        worldObjects[FLAG_1_R].pos.setVecPosition( -HALF_FIELD_X,
                -HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_2_R].pos.setVecPosition( -HALF_FIELD_X,
                HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_1_L].pos.setVecPosition(  HALF_FIELD_X,
                -HALF_FIELD_Y,
                0 );
        worldObjects[FLAG_2_L].pos.setVecPosition( HALF_FIELD_X,
                HALF_FIELD_Y,
                0 );
        // goal posts
        worldObjects[GOALPOST_1_R].pos.setVecPosition( -HALF_FIELD_X,
                -HALF_GOAL_Y,
                GOAL_Z );
        worldObjects[GOALPOST_2_R].pos.setVecPosition( -HALF_FIELD_X,
                HALF_GOAL_Y,
                GOAL_Z);
        worldObjects[GOALPOST_1_L].pos.setVecPosition(  HALF_FIELD_X,
                -HALF_GOAL_Y,
                GOAL_Z );
        worldObjects[GOALPOST_2_L].pos.setVecPosition( HALF_FIELD_X,
                HALF_GOAL_Y,
                GOAL_Z );
    }

}

// Updates the l2g and g2l matrices based on 4 points of the field
void WorldModel::
updateMatricesAndMovingObjs( VecPosition& fieldXPlusYPlus,
                             VecPosition& fieldXPlusYMinus,
                             VecPosition& fieldXMinusYPlus,
                             VecPosition& fieldXMinusYMinus ) {

    VecPosition localFieldCentre = (fieldXPlusYPlus + fieldXMinusYMinus) * 0.5;
    VecPosition localFieldXDirection = (fieldXPlusYPlus - fieldXMinusYPlus).normalize();
    VecPosition localFieldYDirection = (fieldXPlusYPlus - fieldXPlusYMinus).normalize();
    VecPosition localFieldZDirection = (localFieldXDirection.crossProduct(localFieldYDirection)).normalize();

    setLocalToGlobal(0, 0, localFieldXDirection.getX());
    setLocalToGlobal(0, 1, localFieldXDirection.getY());
    setLocalToGlobal(0, 2, localFieldXDirection.getZ());
    setLocalToGlobal(0, 3, -(localFieldCentre.dotProduct(localFieldXDirection)));

    setLocalToGlobal(1, 0, localFieldYDirection.getX());
    setLocalToGlobal(1, 1, localFieldYDirection.getY());
    setLocalToGlobal(1, 2, localFieldYDirection.getZ());
    setLocalToGlobal(1, 3, -(localFieldCentre.dotProduct(localFieldYDirection)));

    setLocalToGlobal(2, 0, localFieldZDirection.getX());
    setLocalToGlobal(2, 1, localFieldZDirection.getY());
    setLocalToGlobal(2, 2, localFieldZDirection.getZ());
    setLocalToGlobal(2, 3, -(localFieldCentre.dotProduct(localFieldZDirection)));

    setLocalToGlobal(3, 0, 0);
    setLocalToGlobal(3, 1, 0);
    setLocalToGlobal(3, 2, 0);
    setLocalToGlobal(3, 3, 1.0);


    VecPosition localBodyCentre = VecPosition(0, 0, 0);
    VecPosition localBodyXDirection = VecPosition(1.0, 0, 0);
    VecPosition localBodyYDirection = VecPosition(0, 1.0, 0);
    VecPosition localBodyZDirection = VecPosition(0, 0, 1.0);

    VecPosition bodyCentre = l2g(localBodyCentre);
    VecPosition bodyXDirection = (l2g(localBodyXDirection) - bodyCentre).normalize();
    VecPosition bodyYDirection = (l2g(localBodyYDirection) - bodyCentre).normalize();
    VecPosition bodyZDirection = (l2g(localBodyZDirection) - bodyCentre).normalize();

    setGlobalToLocal(0, 0, bodyXDirection.getX());
    setGlobalToLocal(0, 1, bodyXDirection.getY());
    setGlobalToLocal(0, 2, bodyXDirection.getZ());
    setGlobalToLocal(0, 3, -(bodyCentre.dotProduct(bodyXDirection)));

    setGlobalToLocal(1, 0, bodyYDirection.getX());
    setGlobalToLocal(1, 1, bodyYDirection.getY());
    setGlobalToLocal(1, 2, bodyYDirection.getZ());
    setGlobalToLocal(1, 3, -(bodyCentre.dotProduct(bodyYDirection)));

    setGlobalToLocal(2, 0, bodyZDirection.getX());
    setGlobalToLocal(2, 1, bodyZDirection.getY());
    setGlobalToLocal(2, 2, bodyZDirection.getZ());
    setGlobalToLocal(2, 3, -(bodyCentre.dotProduct(bodyZDirection)));

    setGlobalToLocal(3, 0, 0);
    setGlobalToLocal(3, 1, 0);
    setGlobalToLocal(3, 2, 0);
    setGlobalToLocal(3, 3, 1.0);

    // Set coordinates for moving objects
    for( int i = WO_BALL; i <= WO_OPPONENT_FOOT_R11; ++i ) {

        WorldObject* pObj = getWorldObject(i);

        if( pObj->currentlySeen ) {


            VecPosition objLocalOrigin = pObj->vision.polar.getCartesianFromPolar();
            VecPosition objGlobal = l2g( objLocalOrigin );

            if( pObj->id == WO_BALL ) {

                setBall( objGlobal );

            } else if( WO_OPPONENT1 <= pObj->id && pObj->id <= WO_OPPONENT11 ) {

                setOpponent( pObj->id, objGlobal );

            } else if( WO_TEAMMATE1 <= pObj->id && pObj->id <= WO_TEAMMATE11 ) {

                setTeammate( pObj->id, objGlobal );

            } else if( WO_TEAMMATE_HEAD1 <= pObj->id && pObj->id <= WO_OPPONENT_FOOT_R11 ) {
                setObjectPosition( pObj->id, objGlobal );
            }

        }
    }
}
