#include <cmath>

#include "parser.h"
#include "../worldmodel/WorldObject.h"
#include "../audio/audio.h"
#include "../headers/headers.h"
#include "../worldmodel/worldmodel.h"
#include "../bodymodel/bodymodel.h"
#include "VisionObject.h"
#include "../particlefilter/PFLocalization.h"
#include "../kalman/BallKF.h"
#include "../kalman/PlayerKF.h"

// For UT Walk
#include <memory/Memory.h>
#include <memory/FrameInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SimEffectorBlock.h>
#include <sys/time.h>

using boost::shared_ptr;

// Already declared in Geometry.h
//#define RAD_T_DEG  (180.0/ M_PI)
//#define DEG_T_RAD  (M_PI/180.0)

///////////////////////////
//Take care of side both halves - constructor wrong currently.
///////////////////////////

Parser::Parser(WorldModel *worldModel, BodyModel *bodyModel, const string &teamName,
               PFLocalization* particleFilter, FrameInfoBlock* vision_frame_info, FrameInfoBlock* frame_info, JointBlock* joint_block, SensorBlock* sensor_block) {
    sensor_block_= sensor_block;
    joint_angles_block_ = joint_block;
    frame_info_ = frame_info;
    vision_frame_info_ =vision_frame_info;

    this->worldModel = worldModel;
    this->bodyModel = bodyModel;
    this->particleFilter = particleFilter;

    //Both side and UNum not necessarily correct, but stop gap initially.
    this->side = SIDE_LEFT;
    this->uNum = 1;

    this->teamName = teamName;
    this->fProcessedVision = false;
}

Parser::~Parser() {

}

vector<string> Parser::tokenise(const string &s) {

    int length = s.length();
    string currentString = "";
    bool currentValid = false;

    vector<string> v;

    for(int i = 0; i < length; ++i) {

        char c = s.at(i);
        if(c != '(' && c != ')' && c != ' ') {
            currentString.append(1, c);
        }
        else {

            if(currentString.length() > 0) {
                v.push_back(currentString);
                currentString = "";
            }
        }
    }

    return v;
}

vector<string> Parser::tokeniseCommaDelim(const string &s) {

    int length = s.length();
    string currentString = "";
    bool currentValid = false;

    vector<string> v;

    for(int i = 0; i < length; ++i) {

        char c = s.at(i);
        if(c != '(' && c != ')' && c != ',') {
            currentString.append(1, c);
        }
        else {

            if(currentString.length() > 0) {
                v.push_back(currentString);
                currentString = "";
            }
        }
    }

    return v;
}

bool Parser::parseTime(const string &str) {

    bool valid = false;
    double time = 0;
    this->fProcessedVision = false;
    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size() - 1; ++i) {
        if(!(tokens[i].compare("now"))) {
            time = atof(tokens[i + 1].c_str());
            valid = true;
        }
    }

    if(valid) {
        worldModel->setTime(time);
        worldModel->incrementCycle();
        frame_info_->seconds_since_start = time;
    }

    return valid;
}

bool Parser::parseGameState(const string &str) {

    bool gameTimeValid = false;
    bool playModeValid = false;
    bool uNumValid = false;
    bool sideValid = false;
    bool scoreLeftValid = false;
    bool scoreRightValid = false;

    double gameTime = 0;
    string playModeStr = "";

    int playMode = -1;
    int scoreLeft = -1;
    int scoreRight = -1;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size() - 1; ++i) {

        if(!(tokens[i].compare("t"))) {
            gameTime = atof(tokens[i + 1].c_str());
            gameTimeValid = true;
            ++i;
        }
        else if(!(tokens[i].compare("pm"))) {

            playModeStr = tokens[i + 1];


            if(!(playModeStr.compare("BeforeKickOff"))) {
                playMode = PM_BEFORE_KICK_OFF;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("KickOff_Left"))) {
                playMode = PM_KICK_OFF_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("KickOff_Right"))) {
                playMode = PM_KICK_OFF_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("PlayOn"))) {
                playMode = PM_PLAY_ON;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("KickIn_Left"))) {
                playMode = PM_KICK_IN_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("KickIn_Right"))) {
                playMode = PM_KICK_IN_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("Goal_Left"))) {
                playMode = PM_GOAL_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("Goal_Right"))) {
                playMode = PM_GOAL_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("GameOver"))) {
                playMode = PM_GAME_OVER;
                playModeValid = true;
            }
            //Extra
            else if(!(playModeStr.compare("corner_kick_left"))) {
                playMode = PM_CORNER_KICK_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("corner_kick_right"))) {
                playMode = PM_CORNER_KICK_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("goal_kick_left"))) {
                playMode = PM_GOAL_KICK_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("goal_kick_right"))) {
                playMode = PM_GOAL_KICK_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("offside_left"))) {
                playMode = PM_OFFSIDE_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("offside_right"))) {
                playMode = PM_OFFSIDE_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("free_kick_left"))) {
                playMode = PM_FREE_KICK_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("free_kick_right"))) {
                playMode = PM_FREE_KICK_RIGHT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("direct_free_kick_left"))) {
                playMode = PM_DIRECT_FREE_KICK_LEFT;
                playModeValid = true;
            }
            else if(!(playModeStr.compare("direct_free_kick_right"))) {
                playMode = PM_DIRECT_FREE_KICK_RIGHT;
                playModeValid = true;
            }
            else {
                playModeValid = false;
                cout << "Unknown play mode: " << playModeStr << "\n";
            }
            ++i;
        }
        else if(!(tokens[i].compare("unum"))) {

            uNum = atoi(tokens[i + 1].c_str());
            if(1 <= uNum && uNum <= 11) {
                uNumValid = true;
            }
            ++i;
        }
        else if(!(tokens[i].compare("team"))) {

            if(!(tokens[i + 1].compare("left"))) {
                side = SIDE_LEFT;
                sideValid = true;
            }
            else if(!(tokens[i + 1].compare("right"))) {
                side = SIDE_RIGHT;
                sideValid = true;
            }
            ++i;
        }
        else if(!(tokens[i].compare("sl"))) {
            scoreLeft = atoi(tokens[i + 1].c_str());
            scoreLeftValid = true;
            ++i;
        }
        else if(!(tokens[i].compare("sr"))) {
            scoreRight = atoi(tokens[i + 1].c_str());
            scoreRightValid = true;
            ++i;
        }

    }

    if(gameTimeValid) {
        worldModel->setGameTime(gameTime);
    }

    if(playModeValid) {
        worldModel->setPlayMode(playMode);
    }

    if(uNumValid) {
        worldModel->setUNum(uNum);
        worldModel->getRVSender()->setUNum(uNum); //for drawing
        worldModel->setUNumSet(true);
    }

    if(sideValid) {
        worldModel->setSide(side);
        worldModel->getRVSender()->setSide(side); //for drawing
        worldModel->setSideSet(true);
    }

    if (scoreLeftValid) {
        worldModel->setScoreLeft(scoreLeft);
    }

    if (scoreRightValid) {
        worldModel->setScoreRight(scoreRight);
    }

    bool valid = gameTimeValid || playModeValid;

    return valid;
}

bool Parser::parseGyro(const string &str) {
    bool valid = false;

    double rateX = 0.0, rateY = 0.0, rateZ = 0.0;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size(); ++i) {
        if(!tokens[i].compare("rt")) {
            if(i + 3 < tokens.size()) {

                rateX = atof(tokens[i + 1].c_str());
                rateY = atof(tokens[i + 2].c_str());
                rateZ = atof(tokens[i + 3].c_str());
                valid = true;
            }
        }

    }

    if(valid) {
        bodyModel->setGyroRates(rateX, rateY, rateZ);

        sensor_block_->values_[gyroX] = rateY;
        sensor_block_->values_[gyroY] = rateX;
        sensor_block_->values_[gyroZ] = rateZ;
    }

    return valid;
}

bool Parser::parseAccelerometer(const string &str) {
    bool valid = false;

    double rateX = 0.0, rateY = 0.0, rateZ = 0.0;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size(); ++i) {
        if(!tokens[i].compare("a")) {
            if(i + 3 < tokens.size()) {

                rateX = atof(tokens[i + 1].c_str());
                rateY = atof(tokens[i + 2].c_str());
                rateZ = atof(tokens[i + 3].c_str());
                valid = true;
            }
        }

    }

    if(valid) {

        // Sometimes spurious (very high or NaN) readings come through. Clip them.
        double spuriousThreshold = 20.0;
        if(rateX != rateX || (fabs(rateX) > spuriousThreshold)) {
            rateX = 0;
        }
        if(rateY != rateY || (fabs(rateY) > spuriousThreshold)) {
            rateY = 0;
        }
        if(rateZ != rateZ || (fabs(rateZ) > spuriousThreshold)) {
            rateZ = 0;
        }

        // Make signs compatible with agent's local axes.
        double correctedRateX = -rateY;
        double correctedRateY = rateX;
        double correctedRateZ = -rateZ;


        // Apply filter to raw accelerometer data
        float K = 0.9;
        VecPosition lastAccel = bodyModel->getAccelRates();
        correctedRateX = K*lastAccel.getX() + (1-K)*correctedRateX;
        correctedRateY = K*lastAccel.getY() + (1-K)*correctedRateY;
        correctedRateZ = K*lastAccel.getZ() + (1-K)*correctedRateZ;


        bodyModel->setAccelRates(correctedRateX, correctedRateY, correctedRateZ);
        sensor_block_->values_[accelY] = rateX;
        sensor_block_->values_[accelX] = rateY;
        sensor_block_->values_[accelZ] = rateZ;
    }

    return valid;
}

//to handle -- do when needed.
bool Parser::parseHear(const string &str) {
    bool valid = false;

    double hearTime;
    bool self;
    double angle;
    string message;

    vector<string> tokens = tokenise(str);
    valid = tokens.size() == 5;

    if (!valid) {
        return valid;
    }

    unsigned int i = 1;

    if (tokens.size() == 5) {
        string team = tokens[i++];
        if (team.compare(teamName)) {
            worldModel->setOpponentTeamName(team);
            return true;
        }
    }

    hearTime = atof(tokens[i++].c_str());
    if(!(tokens[i].compare("self"))) {
        self = true;
        angle = 0;
    }
    else {
        self = false;
        angle = atof(tokens[i].c_str());
    }

    i++;

    if (i >= tokens.size()) {
        // Make sure we don't get tripped up by an empty message
        return false;
    }

    message = tokens[i];

    double timeBallLastSeen;
    bool fallen;
    double ballX;
    double ballY;
    double playerX;
    double playerY;
    int playerNum;
    double time;

    double deltaServerToGameTime = worldModel->getTime()-worldModel->getGameTime();

    if (processHearMessage(message, hearTime+deltaServerToGameTime, playerNum, timeBallLastSeen, ballX, ballY, playerX, playerY, fallen, time)) {
        worldModel->setFallenTeammate(playerNum-1, fallen);

        if (!self && !fallen) {
            static VecPosition lastHeardBallPos = VecPosition(0,0,0);
            VecPosition ballPos = VecPosition(ballX, ballY, 0);
            WorldObject *pBall = worldModel->getWorldObject(WO_BALL);
            if (timeBallLastSeen > worldModel->getLastBallSightingTime()) {
                worldModel->setLastBallSightingTime(timeBallLastSeen);
                pBall->haveSighting = true;
                pBall->sighting = ballPos;
                pBall->sightingTime = timeBallLastSeen;
                pBall->sightingUNum = playerNum;
            }

            lastHeardBallPos = ballPos;

            WorldObject *pTeammate = worldModel->getWorldObject(WO_TEAMMATE1+playerNum-1);
            VecPosition teammatePos = VecPosition(playerX, playerY, 0);
            //cout << "Player - " << worldModel->getUNum() << ": Heard player " << playerNum << " at " << teammatePos << endl;;
            pTeammate->haveSighting = true;
            pTeammate->sighting = teammatePos;
            pTeammate->sightingTime = time;
            pTeammate->sightingUNum = playerNum;
        }
        /*
        cout << "Data Received:\n";
        cout << "\tUNum: " << playerNum << "\n";
        cout << "\tBall last seen time: " << timeBallLastSeen << "\n";
        cout << "\tFallen: " << fallen << "\n";
        cout << "\tBall x: " << ballX << "\n";
        cout << "\tBall y: " << ballY << "\n";
        cout << "\tPlayer x: " << playerX << "\n";
        cout << "\tPlayer y: " << playerY << "\n";
        cout << "\tTime: " << time << "\n";
        */
    }

    return valid;
}

bool Parser::parseHingeJoint(const string &str) {

    bool valid;

    string name;
    double angle = 0.0;

    bool validName = false;
    bool validAngle = false;

    int hingeJointIndexBM = -1;
    int hingeJointIndexWE = -1;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size(); ++i) {

        if(!tokens[i].compare("n")) {
            if(i + 1 < tokens.size()) {

                name = tokens[i + 1];

                if(!(name.compare("hj1"))) {
                    hingeJointIndexBM = HJ_H1;
                    hingeJointIndexWE = HeadYaw; // HJ_H1;
                    validName = true;
                }
                else if(!(name.compare("hj2"))) {
                    hingeJointIndexBM = HJ_H2;
                    hingeJointIndexWE = HeadPitch; //HJ_H2;
                    validName = true;
                }
                else if(!(name.compare("laj1"))) {
                    hingeJointIndexBM = HJ_LA1;
                    hingeJointIndexWE = LShoulderPitch; ///HJ_LA1;
                    validName = true;
                }
                else if(!(name.compare("laj2"))) {
                    hingeJointIndexBM = HJ_LA2;
                    hingeJointIndexWE = LShoulderRoll; //HJ_LA2;
                    validName = true;
                }
                else if(!(name.compare("laj3"))) {
                    hingeJointIndexBM = HJ_LA3;
                    hingeJointIndexWE = LElbowYaw; //HJ_LA3;
                    validName = true;
                }
                else if(!(name.compare("laj4"))) {
                    hingeJointIndexBM = HJ_LA4;
                    hingeJointIndexWE = LElbowRoll; //HJ_LA4;
                    validName = true;
                }
                else if(!(name.compare("raj1"))) {
                    hingeJointIndexBM = HJ_RA1;
                    hingeJointIndexWE = RShoulderPitch; //HJ_RA1;
                    validName = true;
                }
                else if(!(name.compare("raj2"))) {
                    hingeJointIndexBM = HJ_RA2;
                    hingeJointIndexWE = RShoulderRoll; //HJ_RA2;
                    validName = true;
                }
                else if(!(name.compare("raj3"))) {
                    hingeJointIndexBM = HJ_RA3;
                    hingeJointIndexWE = RElbowYaw; //HJ_RA3;
                    validName = true;
                }
                else if(!(name.compare("raj4"))) {
                    hingeJointIndexBM = HJ_RA4;
                    hingeJointIndexWE = RElbowRoll; //HJ_RA4;
                    validName = true;
                }
                else if(!(name.compare("llj1"))) {
                    hingeJointIndexBM = HJ_LL1;
                    hingeJointIndexWE = LHipYawPitch; //HJ_LL1;
                    validName = true;
                }
                else if(!(name.compare("llj2"))) {
                    hingeJointIndexBM = HJ_LL2;
                    hingeJointIndexWE = LHipRoll; //HJ_LL2;
                    validName = true;
                }
                else if(!(name.compare("llj3"))) {
                    hingeJointIndexBM = HJ_LL3;
                    hingeJointIndexWE = LHipPitch; //HJ_LL3;
                    validName = true;
                }
                else if(!(name.compare("llj4"))) {
                    hingeJointIndexBM = HJ_LL4;
                    hingeJointIndexWE = LKneePitch; //HJ_LL4;
                    validName = true;
                }
                else if(!(name.compare("llj5"))) {
                    hingeJointIndexBM = HJ_LL5;
                    hingeJointIndexWE = LAnklePitch; //HJ_LL5;
                    validName = true;
                }
                else if(!(name.compare("llj6"))) {
                    hingeJointIndexBM = HJ_LL6;
                    hingeJointIndexWE = LAnkleRoll; //HJ_LL6;
                    validName = true;
                }
                else if(!(name.compare("llj7"))) {
                    hingeJointIndexBM = HJ_LL7;
                    hingeJointIndexWE = LToePitch; //HJ_LL7;
                    validName = true;
                }
                else if(!(name.compare("rlj1"))) {
                    hingeJointIndexBM = HJ_RL1;
                    hingeJointIndexWE = RHipYawPitch; //HJ_RL1;
                    validName = true;
                }
                else if(!(name.compare("rlj2"))) {
                    hingeJointIndexBM = HJ_RL2;
                    hingeJointIndexWE = RHipRoll; //HJ_RL2;
                    validName = true;
                }
                else if(!(name.compare("rlj3"))) {
                    hingeJointIndexBM = HJ_RL3;
                    hingeJointIndexWE = RHipPitch; //HJ_RL3;
                    validName = true;
                }
                else if(!(name.compare("rlj4"))) {
                    hingeJointIndexBM = HJ_RL4;
                    hingeJointIndexWE = RKneePitch; //HJ_RL4;
                    validName = true;
                }
                else if(!(name.compare("rlj5"))) {
                    hingeJointIndexBM = HJ_RL5;
                    hingeJointIndexWE = RAnklePitch; //HJ_RL5;
                    validName = true;
                }
                else if(!(name.compare("rlj6"))) {
                    hingeJointIndexBM = HJ_RL6;
                    hingeJointIndexWE = RAnkleRoll; //HJ_RL6;
                    validName = true;
                }
                else if(!(name.compare("rlj7"))) {
                    hingeJointIndexBM = HJ_RL7;
                    hingeJointIndexWE = RToePitch; //HJ_RL7;
                    validName = true;
                }
            }
        }

        if(!tokens[i].compare("ax")) {
            if(i + 1 < tokens.size()) {

                angle = atof(tokens[i + 1].c_str());
                validAngle = true;
            }
        }

    }

    valid = validName && validAngle;

    if(valid) {
        //    cout << "HJ: " << name << " " << angle << "\n";
        bodyModel->setJointAngle(hingeJointIndexBM, angle);
        joint_angles_block_->values_[hingeJointIndexWE]=DEG_T_RAD*angle;
    }
    //  else{
    //    cout << str << " not valid.\n";
    //  }

    return valid;
}

bool Parser::parseSee(const string &str) {

    bool valid = true;
    visionObjs.clear();

    vector<string> strSegments = segment(str, true);

    for(size_t i = 0; i < strSegments.size(); ++i) {

        //Goalpost
        if(strSegments[i].at(1) == 'G') {
            valid = parseGoalPost(strSegments[i], visionObjs) && valid;
        }
        //Flags
        else if(strSegments[i].at(1) == 'F') {
            valid = parseFlag(strSegments[i], visionObjs) && valid;
        }
        //Ball
        else if(strSegments[i].at(1) == 'B') {
            valid = parseBall(strSegments[i], visionObjs) && valid;
        }
        //Player
        else if(strSegments[i].at(1)== 'P') {
            valid = parsePlayer(strSegments[i], visionObjs) && valid;
        }
#ifdef GROUND_TRUTH_SERVER
        // MyPos and MyOrien
        else if(strSegments[i].at(1)== 'm') {
            valid = parseMyPos(strSegments[i]) && valid;
        }
        // GroundTruth Ball Position
        else if(strSegments[i].at(1)== 'b') {
            valid = parseBallPos(strSegments[i]) && valid;
        }
#endif
        //See token (ignore)
        else if (strSegments[i].at(1)== 'S') {
        }
        // Line
        else if(strSegments[i].at(1) == 'L' ) {
            worldModel->setLastLineSightingTime(worldModel->getTime());
            valid = parseLine(strSegments[i], visionObjs) && valid;
        }
        else {
            valid = false;
        }

    }

    if(!valid) {
        cout << str << " not valid.\n";
    }
    return valid;
}

bool Parser::parseLine(const string &str, vector< boost::shared_ptr<VisionObject> >& visionObjs) {

    bool valid = false;

    string name;
    double r, theta, phi, r2, theta2, phi2;

    int lineIndex = -1;

    //Check for nan values
    if (str.find("nan") != string::npos) {
        //cerr << "Bad line value: " << str << "\n";
        // Return as valid even though we haven't parsed line as
        // we don't want to mark the whole see message as invalid and
        // throw it out.
        return true;
    }

    vector<string> tokens = tokenise(str);
    valid = (tokens.size() == 9);

    if(valid) {
        name = tokens[0];
        valid = !(tokens[1].compare("pol")) && valid;

        r = atof(tokens[2].c_str());
        theta = atof(tokens[3].c_str());
        phi = atof(tokens[4].c_str());

        valid = !(tokens[5].compare("pol")) && valid;

        r2 = atof(tokens[6].c_str());
        theta2 = atof(tokens[7].c_str());
        phi2 = atof(tokens[8].c_str());

        if(!(name.compare("L"))) {
            lineIndex = LINE;
        } else {
            valid = false;
        }
    }

    if(valid) {
        boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, r2, theta2, phi2, lineIndex ) );
        visionObjs.push_back( obj );
    }
    return valid;
}

bool Parser::parseGoalPost(const string &str, vector< boost::shared_ptr<VisionObject> >& visionObjs) {

    bool valid = false;

    string name;
    double r, theta, phi;

    int goalPostIndex = -1;

    vector<string> tokens = tokenise(str);
    valid = (tokens.size() == 5);

    if(valid) {
        name = tokens[0];
        valid = !(tokens[1].compare("pol")) && valid;

        r = atof(tokens[2].c_str());
        theta = atof(tokens[3].c_str());
        phi = atof(tokens[4].c_str());

        if(!(name.compare("G1L"))) {
            goalPostIndex = GOALPOST_1_L;
        }
        else if(!(name.compare("G1R"))) {
            goalPostIndex = GOALPOST_1_R;
        }
        else if(!(name.compare("G2L"))) {
            goalPostIndex = GOALPOST_2_L;
        }
        else if(!(name.compare("G2R"))) {
            goalPostIndex = GOALPOST_2_R;
        }
        else {
            valid = false;
        }
    }

    if(valid) {
        boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, goalPostIndex ) );
        visionObjs.push_back( obj );
    }
    return valid;
}

bool Parser::parseFlag(const string &str, vector< boost::shared_ptr<VisionObject> >& visionObjs) {
    bool valid = false;

    string name;
    double r, theta, phi;

    int flagIndex = -1;

    vector<string> tokens = tokenise(str);
    valid = (tokens.size() == 5);

    if(valid) {
        name = tokens[0];
        valid = !(tokens[1].compare("pol")) && valid;

        r = atof(tokens[2].c_str());
        theta = atof(tokens[3].c_str());
        phi = atof(tokens[4].c_str());

        if(!(name.compare("F1L"))) {
            flagIndex = FLAG_1_L;
        }
        else if(!(name.compare("F1R"))) {
            flagIndex = FLAG_1_R;
        }
        else if(!(name.compare("F2L"))) {
            flagIndex = FLAG_2_L;
        }
        else if(!(name.compare("F2R"))) {
            flagIndex = FLAG_2_R;
        }
        else {
            valid = false;
        }
    }

    if(valid) {
        boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, flagIndex) );
        visionObjs.push_back( obj );
    }
    return valid;
}

bool Parser::parseBall(const string &str, vector< boost::shared_ptr<VisionObject> >& visionObjs) {
    bool valid = false;
    string name;
    double r, theta, phi;

    vector<string> tokens = tokenise(str);
    valid = (tokens.size() == 5);

    if(valid) {
        name = tokens[0];
        valid = !(tokens[1].compare("pol")) && valid;

        r = atof(tokens[2].c_str());
        theta = atof(tokens[3].c_str());
        phi = atof(tokens[4].c_str());
    }

    if(valid) {
        boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, WO_BALL) );
        visionObjs.push_back( obj );
    }

    return valid;
}

bool Parser::parsePlayer(const string &str, vector< boost::shared_ptr<VisionObject> >& visionObjs) {
    double headAnglePan = bodyModel->getJointAngle(EFF_H1);
    double headAngleTilt = bodyModel->getJointAngle(EFF_H2);
    //  VecPosition headAnglePolarCorrection = VecPosition(0, headAnglePan, headAngleTilt);

    bool valid = true;

    string playerTeamName;
    bool playerTeamNameValid = false;

    int uNum = 0;
    bool uNumValid = false;

    double x = 0, y = 0, z = 0;
    bool coordinatesValid = false;

    vector<string> tokens = tokenise(str);
    int numBodyParts = 0;
    bool notMe = false;
    bool isHead = false;
    bool isArmL = false;
    bool isArmR = false;
    bool isFootL = false;
    bool isFootR = false;
    for(size_t i = 0; i < tokens.size(); ++i) {
        if(!(tokens[i].compare("P"))) {
            //Nothing
        }
        else if(!(tokens[i].compare("rlowerarm") && tokens[i].compare("llowerarm") &&
                  tokens[i].compare("rfoot") && tokens[i].compare("lfoot") &&
                  tokens[i].compare("head"))) {
            isHead = !tokens[i].compare("head");
            isArmL = !tokens[i].compare("llowerarm");
            isArmR = !tokens[i].compare("rlowerarm");
            isFootL = !tokens[i].compare("lfoot");
            isFootR = !tokens[i].compare("rfoot");

            numBodyParts++;
        }
        else if(!(tokens[i].compare("team")) && i < tokens.size() - 1) {

            playerTeamName = tokens[i + 1];
            ++i;
            if(playerTeamName.compare(teamName)) {
                worldModel->setOpponentTeamName(playerTeamName);
            }
            playerTeamNameValid = true;
        }
        else if(!(tokens[i].compare("id")) && i < tokens.size() - 1) {
            uNum = atoi(tokens[i + 1].c_str());
            ++i;
            uNumValid = true;
        }
        else if(!(tokens[i].compare("pol")) &&  i < tokens.size() - 3) {
            double r = atof(tokens[i + 1].c_str());
            double theta = atof(tokens[i + 2].c_str());
            double phi = atof(tokens[i + 3].c_str());
            i += 3;
            if (playerTeamName.compare(teamName) || worldModel->getUNum() != uNum) {
                VecPosition posCartesian = (VecPosition(r, theta, phi)).getCartesianFromPolar();
                x += posCartesian.getX();
                y += posCartesian.getY();
                z += posCartesian.getZ();
                notMe = true;
                int objectIndex = -1;
                if (isHead) {
                    if(!playerTeamName.compare(teamName)) {
                        objectIndex = ( WO_TEAMMATE_HEAD1 - 1 ) + uNum;
                    }
                    else {
                        objectIndex = ( WO_OPPONENT_HEAD1 - 1 ) + uNum;
                    }
                }
                else if (isArmL) {
                    if(!playerTeamName.compare(teamName)) {
                        objectIndex = ( WO_TEAMMATE_ARM_L1 - 1 ) + uNum;
                    }
                    else {
                        objectIndex = ( WO_OPPONENT_ARM_L1 - 1 ) + uNum;
                    }
                }
                else if (isArmR) {
                    if(!playerTeamName.compare(teamName)) {
                        objectIndex = ( WO_TEAMMATE_ARM_R1 - 1 ) + uNum;
                    }
                    else {
                        objectIndex = ( WO_OPPONENT_ARM_R1 - 1 ) + uNum;
                    }
                }
                else if (isFootL) {
                    if(!playerTeamName.compare(teamName)) {
                        objectIndex = ( WO_TEAMMATE_FOOT_L1 - 1 ) + uNum;
                    }
                    else {
                        objectIndex = ( WO_OPPONENT_FOOT_L1 - 1 ) + uNum;
                    }
                }
                else if (isFootR) {
                    if(!playerTeamName.compare(teamName)) {
                        objectIndex = ( WO_TEAMMATE_FOOT_R1 - 1 ) + uNum;
                    }
                    else {
                        objectIndex = ( WO_OPPONENT_FOOT_R1 - 1 ) + uNum;
                    }
                }
                else {
                    cout << "Position for unknown body part\n";
                    valid = false;
                }
                if (objectIndex != -1) {
                    boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, objectIndex ) );
                    visionObjs.push_back( obj );
                }
            }
            coordinatesValid = true;
        }
        else {
            cout << "Unknown token " << tokens[i] << "\n";
        }

    }

    valid = valid && playerTeamNameValid && uNumValid && coordinatesValid;
    // TODO: we assume that players unums will always be between 1 and 11.
    if( uNum < 1 || uNum > 11 ) {
        cout << "We only support players numbers between 1 and 11\n";
        valid = false;
    }

    if(valid && notMe) {
        if (numBodyParts == 0) {
            // Preserving backwards compatibility with 2008 sim
            // Can be removed when backwards compatibility no longer an issue
            numBodyParts = 1;
        }
        x /= numBodyParts;
        y /= numBodyParts;
        z /= numBodyParts;

        VecPosition avgPos = VecPosition(x,y,z);

        double r = avgPos.getMagnitude();
        double theta = avgPos.getTheta();
        double phi = avgPos.getPhi();

        if(!playerTeamName.compare(teamName)) {
            int playerIndex = ( WO_TEAMMATE1 - 1 ) + uNum;
            boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, playerIndex ) );
            visionObjs.push_back( obj );
        }
        else {
            int playerIndex = ( WO_OPPONENT1 - 1 ) + uNum;
            boost::shared_ptr<VisionObject> obj( new VisionObject(r, theta, phi, playerIndex ) );
            visionObjs.push_back( obj );
        }
    }

    return valid;
}

bool Parser::parseFRP(const string &str) {

    bool valid;

    string name;
    double centreX = 0.0, centreY = 0.0, centreZ = 0.0;
    double forceX = 0.0, forceY = 0.0, forceZ = 0.0;

    bool validName = false;
    bool validCentre = false;
    bool validForce = false;

    int FRPIndex = -1;

    vector<string> tokens = tokenise(str);
    for(size_t i = 0; i < tokens.size(); ++i) {

        if(!tokens[i].compare("n")) {
            if(i + 1 < tokens.size()) {

                name = tokens[i + 1];

                if(!(name.compare("lf"))) {
                    FRPIndex = FOOT_LEFT;
                    validName = true;
                }
                if(!(name.compare("rf"))) {
                    FRPIndex = FOOT_RIGHT;
                    validName = true;
                }
                if(!(name.compare("lf1"))) {
                    FRPIndex = TOE_LEFT;
                    validName = true;
                }
                if(!(name.compare("rf1"))) {
                    FRPIndex = TOE_RIGHT;
                    validName = true;
                }
            }
        }

        if(!tokens[i].compare("c")) {
            if(i + 3 < tokens.size()) {

                centreX = atof(tokens[i + 1].c_str());
                centreY = atof(tokens[i + 2].c_str());
                centreZ = atof(tokens[i + 3].c_str());
                validCentre = true;
            }
        }

        if(!tokens[i].compare("f")) {
            if(i + 3 < tokens.size()) {

                forceX = atof(tokens[i + 1].c_str());
                forceY = atof(tokens[i + 2].c_str());
                forceZ = atof(tokens[i + 3].c_str());
                validForce = true;
            }
        }

    }

    valid = validName && validCentre && validForce;

    if(valid) {
        if( FRPIndex == FOOT_LEFT ) {
            bodyModel->setFRPLeft(VecPosition(centreX, centreY, centreZ), VecPosition(forceX, forceY, forceZ));
        } else if ( FRPIndex == FOOT_RIGHT ) {
            bodyModel->setFRPRight(VecPosition(centreX, centreY, centreZ), VecPosition(forceX, forceY, forceZ));
        } else if( FRPIndex == TOE_LEFT ) {
            bodyModel->setFRPLeft1(VecPosition(centreX, centreY, centreZ), VecPosition(forceX, forceY, forceZ));
        } else if ( FRPIndex == TOE_RIGHT ) {
            bodyModel->setFRPRight1(VecPosition(centreX, centreY, centreZ), VecPosition(forceX, forceY, forceZ));
        } else {
            cerr << "Error: FRPIndex is not valid (this shouldn't happen...)" << endl;
        }
    } else {
        cout << str << " not valid.\n";
    }

    return valid;
}

#ifdef GROUND_TRUTH_SERVER
bool Parser::parseMyPos(const string &str) {
    bool valid = false;
    vector<string> tokens = tokenise(str);

    if(!tokens[0].compare("mypos") && tokens.size() >= 4 ) {
        double x = atof(tokens[1].c_str());
        double y = atof(tokens[2].c_str());
        double z = atof(tokens[3].c_str());
        worldModel->setMyPositionGroundTruth(VecPosition(x, y, z));

        // if sent the angle as well
        if( tokens.size() >=5 ) {
            double angle = Rad2Deg( atof(tokens[4].c_str()) );
            worldModel->setMyAngDegGroundTruth( angle );
        }

        // if sent ball position as well
        if( tokens.size() >=8 ) {
            double bx = atof(tokens[5].c_str());
            double by = atof(tokens[6].c_str());
            double bz = atof(tokens[7].c_str());
            VecPosition ballPos = VecPosition(bx, by, bz);
            worldModel->setBallGroundTruth(ballPos);
        }

        valid = true;
    } else if (!tokens[0].compare("myorien") && tokens.size() == 2) {
        double angle = atof(tokens[1].c_str());
        worldModel->setMyAngDegGroundTruth( angle );
        valid = true;
    }

    return valid;
}

bool Parser::parseBallPos(const string &str) {
    bool valid = false;
    vector<string> tokens = tokenise(str);

    if(!tokens[0].compare("ballpos") && tokens.size() == 4 ) {
        double x = atof(tokens[1].c_str());
        double y = atof(tokens[2].c_str());
        double z = atof(tokens[3].c_str());
        worldModel->setBallGroundTruth(VecPosition(x, y, z));

        valid = true;
    }

    return valid;
}
#endif

vector<string> Parser::segment(const string &str, const bool &omitEnds) {
    vector<string> v;

    int ptr = 0;
    int length = str.length();

    if(omitEnds) {
        ptr = 1;
        length = str.length() - 1;
    }

    int bracCount = 0;

    string currentString = "";

    do {

        while(ptr < length && str.at(ptr) != '(') {
            ptr++;
        }

        if(ptr < length) {

            currentString = "";

            do {
                char c = str.at(ptr);

                if(c == '(') {
                    bracCount++;
                }
                else if(c == ')') {
                    bracCount--;
                }

                currentString.append(1, c);
                ptr++;

            } while(bracCount != 0 && ptr < length);

            if(bracCount == 0) {
                v.push_back(currentString);
                currentString = "";
            }
        }
    } while(ptr < length);

    return v;
}

bool Parser::parse(const string &input, bool &fParsedVision) {
    bool valid = true;
    bodyModel->setFRPLeft(VecPosition(0, 0, 0), VecPosition(0, 0, 0));
    bodyModel->setFRPRight(VecPosition(0, 0, 0), VecPosition(0, 0, 0));
    bodyModel->setFRPLeft1(VecPosition(0, 0, 0), VecPosition(0, 0, 0));
    bodyModel->setFRPRight1(VecPosition(0, 0, 0), VecPosition(0, 0, 0));
    vector<string> inputSegments = segment(input, false);


    for(size_t i = 0; i < inputSegments.size(); ++i) {
        //Time
        if(inputSegments[i].at(1)== 't') {
            valid = parseTime(inputSegments[i]) && valid;
        }
        else if(inputSegments[i].at(1) == 'G') {
            //GameState
            if(inputSegments[i].at(2) == 'S') {
                valid = parseGameState(inputSegments[i]) && valid;
            }
            //Gyro
            else {
                valid = parseGyro(inputSegments[i]) && valid;
            }
        }
        //Hear
        else if(inputSegments[i].at(1) == 'h') {
            valid = parseHear(inputSegments[i]) && valid;
        }
        //Hinge Joint
        else if(inputSegments[i].at(1) == 'H') {

            valid = parseHingeJoint(inputSegments[i]) && valid;



        }
        //See
        else if(inputSegments[i].at(1) == 'S') {

            fParsedVision = parseSee(inputSegments[i]);
            valid = fParsedVision && valid;
        }
        //FRP
        else if(inputSegments[i].at(1) == 'F') {
            valid = parseFRP(inputSegments[i]) && valid;
        }
        //Accelerometer
        else if(inputSegments[i].at(1) == 'A') {
            valid = parseAccelerometer(inputSegments[i]) && valid;
        }
        else {
            valid = false;
        }

    }

    return valid;
}

void Parser::processVision() {

    if (!worldModel->getSideSet()) {
        return;
    }
    // Print vision info
    vector< boost::shared_ptr<VisionObject> >::iterator begin( visionObjs.begin() );
    vector< boost::shared_ptr<VisionObject> >::iterator end( visionObjs.end() );

    // Fix vision objects w.r.t. head pan and tilt
    //////////////////  double headAnglePan = bodyModel->getJointAngle(EFF_H1);
    ////////////////  double headAngleTilt = bodyModel->getJointAngle(EFF_H2);
    ///////////////  VecPosition headAnglePolarCorrection = VecPosition(0, headAnglePan, headAngleTilt);

    // Compute ground truth angle relative to torso
    VecPosition cameraDir = bodyModel->transformCameraToOrigin(VecPosition(1,0,0));
    SIM::AngDeg cameraAngle = atan2Deg(cameraDir.getY(), cameraDir.getX());
    worldModel->setMyAngDegGroundTruth( worldModel->getMyAngDegGroundTruth() - cameraAngle );
    if (worldModel->getMyAngDegGroundTruth() > 180) {
        worldModel->setMyAngDegGroundTruth(worldModel->getMyAngDegGroundTruth()-360);
    } else if (worldModel->getMyAngDegGroundTruth() < -180) {
        worldModel->setMyAngDegGroundTruth(worldModel->getMyAngDegGroundTruth()+360);
    }


    begin = visionObjs.begin();
    end = visionObjs.end();
    for( ; begin != end; ++begin ) {
        VisionObject& obj = **begin;
        VecPosition objLocalCamera = obj.polar.getCartesianFromPolar();
        VecPosition objLocalOrigin = bodyModel->transformCameraToOrigin(objLocalCamera);
        obj.polar = objLocalOrigin.getPolarFromCartesian();

        //for line data
        if (obj.id == LINE) {
            VecPosition objLocalCamera = obj.polar2.getCartesianFromPolar();
            VecPosition objLocalOrigin = bodyModel->transformCameraToOrigin(objLocalCamera);
            obj.polar2 = objLocalOrigin.getPolarFromCartesian();
        }
    }

    // update the "seen" information to all world objects
    for( int i = WO_BALL; i < NUM_WORLD_OBJS; ++i ) {
        worldModel->getWorldObject( i )->currentlySeen = false;
    }
    worldModel->getWorldObject(LINE)->lines.clear();
    begin = visionObjs.begin();
    end = visionObjs.end();
    for( ; begin != end; ++begin ) {
        VisionObject& visObj = **begin;
        WorldObject* pObj = worldModel->getWorldObject( visObj.id );
        pObj->currentlySeen = true;
        pObj->validPosition = true;
        pObj->cycleLastSeen = worldModel->getCycle();
        pObj->timeLastSeen = worldModel->getTime();
        if (visObj.id == LINE) {
            pObj->lines.push_back(visObj);
        } else {
            pObj->vision = visObj;
        }
    }


    particleFilter->processFrame();


    double cameraHeightFromTorso = bodyModel->transformCameraToOrigin(VecPosition(0,0,0)).getZ();
    if (worldModel->useGroundTruthDataForLocalization()) {
        VecPosition fixedTruePos = worldModel->getMyPositionGroundTruth();
        fixedTruePos.setZ( fixedTruePos.getZ() - cameraHeightFromTorso );
        worldModel->setMyPosition( fixedTruePos );
        worldModel->setMyAngDeg( worldModel->getMyAngDegGroundTruth() );
    } else {
        VecPosition fixedPos = worldModel->getMyPosition();
        //cout << fixedPos.getZ() << "\n";
        // [patmac] TODO Get a better esitmated value for the
        // height instead of using a hard coded value
        fixedPos.setZ( 0.53 - cameraHeightFromTorso );
        worldModel->setMyPosition( fixedPos );
    }


    // update world matrices and moving objects
    VecPosition fieldXPlusYPlus;
    VecPosition fieldXPlusYMinus;
    VecPosition fieldXMinusYPlus;
    VecPosition fieldXMinusYMinus;
    computeLocalCornerPositions( fieldXPlusYPlus, fieldXPlusYMinus,
                                 fieldXMinusYPlus, fieldXMinusYMinus );

    worldModel->updateMatricesAndMovingObjs( fieldXPlusYPlus, fieldXPlusYMinus,
            fieldXMinusYPlus, fieldXMinusYMinus );


    // Kalman Filter processing should generally be after I know my current position (e.g. particleFilter->processFrame())
    if( worldModel->useKalmanFilter() ) {
        worldModel->getBallKalmanFilter()->processFrame();
    }

    const bool DEBUG_AGENT_ORIENTATIONS = false;

    // Update orientations of agents
    if (DEBUG_AGENT_ORIENTATIONS && worldModel->getUNum() == 1) {
        worldModel->getRVSender()->clearStaticDrawings();
        VecPosition pos = worldModel->getMyPosition();
        VecPosition dir = VecPosition(1,0,0);
        cout << "Me\t" << worldModel->getMyAngDeg() << "\n";
        dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
        worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10, 0,1,0);
        worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY(),0,.8,0);
    }

    for (int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
        WorldObject* pObj = worldModel->getWorldObject( i );
        pObj->currentlySeenOrien = false;
        if (pObj->currentlySeen) {
            bool haveLeft = false;
            bool haveRight = false;
            VecPosition left;
            VecPosition right;
            WorldObject* bodyObj = worldModel->getWorldObject( i-WO_TEAMMATE1+WO_TEAMMATE_ARM_L1 );
            if (bodyObj->currentlySeen) {
                left = bodyObj->pos;
                haveLeft = true;
            }
            bodyObj = worldModel->getWorldObject( i-WO_TEAMMATE1+WO_TEAMMATE_FOOT_L1 );
            if (bodyObj->currentlySeen) {
                if (haveLeft) {
                    left = (left+bodyObj->pos)*.5;
                } else {
                    left = bodyObj->pos;
                    haveLeft = true;
                }
            }
            bodyObj = worldModel->getWorldObject( i-WO_TEAMMATE1+WO_TEAMMATE_ARM_R1 );
            if (bodyObj->currentlySeen) {
                right = bodyObj->pos;
                haveRight = true;
            }
            bodyObj = worldModel->getWorldObject( i-WO_TEAMMATE1+WO_TEAMMATE_FOOT_R1 );
            if (bodyObj->currentlySeen) {
                if (haveRight) {
                    right = (right+bodyObj->pos)*.5;
                } else {
                    right = bodyObj->pos;
                    haveRight = true;
                }
            }
            if (haveLeft && haveRight) {
                pObj->orien = -atan2Deg(left.getX()-right.getX(), left.getY()-right.getY());
                pObj->currentlySeenOrien = true;
                pObj->cycleOrienLastSeen = worldModel->getCycle();
                pObj->timeOrienLastSeen = worldModel->getTime();
                if (DEBUG_AGENT_ORIENTATIONS && worldModel->getUNum() == 1) {
                    cout << "Teammate " << i-WO_TEAMMATE1+1 << "\t" << pObj->orien << "\n";
                    VecPosition pos = pObj->pos;
                    VecPosition dir = VecPosition(1,0,0);
                    dir = dir.rotateAboutZ(-pObj->orien);
                    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10, 1,0,0);
                    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY(), .8,0,0);
                }
            }
        }
    }

    // Update orientations of opponents
    for (int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        WorldObject* pObj = worldModel->getWorldObject( i );
        pObj->currentlySeenOrien = false;
        if (pObj->currentlySeen) {
            bool haveLeft = false;
            bool haveRight = false;
            VecPosition left;
            VecPosition right;
            WorldObject* bodyObj = worldModel->getWorldObject( i-WO_OPPONENT1+WO_OPPONENT_ARM_L1 );
            if (bodyObj->currentlySeen) {
                left = bodyObj->pos;
                haveLeft = true;
            }
            bodyObj = worldModel->getWorldObject( i-WO_OPPONENT1+WO_OPPONENT_FOOT_L1 );
            if (bodyObj->currentlySeen) {
                if (haveLeft) {
                    left = (left+bodyObj->pos)*.5;
                } else {
                    left = bodyObj->pos;
                    haveLeft = true;
                }
            }
            bodyObj = worldModel->getWorldObject( i-WO_OPPONENT1+WO_OPPONENT_ARM_R1 );
            if (bodyObj->currentlySeen) {
                right = bodyObj->pos;
                haveRight = true;
            }
            bodyObj = worldModel->getWorldObject( i-WO_OPPONENT1+WO_OPPONENT_FOOT_R1 );
            if (bodyObj->currentlySeen) {
                if (haveRight) {
                    right = (right+bodyObj->pos)*.5;
                } else {
                    right = bodyObj->pos;
                    haveRight = true;
                }
            }
            if (haveLeft && haveRight) {
                pObj->orien = -atan2Deg(left.getX()-right.getX(), left.getY()-right.getY());
                pObj->currentlySeenOrien = true;
                pObj->cycleOrienLastSeen = worldModel->getCycle();
                pObj->timeOrienLastSeen = worldModel->getTime();
                if (DEBUG_AGENT_ORIENTATIONS && worldModel->getUNum() == 1) {
                    cout << "Opponent " << i-WO_OPPONENT1+1 << "\t" << pObj->orien << "\n";
                    VecPosition pos = pObj->pos;
                    VecPosition dir = VecPosition(1,0,0);
                    dir = dir.rotateAboutZ(-pObj->orien);
                    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10, 0,0,1);
                    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY(), 0, 0, .8);
                }
            }
        }
    }


    VecPosition lastBall = worldModel->getBall();


    if (worldModel->getWorldObject(WO_BALL)->currentlySeen) {
        worldModel->setLastBallSightingTime(worldModel->getTime());
        if (worldModel->isLocalized()) {
            VecPosition ballPos = worldModel->getWorldObject(WO_BALL)->pos;
            ballPos.setZ(0);
            worldModel->setLastBallSeenPosition(ballPos);
            worldModel->setLastBallSeenTime(worldModel->getTime());
        }
    }

    processSightings(false /*fIgnoreVision*/);

    double SIGHT_WINDOW_DEG = 45.0;
    for( int i = WO_BALL; i < NUM_WORLD_OBJS; ++i ) {
        WorldObject *pObj = worldModel->getWorldObject( i );

        if (!pObj->currentlySeen && pObj->validPosition) {
            VecPosition objPosLocal = worldModel->g2l(pObj->pos);
            // [patmac] TODO: This is an approximation and should be made more correct
            // with appropriate transforms
            SIM::AngDeg objAngle = atan2Deg(objPosLocal.getY(), objPosLocal.getX()) - bodyModel->getJointAngle(EFF_H1);

            if (abs(objAngle) < SIGHT_WINDOW_DEG) {
                pObj->validPosition = false;
            }

        }
        // Don't reset haveSighting flag here as we need to know its value when updating kalman filters
        //pObj->haveSighting = false;
    }

    worldModel->getOpponentKalmanFilters()->processFrame();

    for( int i = WO_BALL; i < NUM_WORLD_OBJS; ++i ) {
        WorldObject *pObj = worldModel->getWorldObject( i );
        pObj->haveSighting = false;
    }


    for(int i = 0; i < NUM_AGENTS; ++i) {
        WorldObject* opponent_head = worldModel->getWorldObject(WO_OPPONENT_HEAD1+i);
        if (opponent_head->currentlySeen) {
            worldModel->setFallenOpponent(i, opponent_head->pos.getZ() < .3);
        }
    }

    this->fProcessedVision = true;

}

/* Process reported sightings of objects from teammates */
void Parser::processSightings(bool fIgnoreVision) {
    if (!worldModel->getSideSet()) {
        return;
    }

    // Evaluate ball sighting
    if ((fIgnoreVision || !worldModel->getWorldObject(WO_BALL)->currentlySeen) && worldModel->getWorldObject(WO_BALL)->haveSighting) {
        worldModel->setBall(worldModel->getWorldObject(WO_BALL)->sighting);
        worldModel->setLastBallSightingTime(worldModel->getWorldObject(WO_BALL)->sightingTime);
        worldModel->getWorldObject(WO_BALL)->validPosition = true;
    }

    // Evaluate teammate sightings
    for(int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
        WorldObject *pTeammate = worldModel->getWorldObject(i);
        if (worldModel->getUNum() == i - WO_TEAMMATE1 + 1) {
            if (!worldModel->canTrustVision() && pTeammate->haveSighting) {
                VecPosition sighting = pTeammate->sighting;
                particleFilter->setForTeammateSighting(sighting.getX(), sighting.getY(), pTeammate->sightingOrien);
            }
        }
        else if ((fIgnoreVision || !pTeammate->currentlySeen) && pTeammate->haveSighting) {
            pTeammate->pos = pTeammate->sighting;
            pTeammate->orien = pTeammate->sightingOrien;
            pTeammate->validPosition = true;
        } else if (!pTeammate->currentlySeenOrien && pTeammate->haveSighting) {
            pTeammate->orien = pTeammate->sightingOrien;
        }
    }

    // Evaluate opponent sightings
    for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
        WorldObject *pOpponent = worldModel->getWorldObject(i);
        if ((fIgnoreVision || !pOpponent->currentlySeen) && pOpponent->haveSighting) {
            pOpponent->pos = pOpponent->sighting;
            pOpponent->orien = pOpponent->sightingOrien;
            pOpponent->validPosition = true;
        } else if (!pOpponent->currentlySeenOrien && pOpponent->haveSighting) {
            pOpponent->orien = pOpponent->sightingOrien;
        }

        if ((fIgnoreVision || !worldModel->getWorldObject(WO_OPPONENT_HEAD1 + i-WO_OPPONENT1)->currentlySeen) && pOpponent->haveSighting) {
            worldModel->setFallenOpponent(i-WO_OPPONENT1, pOpponent->sighting.getZ() < .3);
        }
    }
}


void Parser::
computeLocalCornerPositions( VecPosition& fieldXPlusYPlus,
                             VecPosition& fieldXPlusYMinus,
                             VecPosition& fieldXMinusYPlus,
                             VecPosition& fieldXMinusYMinus ) {

    // TODO: currently we assume that z is constant
    VecPosition myPos = worldModel->getMyPosition();
    const double myX = myPos.getX();
    const double myY = myPos.getY();
    const double myZ = myPos.getZ();
    const double myOrient = worldModel->getMyAngDeg();

    VecPosition globalPlusPlus, globalPlusMinus, globalMinusPlus, globalMinusMinus;
    if (worldModel->getSide() == SIDE_LEFT ) {
        globalPlusPlus = worldModel->getWorldObject( FLAG_1_R )->pos;
        globalPlusMinus = worldModel->getWorldObject(FLAG_2_R)->pos;
        globalMinusPlus = worldModel->getWorldObject(FLAG_1_L)->pos;
        globalMinusMinus = worldModel->getWorldObject(FLAG_2_L)->pos;
    } else { // SIDE_RIGHT
        globalPlusPlus = worldModel->getWorldObject(FLAG_2_L)->pos;
        globalPlusMinus = worldModel->getWorldObject(FLAG_1_L)->pos;
        globalMinusPlus = worldModel->getWorldObject(FLAG_2_R)->pos;
        globalMinusMinus = worldModel->getWorldObject(FLAG_1_R)->pos;
    }


    // Compute local coordinates of corners
    fieldXPlusYPlus.setVecPosition( globalPlusPlus.getX() - myX,
                                    globalPlusPlus.getY() - myY,
                                    0 - myZ );
    fieldXPlusYPlus = fieldXPlusYPlus.rotateAboutZ( myOrient );

    fieldXPlusYMinus.setVecPosition(  globalPlusMinus.getX() - myX,
                                      globalPlusMinus.getY() - myY,
                                      0 - myZ );
    fieldXPlusYMinus = fieldXPlusYMinus.rotateAboutZ( myOrient );

    fieldXMinusYPlus.setVecPosition(  globalMinusPlus.getX() - myX,
                                      globalMinusPlus.getY() - myY,
                                      0 - myZ );
    fieldXMinusYPlus = fieldXMinusYPlus.rotateAboutZ( myOrient );


    fieldXMinusYMinus.setVecPosition( globalMinusMinus.getX() - myX,
                                      globalMinusMinus.getY() - myY,
                                      0 - myZ );
    fieldXMinusYMinus = fieldXMinusYMinus.rotateAboutZ( myOrient );
}
