#include "recordstatsbehavior.h"
#include <fstream>

extern string gHost;

RecordStatsBehavior::
RecordStatsBehavior( const std::string teamName,
                     int uNum,
                     const map<string, string>& namedParams_,
                     const string& rsg_,
                     const string& outputFile_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_ ),
      outputFile( outputFile_ ) {
    scoreLeft = 0;
    scoreRight = 0;
    sumBallPositionX = 0.0;
    numBallMeasures = 0;
    cyclesInPossession = 0;
    numPossessionMeasures = 0;
    kickOffsScored = 0;
    kickOffsMissed = 0;
    oppKickOffsScored = 0;
    oppKickOffsMissed = 0;
    cornersScored = 0;
    cornersMissed = 0;
    oppCornersScored = 0;
    oppCornersMissed = 0;
    indirectKicksScored = 0;
    indirectKicksMissed = 0;
    oppIndirectKicksScored = 0;
    oppIndirectKicksMissed = 0;
    kicksAttempted = 0;
    missingOpp = vector<bool>(NUM_AGENTS,true);
    missingTeam = vector<bool>(NUM_AGENTS,true);
}

void RecordStatsBehavior::
updateFitness() {

    double gameTime = worldModel->getGameTime();
    static bool written = false;

    static double lastKickOffStart = -1.0;
    static double oppLastKickOffStart = -1.0;
    static bool ourKickOff = false;
    static bool oppKickOff = false;
    static double lastCornerEnd = -1.0;
    static double oppLastCornerEnd = -1.0;
    static double lastIndirectKickEnd = -1.0;
    static double oppLastIndirectKickEnd = -1.0;

    static SkillType previousLastSkill = SKILL_NONE;

    if (isKickSkill(previousLastSkill) && !isKickSkill(worldModel->getLastSkill())) {
        kicksAttempted++;
    }

    previousLastSkill = worldModel->getLastSkill();

    if (improperPlayMode() && (worldModel->getPlayMode() == PM_KICK_OFF_LEFT || worldModel->getPlayMode() == PM_KICK_OFF_RIGHT) && worldModel->getPlayMode() != worldModel->getLastPlayMode()) {
        if (oppKickOff) {
            oppKickOffsMissed++;
        }

        oppKickOff = true;
        oppLastKickOffStart = gameTime;
    }
    if (!improperPlayMode() && (worldModel->getPlayMode() == PM_KICK_OFF_LEFT || worldModel->getPlayMode() == PM_KICK_OFF_RIGHT) && worldModel->getPlayMode() != worldModel->getLastPlayMode()) {
        if (ourKickOff) {
            kickOffsMissed++;
        }

        ourKickOff = true;
        lastKickOffStart = gameTime;
    }

    if (worldModel->getPlayMode() != worldModel->getLastPlayMode() && (worldModel->getLastPlayMode() == PM_CORNER_KICK_LEFT || worldModel->getLastPlayMode() == PM_CORNER_KICK_RIGHT)) {
        if (!improperPlayMode(worldModel->getLastPlayMode())) {
            lastCornerEnd = gameTime;
        } else {
            oppLastCornerEnd = gameTime;
        }
    }

    if (worldModel->getPlayMode() != worldModel->getLastPlayMode() && isIndirectKick(worldModel->getLastPlayMode())
            && worldModel->getLastPlayMode() != PM_KICK_OFF_LEFT && worldModel->getLastPlayMode() != PM_KICK_OFF_RIGHT) {
        if (!improperPlayMode(worldModel->getLastPlayMode())) {
            lastIndirectKickEnd = gameTime;
        } else {
            oppLastIndirectKickEnd = gameTime;
        }
    }

    if (ourKickOff && gameTime-lastKickOffStart > MAX_TIME_SCORE_KICKOFF_ME) {
        ourKickOff = false;
        kickOffsMissed++;
    }

    if (oppKickOff && gameTime-oppLastKickOffStart > MAX_TIME_SCORE_KICKOFF_OPP) {
        oppKickOff = false;
        oppKickOffsMissed++;
    }

    if (lastCornerEnd > 0 && gameTime-lastCornerEnd > MAX_TIME_SCORE_CORNER_ME) {
        lastCornerEnd = -1.0;
        cornersMissed++;
    }

    if (oppLastCornerEnd > 0 && gameTime-oppLastCornerEnd > MAX_TIME_SCORE_CORNER_OPP) {
        oppLastCornerEnd = -1.0;
        oppCornersMissed++;
    }

    if (lastIndirectKickEnd > 0 && (gameTime-lastIndirectKickEnd > MAX_TIME_SCORE_INDIRECT_KICK_ME || isIndirectKick())) {
        lastIndirectKickEnd = -1.0;
        indirectKicksMissed++;
    }

    if (oppLastIndirectKickEnd > 0 && (gameTime-oppLastIndirectKickEnd > MAX_TIME_SCORE_INDIRECT_KICK_OPP || isIndirectKick())) {
        oppLastIndirectKickEnd = -1.0;
        oppIndirectKicksMissed++;
    }

    if (gameTime == 0) {
        for (int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; i++) {
            if (worldModel->getWorldObject(i)->timeLastSeen > 0) {
                missingTeam[i-WO_TEAMMATE1] = false;
            }
        }
        for (int i = WO_OPPONENT1; i <= WO_OPPONENT11; i++) {
            if (worldModel->getWorldObject(i)->timeLastSeen > 0) {
                missingOpp[i-WO_OPPONENT1] = false;
            }
        }
    }

    if (gameTime >= 300.0 && !written) {
        static double startEndHalf = -1;
        if (startEndHalf < 0) {
            startEndHalf = worldModel->getTime();
        }
        if (worldModel->getTime() - startEndHalf < 10) {
            return;
        }

        int scoreMe = worldModel->getSide() == SIDE_LEFT ? scoreLeft : scoreRight;
        int scoreOpp = worldModel->getSide() == SIDE_LEFT ? scoreRight : scoreLeft;
        fstream file;
        file.open(outputFile.c_str(), ios::out | ios::app);

        file << "kicks" << worldModel->getUNum() << " = " << kicksAttempted << endl;
        //cout << "kicks" << worldModel->getUNum() << " = " << kicksAttempted << endl;

        bool writeTeamData = true;
        for (int i = 1; i < worldModel->getUNum(); i++) {
            // Only write team data if all teammates with lower uniform numbers
            // do not exist
            int objectID = WO_TEAMMATE1+i-1;
            if (worldModel->getTime() - worldModel->getWorldObject(objectID)->timeLastSeen <= 10 || worldModel->getTime() - worldModel->getTeammateLastHeardTime(i-1) <= 10) {
                // Agent with uniform number i exists
                writeTeamData = false;
                break;
            }
        }

        if (writeTeamData) {
            file << "score = " << scoreMe << " " << scoreOpp << endl;
            file << "average_ball_posX = " << sumBallPositionX / (double)numBallMeasures << endl;
            file << "kickoffs = " << kickOffsScored << " " << kickOffsMissed << endl;
            file << "opp_kickoffs = " << oppKickOffsScored << " " << oppKickOffsMissed << endl;
            file << "corners = " << cornersScored << " " << cornersMissed << endl;
            file << "opp_corners = " << oppCornersScored << " " << oppCornersMissed << endl;
            file << "indirect_kicks = " << indirectKicksScored << " " << indirectKicksMissed << endl;
            file << "opp_indirect_kicks = " << oppIndirectKicksScored << " " << oppIndirectKicksMissed << endl;
            file << "possession = " << (double)cyclesInPossession/(double)numPossessionMeasures<< endl;
            bool fMissing = false;
            bool fCrash = false;
            bool fMeCrash = false;
            bool fAllCrash = true;
            for (int i = WO_OPPONENT1; i <= WO_OPPONENT11; i++) {
                if (worldModel->getWorldObject(i)->timeLastSeen < 0) {
                    file << "missing_opp " << i-WO_OPPONENT1+1 << endl;
                    fMissing = true;
                } else if (missingOpp[i-WO_OPPONENT1]) {
                    file << "late_opp " << i-WO_OPPONENT1+1 << endl;
                    fMissing = true;
                }
                if (worldModel->getTime() - worldModel->getWorldObject(i)->timeLastSeen > 10 && worldModel->getWorldObject(i)->timeLastSeen > 0) {
                    file << "crash_opp " << i-WO_OPPONENT1+1 << endl;
                    fCrash = true;
                } else if (worldModel->getWorldObject(i)->timeLastSeen > 0) {
                    fAllCrash = false;
                }
            }
            if (fAllCrash) {
                file << "all_crashed_opp" << endl;
            }
            for (int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; i++) {
                if (worldModel->getUNum() == i-WO_TEAMMATE1+1) {
                    continue;
                }
                if (worldModel->getWorldObject(i)->timeLastSeen < 0) {
                    file << "missing_team " << i-WO_TEAMMATE1+1 << endl;
                    fMissing = true;
                } else if (missingTeam[i-WO_TEAMMATE1]) {
                    file << "late_team " << i-WO_TEAMMATE1+1 << endl;
                    fMissing = true;
                }
                if (worldModel->getTime() - worldModel->getWorldObject(i)->timeLastSeen > 10 && worldModel->getWorldObject(i)->timeLastSeen > 0 && worldModel->getTime() - worldModel->getTeammateLastHeardTime(i-WO_TEAMMATE1) > 10) {
                    file << "crash_team " << i-WO_TEAMMATE1+1 << endl;
                    fCrash = true;
                    fMeCrash = true;
                }
            }

            if (fMissing || fCrash) {
                file << "host " <<  gHost << endl;
            }

            //cout << "score = " << scoreMe << " " << scoreOpp << endl;
            //cout << "average_ball_posX = " << sumBallPositionX / (double)numBallMeasures << endl;
            //cout << "kickoffs = " << kickOffsScored << " " << kickOffsMissed << endl;
            //cout << "opp_kickoffs = " << oppKickOffsScored << " " << oppKickOffsMissed << endl;
            //cout << "corners = " << cornersScored << " " << cornersMissed << endl;
            //cout << "opp_corners = " << oppCornersScored << " " << oppCornersMissed << endl;
            //cout << "indirect_kicks = " << indirectKicksScored << " " << indirectKicksMissed << endl;
            //cout << "opp_indirect_kicks = " << oppIndirectKicksScored << " " << oppIndirectKicksMissed << endl;
            //cout << "possession = " << (double)cyclesInPossession/(double)numPossessionMeasures << endl;
        }
        file.close();
        written = true;
    } else if (gameTime > 0.0 && gameTime < 300.0) {
        if (worldModel->getLastPlayMode() != worldModel->getPlayMode()) {
            if (worldModel->getPlayMode() == PM_GOAL_LEFT) {
                scoreLeft++;
                if (worldModel->getSide() == SIDE_LEFT) {
                    if (ourKickOff) {
                        kickOffsScored++;
                        ourKickOff = false;
                    }

                    if (lastCornerEnd > 0) {
                        cornersScored++;
                        lastCornerEnd = -1.0;
                    }

                    if (lastIndirectKickEnd > 0) {
                        indirectKicksScored++;
                        lastIndirectKickEnd = -1.0;
                    }
                } else {
                    if (oppKickOff) {
                        oppKickOffsScored++;
                        oppKickOff = false;
                    }

                    if (oppLastCornerEnd > 0) {
                        oppCornersScored++;
                        oppLastCornerEnd = -1.0;
                    }

                    if (oppLastIndirectKickEnd > 0) {
                        oppIndirectKicksScored++;
                        oppLastIndirectKickEnd = -1.0;
                    }
                }
            } else if (worldModel->getPlayMode() == PM_GOAL_RIGHT) {
                scoreRight++;
                if (worldModel->getSide() == SIDE_RIGHT) {
                    if (ourKickOff) {
                        kickOffsScored++;
                        ourKickOff = false;
                    }

                    if (lastCornerEnd > 0) {
                        cornersScored++;
                        lastCornerEnd = -1.0;
                    }

                    if (lastIndirectKickEnd > 0) {
                        indirectKicksScored++;
                        lastIndirectKickEnd = -1.0;
                    }
                } else {
                    if (oppKickOff) {
                        oppKickOffsScored++;
                        oppKickOff = false;
                    }

                    if (oppLastCornerEnd > 0) {
                        oppCornersScored++;
                        oppLastCornerEnd = -1.0;
                    }

                    if (oppLastIndirectKickEnd > 0) {
                        oppIndirectKicksScored++;
                        oppLastIndirectKickEnd = -1.0;
                    }
                }
            } else if (worldModel->getPlayMode() == PM_CORNER_KICK_LEFT || worldModel->getPlayMode() == PM_CORNER_KICK_RIGHT) {
                if (lastCornerEnd > 0) {
                    cornersMissed++;
                    lastCornerEnd = -1.0;
                }

                if (oppLastCornerEnd > 0) {
                    oppCornersMissed++;
                    oppLastCornerEnd = -1.0;
                }
            }
        }

        if (worldModel->getWorldObject(WO_BALL)->validPosition) {
            sumBallPositionX += ball.getX();
            numBallMeasures++;
        }

        double closestTeammateDistanceToBall = 10000;
        for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
            VecPosition temp;
            int playerNum = i - WO_TEAMMATE1 + 1;
            if (worldModel->getUNum() == playerNum) {
                // This is us
                temp = me;
            } else {
                WorldObject* teammate = worldModel->getWorldObject( i );
                if (teammate->validPosition) {
                    temp = teammate->pos;
                } else {
                    continue;
                }
            }
            temp.setZ(0);

            double distanceToBall = temp.getDistanceTo(ball);
            if (distanceToBall < closestTeammateDistanceToBall) {
                closestTeammateDistanceToBall = distanceToBall;
            }
        }

        double closestOpponentDistanceToBall = 10000;
        for(int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i) {
            WorldObject* opponent = worldModel->getWorldObject( i );
            if (!opponent->validPosition) {
                continue;
            }
            VecPosition opp = opponent->pos;
            opp.setZ(0);
            double distanceToBall = opp.getDistanceTo(ball);
            if (distanceToBall < closestOpponentDistanceToBall) {
                closestOpponentDistanceToBall = distanceToBall;
            }
        }

        if (!beamablePlayMode()) {
            if (!improperPlayMode() && (kickPlayMode() || closestTeammateDistanceToBall < closestOpponentDistanceToBall)) {
                cyclesInPossession++;
            }
            numPossessionMeasures++;
        }
    }
}
