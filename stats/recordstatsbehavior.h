#ifndef _RECORD_STATS_BEHAVIOR_H
#define _RECORD_STATS_BEHAVIOR_H

#include "../behaviors/naobehavior.h"

class RecordStatsBehavior : public NaoBehavior {

    string outputFile;
    int scoreLeft;
    int scoreRight;
    double sumBallPositionX;
    long numBallMeasures;
    long cyclesInPossession;
    long numPossessionMeasures;
    int kickOffsScored;
    int kickOffsMissed;
    int oppKickOffsScored;
    int oppKickOffsMissed;
    int cornersScored;
    int cornersMissed;
    int oppCornersScored;
    int oppCornersMissed;
    int indirectKicksScored;
    int indirectKicksMissed;
    int oppIndirectKicksScored;
    int oppIndirectKicksMissed;
    int kicksAttempted;
    vector<bool> missingOpp;
    vector<bool> missingTeam;

public:

    RecordStatsBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_, const string& outputFile_);

    virtual void updateFitness();

};

#define MAX_TIME_SCORE_KICKOFF_ME 35.0
#define MAX_TIME_SCORE_KICKOFF_OPP 35.0
#define MAX_TIME_SCORE_CORNER_ME 20.0
#define MAX_TIME_SCORE_CORNER_OPP 20.0
#define MAX_TIME_SCORE_INDIRECT_KICK_ME 25.0
#define MAX_TIME_SCORE_INDIRECT_KICK_OPP 25.0

#endif
