#ifndef _OPTIMIZATION_BEHAVIORS_H
#define _OPTIMIZATION_BEHAVIORS_H

#include "naobehavior.h"

bool isBallMoving(const WorldModel *worldModel);

class OptimizationBehaviorFixedKick: public NaoBehavior {
    const string outputFile;

    double timeStart;
    bool hasKicked;
    bool beamChecked;
    bool backwards;
    bool ranIntoBall;
    bool fallen;

    int kick;

    double INIT_WAIT_TIME;

    VecPosition ballInitPos;
    void initKick();
    void writeFitnessToOutputFile(double fitness);

public:

    OptimizationBehaviorFixedKick(const std::string teamName, int uNum, const map<
                                  string, string>& namedParams_, const string& rsg_, const string& outputFile_);

    virtual void beam(double& beamX, double& beamY, double& beamAngle);
    virtual SkillType selectSkill();
    virtual void updateFitness();

};


#endif
