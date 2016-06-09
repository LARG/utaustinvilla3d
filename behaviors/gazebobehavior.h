#ifndef _GAZEBO_BEHAVIOR_H
#define _GAZEBO_BEHAVIOR_H

#include "naobehavior.h"

class GazeboBehavior : public NaoBehavior {
public:

    GazeboBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual SkillType selectSkill();
};

#endif
