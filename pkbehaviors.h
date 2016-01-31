#ifndef _PK_BEHAVIORS_H
#define _PK_BEHAVIORS_H

#include "naobehavior.h"

class PKShooterBehavior : public NaoBehavior {
public:

    PKShooterBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual SkillType selectSkill();
};

class PKGoalieBehavior : public NaoBehavior {
public:

    PKGoalieBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual SkillType selectSkill();
};

#endif
