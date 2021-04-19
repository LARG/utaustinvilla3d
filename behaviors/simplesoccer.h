#ifndef _SIMPLE_SOCCER_H
#define _SIMPLE_SOCCER_H

#include "naobehavior.h"

enum Role {
    GOALIE,
    SUPPORTER,
    BACK_LEFT,
    BACK_RIGHT,
    MID_LEFT,
    MID_RIGHT,
    WING_LEFT,
    WING_RIGHT,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    ON_BALL,
    NUM_ROLES
};

class SimpleSoccerBehavior : public NaoBehavior {
public:
    SimpleSoccerBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual SkillType selectSkill();

protected:
    int getPlayerClosestToBall();
    SkillType goToSpace(VecPosition space);
    SkillType watchPosition(VecPosition pos);
    VecPosition getSpaceForRole(Role role);
    map<int, Role> getAgentRoles();
    double getAgentDistanceToRole(int uNum, Role role);
};

#endif
