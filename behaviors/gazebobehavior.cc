#include "gazebobehavior.h"

GazeboBehavior::
GazeboBehavior( const std::string teamName,
                int uNum,
                const map<string, string>& namedParams_,
                const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void GazeboBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X+.5;
    beamY = 0;
    beamAngle = 0;
}

SkillType GazeboBehavior::
selectSkill() {
    // Walk to the ball
    return goToTarget(ball);

    //return SKILL_STAND;
}
