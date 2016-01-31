#include "pkbehaviors.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// GOALIE
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

PKGoalieBehavior::
PKGoalieBehavior( const std::string teamName,
                  int uNum,
                  const map<string, string>& namedParams_,
                  const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void PKGoalieBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X+.5;
    beamY = 0;
    beamAngle = 0;
}




SkillType PKGoalieBehavior::
selectSkill() {
    return SKILL_STAND;
}





////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// SHOOTER
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
PKShooterBehavior::
PKShooterBehavior( const std::string teamName,
                   int uNum,
                   const map<string, string>& namedParams_,
                   const string& rsg_ )
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_ ) {
}

void PKShooterBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -0.5;
    beamY = 0;
    beamAngle = 0;
}

SkillType PKShooterBehavior::
selectSkill() {
    return SKILL_STAND;
}
