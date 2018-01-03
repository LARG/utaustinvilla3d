#ifndef NAOBEHAVIOR_H
#define NAOBEHAVIOR_H

#include "behavior.h"
#include "../headers/headers.h"
#include "../parser/parser.h"
#include "../worldmodel/worldmodel.h"
#include "../bodymodel/bodymodel.h"
#include "../particlefilter/PFLocalization.h"
#include "../skills/skill.h"

// For UT Walk
#include <MotionCore.h>
#include <memory/Memory.h>
#include <memory/FrameInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SimEffectorBlock.h>
#include <memory/WalkRequestBlock.h>

using namespace std;

// TODO: Temporary home. Not sure if this this the best place to put this.
struct WalkVelocity
{
    WalkRequestBlock::ParamSet paramSet;

    double x;   // X direction velocity (unspecified unit)
    double y;   // Y direction velocity
    double rot;   // Rotational velocity about the Z-axis

    WalkVelocity() : paramSet(WalkRequestBlock::PARAMS_DEFAULT), x(0), y(0), rot(0) {}

    WalkVelocity(const double& velX, const double& velY, const double& velRot) :
        paramSet(WalkRequestBlock::PARAMS_DEFAULT), x(velX), y(velY), rot(velRot) {}

    WalkVelocity(WalkRequestBlock::ParamSet param, const double& velX, const double& velY, const double& velRot) :
        paramSet(param), x(velX), y(velY), rot(velRot) {}

    friend std::ostream& operator<<(std::ostream &out, const WalkVelocity& v)
    {
        out << "Parameter Set: " << v.paramSet << " T: (" << v.x << ", " << v.y << ") |R: " << v.rot;
        return out;
    }
};

class NaoBehavior : public Behavior {
    friend class KickClassifier;
protected:

    double currentFallStateStartTime;

    // TODO: eliminate these and use a better solution
    string classname;

    map< SkillType, boost::shared_ptr<Skill> > skills;
    const map<string, string>& namedParams;
    string rsg;

    std::string agentTeamName;
    int agentUNum;

    Parser *parser;
    WorldModel *worldModel;
    BodyModel *bodyModel;
    PFLocalization* particleFilter;

    // For UT Walk
    MotionCore* core;
    Memory *memory_;
    FrameInfoBlock* frame_info_;
    FrameInfoBlock* vision_frame_info_;
    SensorBlock* raw_sensors_;
    JointBlock* raw_joint_angles_;
    JointCommandBlock* raw_joint_commands_;
    JointBlock* processed_joint_angles_;
    JointCommandBlock* processed_joint_commands_;
    SimEffectorBlock* sim_effectors_;
    WalkVelocity velocity;

    // For UT Walk
    void calculateAngles();
    void preProcessJoints();
    void postProcessJoints();

    double hoverTime;
    bool mInit;
    bool initBeamed;
    double beamTime;


    bool initialized;

    SkillType skill;
    int skillState;

    int fallState;
    bool fallenLeft, fallenRight, fallenDown, fallenUp;
    double fallTimeStamp;
    double fallTimeWait;

    VecPosition kickDirection;
    int kickType;
    VecPosition kickTarget;

    double lastGetupRecoveryTime;

    SkillType currentKick;
    int currentKickType;

    VecPosition me;
    VecPosition myXDirection, myYDirection, myZDirection;

    VecPosition ball;

    //SCORE
    int scoreMe;
    int scoreOpp;

    string monMsg;

    bool fParsedVision;
    string composeAction();

    virtual void resetSkills();
    void resetScales();
    void refresh();
    void act();

    // ----------------------------------------------------
    // ---------  THESE FUNCTIONS ARE
    // ---------  TO BE OVERRIDEN BY AGENTS...
    virtual SkillType selectSkill();
    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual void updateFitness() {}

    // ----------------------------------------------------


    bool checkingFall();

    /**
     * Trims the value to within [min, max].
     *
     * value - The value to trim.
     * min   - The minimum that the value can be.
     * max   - The maximum that the value can be.
     */
    double trim(const double& value, const double& min, const double&max);

    /**
     * Returns the skill that will best approximate the desired
     * walk direction and rotation using the currently implemented
     * walk. This function was designed for use by selectSkill().
     * It allows for a more general implementation of selectSkill()
     * that does not depend (as much) on the currently implemented
     * walk. This function delivers the fastest possible speed, so it
     * is not appropriate for alignment/fine-tuning.
     *
     * For the purpose of this implementation, rotation = 0 is not
     * the same as 360 is not the same as -360. 0 means no rotation,
     * 360 means rotate to the left, while -360 means rotate to the
     * right.
     *
     * direction - The angle to walk in degrees relative to the
     *             direction the robot is facing.
     * rotation  - The angle in degrees to turn the robot.
     * speed     - The percentage of maximum walk speed to use. Should
     *             be a value between 0 and 1. Default 1. This argument
     *             does not affect turn speed.
     * fAllowOver180Turn - allow for turns greater than abs(180) instead
     *                     of mapping them to their complement
     *
     */
    SkillType getWalk(const double& direction, const double& rotation, double speed = 1.0, bool fAllowOver180Turn=false);
    SkillType getWalk(WalkRequestBlock::ParamSet paramSet, const double& direction, double rotation, double speed, bool fAllowOver180Turn=false);

    /**
     * Returns the skill that's needed to get to the target location facing the target
     * rotation. All targets are offsets relative to the robot's current location and
     * orientation. Note that something like (globalTarget - me) is NOT the correct local
     * target. g2l(globalTarget) should be used instead.
     */
    SkillType goToTargetRelative(const VecPosition& targetLoc, const double& targetRot, double speed=1, bool fAllowOver180Turn=false, WalkRequestBlock::ParamSet paramSet=WalkRequestBlock::PARAMS_DEFAULT);

    SkillType goToTarget(const VecPosition &target);

    VecPosition collisionAvoidance(bool avoidTeammate, bool avoidOpponent, bool avoidBall, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, bool fKeepDistance=true);
    VecPosition collisionAvoidanceCorrection(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition collisionAvoidanceApproach(double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition collisionAvoidanceApproach(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition navigateAroundBall(VecPosition target, double PROXIMITY_THRESH, double COLLISION_THRESH );

    void resetKickState();

    double computeKickCost(VecPosition target, SkillType kickType);
    SkillType kickBall(const int kickTypeToUse, const VecPosition &target);
    SkillType kickBallAtPresetTarget();

    void getTargetDistanceAndAngle(const VecPosition &target, double &distance, double &angle);

    SkillType kickBallAtTargetSimplePositioning( const VecPosition &targetToKickAt, SkillType kick_skill, int kickType);

    /**
     * Returns the maximum direction at which we can walk and still maintain maximum forward speed.
     * In other words, returns the angle theta such that if we walk in any direction in [-theta, theta],
     * our forward translation will be the maximum that the walk engine will allow.
     */
    double getLimitingAngleForward();

    bool beamablePlayMode();
    bool improperPlayMode();
    bool improperPlayMode(int pm);
    bool kickPlayMode();
    bool kickPlayMode(int pm, bool eitherTeam=false);
    bool isIndirectKick();
    bool isIndirectKick(int pm);

    void readSkillsFromFile( const std::string& filename);

    bool isRightSkill( SkillType skill );
    bool isLeftSkill( SkillType skill );

    double getParameter(const std::string& name);
    double getStdNameParameter(const SkillType kick_skill, const std::string& parameter);
    void getSkillsForKickType(int kickType, SkillType skillsForType[]);

    SkillType demoKickingCircle();

public:

    NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);
    virtual ~NaoBehavior();

    virtual std::string Init();
    virtual std::string Think(const std::string& message);

    void setMonMessage(const std::string& msg);
    string getMonMessage();

    inline MotionCore* getCore() {
        return core;
    }
};

#endif // NAOBEHAVIOR_H

