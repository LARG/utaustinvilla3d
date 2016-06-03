#include "naobehavior.h"
#include <list>
#include <algorithm>
#include "../rvdraw/rvdraw.h"

#include <sys/time.h>

extern int agentBodyType;

// The names of important kicking parameters
const string& ANGLE = "angle";
const string& CW_ANGLE_THRESH = "cw_angle_thresh";
const string& CCW_ANGLE_THRESH = "ccw_angle_thresh";
const string& OFFSET_X = "xoffset";
const string& OFFSET_Y = "yoffset";
const string& BOUNDING_BOX_RIGHT = "max_displacement_right"; // Right and left are flipped for right legged kicks
const string& BOUNDING_BOX_LEFT = "max_displacement_left";
const string& BOUNDING_BOX_TOP = "max_displacement_top";
const string& BOUNDING_BOX_BOTTOM = "max_displacement_bottom";


/*
 * Basic method to kick (or dribble) the ball toward a target position
 */
SkillType NaoBehavior::kickBall(const int kickTypeToUse, const VecPosition &target) {
    kickTarget = target;
    kickDirection = (kickTarget - ball).normalize();

    kickType = kickTypeToUse;

    if (me.getDistanceTo(ball) > 1) {
        // Far away from the ball so walk toward target offset from the ball
        VecPosition approachBallTarget = ball - kickDirection*atof(namedParams.find("drib_target")->second.c_str());
        return goToTarget(approachBallTarget);
    }

    return kickBallAtPresetTarget();
}


/*Computes an estimated cost of reaching the appropriate position to execute a kick
  */
double NaoBehavior::computeKickCost(VecPosition target, SkillType kickType) {
    const double TURN_PENALTY = .5;
    double angle = getStdNameParameter(kickType, ANGLE);

    VecPosition x_vec = (target - ball).normalize();
    VecPosition y_vec = x_vec.rotateAboutZ(-90);

    VecPosition offX = x_vec * getStdNameParameter(kickType, OFFSET_X);
    VecPosition offY = y_vec * (isRightSkill(kickType) ? -1 : 1) * getStdNameParameter(kickType, OFFSET_Y);
    VecPosition stand_pos = ball + offX + offY;
    SIM::AngDeg walkRotation = abs(x_vec.getTheta() + angle - worldModel->getMyAngDeg());
    if (walkRotation > 180.0) {
        walkRotation -= 180.0;
    }
    double cost = me.getDistanceTo(stand_pos) + walkRotation / 180.0 * TURN_PENALTY;
    bool ballInPath =  abs(VecPosition(0, 0, 0).getAngleBetweenPoints(stand_pos - ball, me - ball)) > 90;
    if (ballInPath) {
        cost += .5;
    }
    return cost;
}

/**
 * Positions the agent so as to kick the ball as
 * parameterized by this->kickTarget and this->kickType.
 *
 * After positioning, executes kick.
 */
SkillType NaoBehavior::kickBallAtPresetTarget() {
    static SkillType lastKickSelected = SKILL_NONE;
    static double lastKickSelectedTime = -1;
    double angleThreshold;
    double lateralThreshold;
    SkillType tempSkill;
    VecPosition target;

    SIM::Point2D ballVel = worldModel->getWorldObject(WO_BALL)->absVel;

    // If we're within 1 meter of the ball and not dribbling, we need to select a kick and position ourselves
    // Dribbling is handled later
    if (me.getDistanceTo(ball) <= 1 && KICK_DRIBBLE != kickType) {
        // Figure out which kick to use (left or right foot)
        SkillType kick;
        double ball_dist;
        double ball_angle;
        double target_dist;
        double target_angle;
        getTargetDistanceAndAngle(ball, ball_dist, ball_angle);
        getTargetDistanceAndAngle(kickTarget, target_dist, target_angle);
        SkillType kick_array[2];
        getSkillsForKickType(kickType, kick_array);
        const int NUM_KICKS = sizeof(kick_array)/sizeof(SkillType);
        double lowestKickCost = 99999999;
        SkillType kickSkill = kick_array[0];
        for (int k = 0; k < NUM_KICKS; k++) {
            SkillType kick = kick_array[k];
            double kickCost = computeKickCost(kickTarget, kick);
            if (kickCost < lowestKickCost) {
                lowestKickCost = kickCost;
                kickSkill = kick;
            }
        }

        static double timeLastSetTarget = -100;
        static VecPosition currentTarget = VecPosition(0,0,0);

        // May only change target every five seconds or if kickType changes
        // When changing target reset everything
        VecPosition targetToKickAt;
        if (worldModel->getTime()-timeLastSetTarget > 5  || currentKickType != kickType || currentKick == SKILL_NONE) {
            timeLastSetTarget = worldModel->getTime();
            currentTarget = kickTarget;
            lastKickSelected = kickSkill;
            lastKickSelectedTime = worldModel->getTime();
            currentKick = kickSkill;
            currentKickType = kickType;

        }


        if (lastKickSelected != kickSkill) {
            lastKickSelected = kickSkill;
            lastKickSelectedTime = worldModel->getTime();
        }

        // Kick skill hysteresis requires us to hold a kick skill for a certain amount of time before switching (but don't switch between kick types)
        if (worldModel->getTime() - lastKickSelectedTime > .5 && currentKickType == kickType) {
            currentKick = lastKickSelected;
        }

        return kickBallAtTargetSimplePositioning(currentTarget, currentKick, currentKickType);
    } else {
        // Here is how we dribble

        currentKickType = KICK_DRIBBLE;
        angleThreshold = 10;

        VecPosition localBall = worldModel->g2l(ball);
        localBall.setZ(0);
        SIM::AngDeg localBallAngle = atan2Deg(localBall.getY(), localBall.getX());
        VecPosition me2Ball = ball - me;

        SIM::AngDeg angleError = abs(VecPosition(0, 0, 0).getAngleBetweenPoints(kickDirection, me2Ball));
        SIM::AngDeg turnAngle = localBallAngle;

        if (angleError < angleThreshold) {
            // Dribble
            return goToTargetRelative(localBall, turnAngle);
        }

        VecPosition ballTarget = ball;

        target = ballTarget - (VecPosition(kickDirection)  * atof(namedParams.find("drib_target")->second.c_str()));
        target.setZ(0);

        VecPosition originalTarget = target;
        target = navigateAroundBall(target, .5 /*PROXIMITY_TRESH*/,atof(namedParams.find("drib_coll_thresh")->second.c_str()));
        target = collisionAvoidance(true /*Avoid teamate*/, false /*Avoid opponent*/, false /*Avoid ball*/, .5, .5, target,
                                    false /*fKeepDistance*/);
        target = collisionAvoidance(false /*Avoid teamate*/, false /*Avoid opponent*/, true /*Avoid ball*/, .5 /*PROXIMITY_TRESH*/,
                                    atof(namedParams.find("drib_coll_thresh")->second.c_str()), target);
        VecPosition localTarget = worldModel->g2l(target);
        SIM::AngDeg localTargetAngle = atan2Deg(localTarget.getY(), localTarget.getX());
        //cout << "CIRCLE\t" << worldModel->getGameTime() << "\n";
        /*
            if (me.getDistanceTo(ball) < me.getDistanceTo(originalTarget)) {
                // Walk facing to local target instead of ball if desired
                return goToTargetRelative(localTarget, localTargetAngle, 1.0, false, WalkRequestBlock::PARAMS_POSITIONING);
            }
        */
        return goToTargetRelative(localTarget, turnAngle, 1.0, false /*fAllowOver180Turn*/, WalkRequestBlock::PARAMS_POSITIONING);
    }
}




void NaoBehavior::resetKickState() {
    currentKick = SKILL_NONE;
    currentKickType = KICK_NONE;
}



SkillType NaoBehavior::kickBallAtTargetSimplePositioning(const VecPosition &targetToKickAt, SkillType kick_skill, int kickType) {

    if (kick_skill == SKILL_NONE || kickType == KICK_NONE) {
        // Safety check that we don't have bad values
        cerr << "Bad kick skill/type: kick_skill = " <<  EnumParser<SkillType>::getStringFromEnum( kick_skill ) << "\tkick type = " << kickType << "\n";
        kick_skill = SKILL_KICK_LEFT_LEG;
        kickType = KICK_FORWARD;
    }

    VecPosition x_vec = (targetToKickAt - ball).normalize();
    VecPosition y_vec = x_vec.rotateAboutZ( -90 );

    VecPosition offX = x_vec * getStdNameParameter(kick_skill, OFFSET_X);
    VecPosition offY =   y_vec * (isRightSkill(kick_skill) ? -1 : 1) * getStdNameParameter(kick_skill, OFFSET_Y);
    VecPosition stand_pos = ball + offX + offY;

    SIM::AngDeg walkRotation = x_vec.getTheta() + (isRightSkill(kick_skill) ? -1 : 1) * getStdNameParameter(kick_skill, ANGLE) - worldModel->getMyAngDeg();


    if (walkRotation > 180) {
        walkRotation -= 360;
    } else if (walkRotation < -180) {
        walkRotation += 360;
    }

    VecPosition spToMe = (me - stand_pos);
    VecPosition spToBallX = (ball - stand_pos).normalize();
    VecPosition spToBallY = spToBallX;
    spToBallY.setX(-spToBallX.getY());
    spToBallY.setY(spToBallX.getX());
    spToBallY.normalize(); // Should already be normalized if the ball is on the ground

    VecPosition yOffset = spToMe.project(spToBallY); // left is 0 degrees
    VecPosition xOffset = spToMe.project(spToBallX); // forward is 0 degrees
    bool inPosY = true;
    if (!isRightSkill(kick_skill)) {
        if (getStdNameParameter(kick_skill, OFFSET_Y) > 0) { // SP is to the right of the ball
            inPosY = yOffset.getMagnitude() < getStdNameParameter(kick_skill, (yOffset.getTheta() == spToBallY.getTheta() ? BOUNDING_BOX_LEFT : BOUNDING_BOX_RIGHT));
        } else {
            inPosY = yOffset.getMagnitude() < getStdNameParameter(kick_skill, (yOffset.getTheta() == spToBallY.getTheta() ? BOUNDING_BOX_RIGHT : BOUNDING_BOX_LEFT));
        }
    } else { // Everything is flipped for right foot
        if (getStdNameParameter(kick_skill, OFFSET_Y) > 0) { // SP is to the right of the ball
            inPosY = yOffset.getMagnitude() < getStdNameParameter(kick_skill, (yOffset.getTheta() == spToBallY.getTheta() ? BOUNDING_BOX_RIGHT : BOUNDING_BOX_LEFT));
        } else {
            inPosY = yOffset.getMagnitude() < getStdNameParameter(kick_skill, (yOffset.getTheta() == spToBallY.getTheta() ? BOUNDING_BOX_LEFT : BOUNDING_BOX_RIGHT));
        }
    }
    // Assuming that the xOffset is always negative (behind the ball)
    bool inPosX = xOffset.getMagnitude() < getStdNameParameter(kick_skill, (xOffset.getTheta() == spToBallX.getTheta() ? BOUNDING_BOX_TOP : BOUNDING_BOX_BOTTOM));
    bool inPosAngle = true;
    if (isRightSkill(kick_skill)) { // Flipped for right kicks
        inPosAngle = abs(walkRotation) < getStdNameParameter(kick_skill, walkRotation < 0 ? CW_ANGLE_THRESH : CCW_ANGLE_THRESH);
    } else {
        inPosAngle = abs(walkRotation) < getStdNameParameter(kick_skill, walkRotation < 0 ? CCW_ANGLE_THRESH : CW_ANGLE_THRESH);
    }
    bool inPosition = inPosY && inPosX && inPosAngle;

    bool canExecute = true;

    SIM::Point2D ballVel = worldModel->getWorldObject(WO_BALL)->absVel;
    const double BALL_VEL_THRESH = .1;

    // Don't try and kick a moving ball
    if (ballVel.getMagnitude() > BALL_VEL_THRESH) {
        canExecute = false;
    }

    // if can kick it, make sure stabilized then kick
    bool fBallCurrentlySeen = worldModel->getWorldObject(WO_BALL)->currentlySeen;
    if(fBallCurrentlySeen && inPosition && canExecute) {
        skills[kick_skill]->reset();
        return kick_skill;
    }

    VecPosition targetLoc = worldModel->g2l(stand_pos);
    double walkSpeed = 1;

    if (ball.getAngleBetweenPoints(stand_pos, me) >= atof(namedParams.find("kick_gen_approach_navBallAngle")->second.c_str())) { // If we're not behind the ball, consider navigating around it
        double navBallDist = atof(namedParams.find("kick_gen_approach_navBallDist")->second.c_str());
        targetLoc = worldModel->g2l(navigateAroundBall(stand_pos, navBallDist, atof(namedParams.find("kick_gen_approach_navBallCollision")->second.c_str())));
    }

    SIM::AngDeg walkDirection = targetLoc.getTheta();
    targetLoc.setZ(0);

    if (me.getDistanceTo(stand_pos) > atof(namedParams.find("kick_gen_approach_turnDist")->second.c_str())) { // Face toward stand_pos (to facilitate walking to it) until close enough. Then face the correct angle.
        VecPosition localStandPos = worldModel->g2l(stand_pos);
        localStandPos.setZ(0);
        SIM::AngDeg localSPAngle = atan2Deg(localStandPos.getY(), localStandPos.getX());
        walkRotation = localSPAngle;
        if (ball.getAngleBetweenPoints(stand_pos, me) > 135) {
            walkRotation = atan2Deg(targetLoc.getY(), targetLoc.getX());
        }
    }

    const double MPS_SCALE = 43.674733875; // multiply particle filter's velocity estimate by this to get m/s
    /*const*/
    double MAX_DECEL_X = atof(namedParams.find("kick_gen_approach_maxDecelX")->second.c_str()); // m/s^2
    const double MAX_DECEL_Y = atof(namedParams.find("kick_gen_approach_maxDecelY")->second.c_str()); //m/s^2 // TODO: The command getWalk(90, 0, 1) doesn't actually cause the robot to walk directly to the side...
    const double MAX_VELOCITY_Y = 0.3;//core->motion_->getMaxYSpeed(); // m/s
    const double MAX_VELOCITY_X = 0.8;//core->motion_->getMaxXSpeed(); // m/s
    /*const*/
    double BUFFER_DIST = atof(namedParams.find("kick_gen_approach_buff")->second.c_str()); // Try to stop this far behind the ball

    VecPosition estimatedVelocity = particleFilter->getOdometryDisplacementEstimateXY() * MPS_SCALE + atof(namedParams.find("kick_gen_approach_estVelCorrection")->second.c_str());

    double theta = targetLoc.getTheta();
    if (targetLoc.getX() >= 0 && targetLoc.getY() >= 0) {
        // Do nothing
    } else if (targetLoc.getX() >= 0 && targetLoc.getY() < 0) {
        theta = 360 - theta; // We want sin and cos to end up being positive
    } else if (targetLoc.getX() < 0 && targetLoc.getY() >= 0) {
        theta = 180 - theta;
    } else { // targetLoc.getX() < 0 && targetLoc.getY() < 0
        theta = theta - 180;
    }
    theta = Deg2Rad(theta);

    double distToTargetX = abs(targetLoc.getX());
    if (distToTargetX > 2 * BUFFER_DIST) distToTargetX -= BUFFER_DIST;
    double distToTargetY = abs(targetLoc.getY());
    if (distToTargetY > 2 * BUFFER_DIST) distToTargetY -= BUFFER_DIST;
    double velX = sqrt(2 * MAX_DECEL_X * distToTargetX);
    double velY = sqrt(2 * MAX_DECEL_Y * distToTargetY);
    double desiredVel = hypot(velX, velY);
    if (targetLoc.getX() != 0 && targetLoc.getY() != 0) {
        if (desiredVel * cos(theta) > velX) {
            velY = velX * tan(theta);
        } else if (desiredVel * sin(theta) > velY) {
            velX = velY / tan(theta);
        }
    }
    desiredVel = hypot(velX, velY);
    walkSpeed = desiredVel > estimatedVelocity.getMagnitude() ? desiredVel / hypot(MAX_VELOCITY_X, MAX_VELOCITY_Y) : 0; // TODO Really need to know what walkSpeed the walk engine is currently trying to achieve so we can overshoot by enough to get the desiredVel
    return getWalk(WalkRequestBlock::PARAMS_APPROACH_BALL, walkDirection, walkRotation, walkSpeed);
}



double NaoBehavior::getParameter(const string& name) {
    return atof(namedParams.find(name)->second.c_str());
}

double NaoBehavior::getStdNameParameter(const SkillType kick_skill, const string& parameter) {
    string prefix = "";
    switch(kick_skill) {
    case SKILL_KICK_IK_0_LEFT_LEG:
    case SKILL_KICK_IK_0_RIGHT_LEG:
        prefix = "kick_ik_0_";
        break;
    case SKILL_KICK_LEFT_LEG:
    case SKILL_KICK_RIGHT_LEG:
        prefix = "kick_";
        break;
    default:
        cerr << "Tried to get a parameter for unsupported kick: " + EnumParser<SkillType>::getStringFromEnum(kick_skill) << endl;
        break;
    }
    return getParameter(prefix + parameter);
}

void NaoBehavior::getSkillsForKickType(int kickType, SkillType skills[]) {
    switch(kickType) {
    case KICK_IK:
        skills[0] = SKILL_KICK_IK_0_LEFT_LEG;
        skills[1] = SKILL_KICK_IK_0_RIGHT_LEG;
        break;
    case KICK_FORWARD:
    default:
        skills[0] = SKILL_KICK_LEFT_LEG;
        skills[1] = SKILL_KICK_RIGHT_LEG;
        break;
    }
}


// Determines whether a collision will occur while moving to the ball, adjusting accordingly when necessary
VecPosition NaoBehavior::navigateAroundBall(VecPosition target, double PROXIMITY_THRESH, double COLLISION_THRESHOLD) {
    double distanceToBall = me.getDistanceTo(ball);

    if (fabs(me.getAngleBetweenPoints(target, ball)) >= 90.0 ||
            distanceToBall > me.getDistanceTo(target)) {
        // We shouldn't be running into the ball when going to the target from
        // this position
        target = collisionAvoidance(true/*Avoid teamate*/,false/*Avoid opponent*/,false/*Avoid ball*/, .5, .5, target, false /*fKeepDistance*/);
        return collisionAvoidance(false/*Avoid teamate*/,false/*Avoid opponent*/,true/*Avoid ball*/, PROXIMITY_THRESH, COLLISION_THRESHOLD, target);
    }


    // Determine where you need to move to avoid running into the ball
    VecPosition me2Target = target-me;
    VecPosition me2TargetDir = VecPosition(me2Target).normalize();
    VecPosition me2Ball = ball-me;
    VecPosition me2BallDir = VecPosition(me2Ball).normalize();

    if (distanceToBall > PROXIMITY_THRESH) {
        target = collisionAvoidanceApproach(PROXIMITY_THRESH, COLLISION_THRESHOLD, target, ball);

        VecPosition closestPathPoint = me+
                                       (me2TargetDir*(me2Ball.dotProduct(me2TargetDir)));

        double pathDistanceFromBall = (ball-closestPathPoint).getMagnitude();

        VecPosition originalTarget = target;
        if (pathDistanceFromBall < PROXIMITY_THRESH) {
            double distanceToBall = me2Ball.getMagnitude();
            SIM::AngDeg correctionAngle = fabs(Rad2Deg(asin(min(PROXIMITY_THRESH/distanceToBall, .999))));
            VecPosition correctLeft = ball+me2TargetDir.rotateAboutZ(correctionAngle)*PROXIMITY_THRESH;
            VecPosition correctRight = ball+me2TargetDir.rotateAboutZ(-correctionAngle)*PROXIMITY_THRESH;

            if (correctLeft.getDistanceTo(target) < correctRight.getDistanceTo(target)) {
                target = correctLeft;
            } else {
                target = correctRight;
            }
        }
    } else { // distanceToBall <= PROXIMITY_THRESH

    }

    target = collisionAvoidance(true/*Avoid teammate*/,false/*Avoid opponent*/,false/*Avoid ball*/,.5,.5,target, false /*fKeepDistance*/);
    target = collisionAvoidance(false/*Avoid teammate*/,false/*Avoid opponent*/,true/*Avoid ball*/,PROXIMITY_THRESH,atof(namedParams.find("drib_coll_thresh")->second.c_str()),target);
    return target;
}
