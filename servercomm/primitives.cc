#include "../behaviors/naobehavior.h"
#include "../audio/audio.h"
#include <common/RobotInfo.h>

extern int agentBodyType;

extern bool fFatProxy;

static Effectors UTWalkJointToSimEffectors(const Joint j) {
    switch (j) {
    case HeadYaw:
        return EFF_H1;
    case HeadPitch:
        return EFF_H2;
    case LHipYawPitch:
        return EFF_LL1;
    case LHipRoll:
        return EFF_LL2;
    case LHipPitch:
        return EFF_LL3;
    case LKneePitch:
        return EFF_LL4;
    case LAnklePitch:
        return EFF_LL5;
    case LAnkleRoll:
        return EFF_LL6;
    case LToePitch:
        return EFF_LL7;
    case RHipYawPitch:
        return EFF_RL1;
    case RHipRoll:
        return EFF_RL2;
    case RHipPitch:
        return EFF_RL3;
    case RKneePitch:
        return EFF_RL4;
    case RAnklePitch:
        return EFF_RL5;
    case RAnkleRoll:
        return EFF_RL6;
    case RToePitch:
        return EFF_RL7;
    case LShoulderPitch:
        return EFF_LA1;
    case LShoulderRoll:
        return EFF_LA2;
    case LElbowYaw:
        return EFF_LA3;
    case LElbowRoll:
        return EFF_LA4;
    case RShoulderPitch:
        return EFF_RA1;
    case RShoulderRoll:
        return EFF_RA2;
    case RElbowYaw:
        return EFF_RA3;
    case RElbowRoll:
        return EFF_RA4;
    default:
        cerr << "Unknown Joint_Enum: " << j << "\n";
        return EFF_NUM;
    }
}

string NaoBehavior::composeAction() {

    stringstream ss("");

    if (bodyModel->useOmniWalk()) {

        // First two joints are head joints which we want to control ourself
        for (int i=2; i < EFF_NUM; i++) {
            // i is the UTWalk joint, and then mapped to our joint using JointToUTWalkEffectors
            bodyModel->setTargetAngle(UTWalkJointToSimEffectors(Joint(i)), RAD_T_DEG*raw_joint_commands_->angles_[i]);
        }

        // Always use our computeTorque() for the head
        ss << "(he1 " << bodyModel->computeTorque(EFF_H1) << ")";
        //ss << "(he1 " << bodyModel->computeTorque((HeadYaw, sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(he2 " << bodyModel->computeTorque(EFF_H2) << ")";
        //ss << "(he2 " << bodyModel->computeTorque(HeadPitch, sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";

        ss << "(lae1 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LShoulderPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lae2 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LShoulderRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lae3 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LElbowYaw), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lae4 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LElbowRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";

        ss << "(rae1 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RShoulderPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rae2 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RShoulderRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rae3 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RElbowYaw), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rae4 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RElbowRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";

        ss << "(lle1 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LHipYawPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lle2 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LHipRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lle3 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LHipPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lle4 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LKneePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lle5 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LAnklePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(lle6 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LAnkleRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        if (bodyModel->hasToe()) {
            ss << "(lle7 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(LToePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        }

        ss << "(rle1 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RHipYawPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rle2 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RHipRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rle3 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RHipPitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rle4 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RKneePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rle5 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RAnklePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        ss << "(rle6 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RAnkleRoll), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        if (bodyModel->hasToe()) {
            ss << "(rle7 " << bodyModel->computeTorque(UTWalkJointToSimEffectors(RToePitch), sim_effectors_, raw_joint_angles_, raw_joint_commands_) << ")";
        }

    } else {
        ss << "(he1 " << bodyModel->computeTorque(EFF_H1) << ")";
        ss << "(he2 " << bodyModel->computeTorque(EFF_H2) << ")";

        ss << "(lae1 " << bodyModel->computeTorque(EFF_LA1) << ")";
        ss << "(lae2 " << bodyModel->computeTorque(EFF_LA2) << ")";
        ss << "(lae3 " << bodyModel->computeTorque(EFF_LA3) << ")";
        ss << "(lae4 " << bodyModel->computeTorque(EFF_LA4) << ")";

        ss << "(rae1 " << bodyModel->computeTorque(EFF_RA1) << ")";
        ss << "(rae2 " << bodyModel->computeTorque(EFF_RA2) << ")";
        ss << "(rae3 " << bodyModel->computeTorque(EFF_RA3) << ")";
        ss << "(rae4 " << bodyModel->computeTorque(EFF_RA4) << ")";

        ss << "(lle1 " << bodyModel->computeTorque(EFF_LL1) << ")";
        ss << "(lle2 " << bodyModel->computeTorque(EFF_LL2) << ")";
        ss << "(lle3 " << bodyModel->computeTorque(EFF_LL3) << ")";
        ss << "(lle4 " << bodyModel->computeTorque(EFF_LL4) << ")";
        ss << "(lle5 " << bodyModel->computeTorque(EFF_LL5) << ")";
        ss << "(lle6 " << bodyModel->computeTorque(EFF_LL6) << ")";
        if (bodyModel->hasToe()) {
            ss << "(lle7 " << bodyModel->computeTorque(EFF_LL7) << ")";
        }

        ss << "(rle1 " << bodyModel->computeTorque(EFF_RL1) << ")";
        ss << "(rle2 " << bodyModel->computeTorque(EFF_RL2) << ")";
        ss << "(rle3 " << bodyModel->computeTorque(EFF_RL3) << ")";
        ss << "(rle4 " << bodyModel->computeTorque(EFF_RL4) << ")";
        ss << "(rle5 " << bodyModel->computeTorque(EFF_RL5) << ")";
        ss << "(rle6 " << bodyModel->computeTorque(EFF_RL6) << ")";
        if (bodyModel->hasToe()) {
            ss << "(rle7 " << bodyModel->computeTorque(EFF_RL7) << ")";
        }

    }


    // Create say message
    double time = worldModel->getTime();
    bool fallen = worldModel->isFallen();
    VecPosition ball = worldModel->getLastBallSeenPosition()[0];
    double ballX = ball.getX();
    double ballY = ball.getY();
    double myX = worldModel->l2g(VecPosition(0,0,0)).getX();
    double myY = worldModel->l2g(VecPosition(0,0,0)).getY();
    bool seeingBall = worldModel->getWorldObject(WO_BALL)->currentlySeen;
    double timeBallLastSeen = worldModel->getLastBallSeenTime()[0];
    bool canTrust = worldModel->canTrustVision();
    int uNum = worldModel->getUNum();

    string message;
    if (canTrust || fallen) {
        if(makeSayMessage(uNum, time, timeBallLastSeen, ballX, ballY, myX, myY, fallen, message)) {
            ss << message;
            /*
            cout << "Data Sent:\n";
            cout << "\tTime: " << time << "\n";
            cout << "\tTime ball last seen: " << timeBallLastSeen << "\n";
            cout << "\tFallen: " << fallen << "\n";
            cout << "\tBall x: " << ballX << "\n";
            cout << "\tBall y: " << ballY << "\n";
            cout << "\tMy x: " << myX << "\n";
            cout << "\tMy y: " << myY << "\n";
            cout << "\n";
            */
        }
    }

    // Add dash command if using fat proxy and walking
    if (fFatProxy && bodyModel->useOmniWalk()) {
        double x = velocity.x*100;
        double y = velocity.y*100;
        double rot = velocity.rot*60;
        ss << "(proxy dash " << to_string(x) << " " << to_string(y) << " " << to_string(rot) << ")";
    }

    // Add kick command if using fat proxy and kicking
    if (fFatProxy && isKickSkill(worldModel->getLastSkill())) {
        const double KICKABLE_MARGIN = 0.44;
        double ballDistance = me.getDistanceTo(ball) + abs(worldModel->getBall().getZ());
        if (ballDistance <= KICKABLE_MARGIN) {
            VecPosition kickTargetLocal = worldModel->g2l(kickTarget);
            double kickAngle = atan2Deg(kickTargetLocal.getY(), kickTargetLocal.getX());
            double deltaAngle = abs(kickAngle)/180.0;
            double kickDistance = kickTargetLocal.getMagnitude();
            double kickPower = kickDistance/(1.5 * (1 - 0.25 * deltaAngle - 0.25 * ballDistance / KICKABLE_MARGIN));
            ss << "(proxy kick " << to_string(kickPower) << " " << to_string(kickAngle) << " " << to_string(kickVerticalAngle) << ")";
        }
    }

    return ss.str();
}
