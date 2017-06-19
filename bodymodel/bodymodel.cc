#include "bodymodel.h"
#include <common/RobotInfo.h>
#include <algorithm>
#include <boost/bind.hpp>

// For UT walk
#include <memory/SimEffectorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>

extern int agentBodyType;

BodyModel::BodyModel(WorldModel *worldModel) {

    this->worldModel = worldModel;

    initialiseEffectors();
    initialiseComponents();

    gyroRates = VecPosition(0, 0, 0);
    accelRates = VecPosition(0, 0, 0);

    for(int i = 0; i < HJ_NUM; ++i) {
        joint.push_back(SIMJoint());
    }

    FRPCentreLeft = VecPosition(0, 0, 0);
    FRPForceLeft = VecPosition(0, 0, 0);
    FRPCentreRight = VecPosition(0, 0, 0);
    FRPForceRight = VecPosition(0, 0, 0);
    FRPCentreLeft1 = VecPosition(0, 0, 0);
    FRPForceLeft1 = VecPosition(0, 0, 0);
    FRPCentreRight1 = VecPosition(0, 0, 0);
    FRPForceRight1 = VecPosition(0, 0, 0);

    bodyWorldInterface.createIdentity();

    refreshTorso();

    ll1 = HCTMatrix(component[COMP_LHIP1].backTranslateMatrix);
    ll1.multiply(component[COMP_LHIP2].translateMatrix);

    ll2 = HCTMatrix(component[COMP_LHIP2].backTranslateMatrix);
    ll2.multiply(component[COMP_LTHIGH].translateMatrix);

    ll3 = HCTMatrix(component[COMP_LTHIGH].backTranslateMatrix);
    ll3.multiply(component[COMP_LSHANK].translateMatrix);

    ll4 = HCTMatrix(component[COMP_LSHANK].backTranslateMatrix);
    ll4.multiply(component[COMP_LANKLE].translateMatrix);

    ll5 = HCTMatrix(component[COMP_LANKLE].backTranslateMatrix);
    ll5.multiply(component[COMP_LFOOT].translateMatrix);


    rl1 = HCTMatrix(component[COMP_RHIP1].backTranslateMatrix);
    rl1.multiply(component[COMP_RHIP2].translateMatrix);

    rl2 = HCTMatrix(component[COMP_RHIP2].backTranslateMatrix);
    rl2.multiply(component[COMP_RTHIGH].translateMatrix);

    rl3 = HCTMatrix(component[COMP_RTHIGH].backTranslateMatrix);
    rl3.multiply(component[COMP_RSHANK].translateMatrix);

    rl4 = HCTMatrix(component[COMP_RSHANK].backTranslateMatrix);
    rl4.multiply(component[COMP_RANKLE].translateMatrix);

    rl5 = HCTMatrix(component[COMP_RANKLE].backTranslateMatrix);
    rl5.multiply(component[COMP_RFOOT].translateMatrix);

    fUseOmniWalk = true;
}

BodyModel::BodyModel(const BodyModel *current, const int legIndex, const double a1, const double a2, const double a3, const double a4, const double a5, const double a6) {
    this->worldModel = current->worldModel;
    gyroRates = VecPosition(0, 0, 0);
    accelRates = VecPosition(0, 0, 0);
    fallAngle = 0.0;
    fUseOmniWalk = false;
    effector = std::vector<Effector>();
    component = std::vector<Component>(current->component);
    joint = std::vector<SIMJoint>(current->joint);

    bodyWorldInterface = HCTMatrix(current->bodyWorldInterface);
    ll1 = HCTMatrix(current->ll1);
    ll2 = HCTMatrix(current->ll2);
    ll3 = HCTMatrix(current->ll3);
    ll4 = HCTMatrix(current->ll4);
    ll5 = HCTMatrix(current->ll5);
    rl1 = HCTMatrix(current->rl1);
    rl2 = HCTMatrix(current->rl2);
    rl3 = HCTMatrix(current->rl3);
    rl4 = HCTMatrix(current->rl4);
    rl5 = HCTMatrix(current->rl5);

    if(LEG_LEFT == legIndex) {
        joint[HJ_LL1].angle = a1;
        joint[HJ_LL2].angle = a2;
        joint[HJ_LL3].angle = a3;
        joint[HJ_LL4].angle = a4;
        joint[HJ_LL5].angle = a5;
        joint[HJ_LL6].angle = a6;
    }
    else if (LEG_RIGHT == legIndex) {
        joint[HJ_RL1].angle = a1;
        joint[HJ_RL2].angle = a2;
        joint[HJ_RL3].angle = a3;
        joint[HJ_RL4].angle = a4;
        joint[HJ_RL5].angle = a5;
        joint[HJ_RL6].angle = a6;
    } else {
        throw bad_leg_index(legIndex);
    }

    joint[HJ_LA1].angle = -90;
    joint[HJ_LA2].angle = 0;
    joint[HJ_LA3].angle = 0;
    joint[HJ_LA4].angle = 0;
    joint[HJ_RA1].angle = -90;
    joint[HJ_RA2].angle = 0;
    joint[HJ_RA3].angle = 0;
    joint[HJ_RA4].angle = 0;
}

BodyModel::~BodyModel() {
}

void BodyModel::initialiseEffectors() {

    for(int i = 0; i < EFF_NUM; ++i) {
        effector.push_back(Effector(0, 0, 0, 0, 0, 0));
    }

    // PID controller parameters
    double P = 0.15;
    double I = 0.0;
    double D = 0.01;
    double ET = 2.0;

    if (isGazebo()) {
        P = 0.2;
    }

    //                          minAng  maxAng
    effector[EFF_H1] = Effector(-120.0, 120.0, P, I, D, ET);//NECK
    effector[EFF_H2] = Effector(-45.0, 45.0, P, I, D, ET);//HEAD

    effector[EFF_LA1] = Effector(-120.0, 120.0, P, I, D, ET);//SHOULDER (-y)
    effector[EFF_LA2] = Effector(-1.0, 95.0, P, I, D, ET);//UPPERARM (z)
    effector[EFF_LA3] = Effector(-120.0, 120.0, P, I, D, ET);//LOWERARM (x)
    effector[EFF_LA4] = Effector(-90.0, 1.0, P, I, D, ET);//ELBOW (z)

    effector[EFF_RA1] = Effector(-120.0, 120.0, P, I, D, ET);//SHOULDER (-y)
    effector[EFF_RA2] = Effector(-95.0, 1.0, P, I, D, ET);//UPPERARM (z)
    effector[EFF_RA3] = Effector(-120.0, 120.0, P, I, D, ET);//LOWERARM (x)
    effector[EFF_RA4] = Effector(-1.0, 90.0, P, I, D, ET);//ELBOW (z)

    effector[EFF_LL1] = Effector(-90.0, 1.0, P, I, D, ET);//HIP1 (y, -z)
    effector[EFF_LL2] = Effector(-25.0, 45.0, P, I, D, ET);//HIP2 (x)
    effector[EFF_LL3] = Effector(-25.0, 100.0, P, I, D, ET);//HIP3 (-y)
    effector[EFF_LL4] = Effector(-130.0, 1.0, P, I, D, ET);//KNEE (-y)
    effector[EFF_LL5] = Effector(-45.0, 75.0, P, I, D, ET);//ANKLE PITCH (-y)
    effector[EFF_LL6] = Effector(-45.0, 25.0, P, I, D, ET);//ANKLE ROLL (x)
    effector[EFF_LL7] = Effector(-1.0, 70.0, P, I, D, ET);//TOE

    effector[EFF_RL1] = Effector(-90.0, 1.0, P, I, D, ET);//HIP1 (y, z)
    effector[EFF_RL2] = Effector(-45.0, 25.0, P, I, D, ET);//HIP2 (x)
    effector[EFF_RL3] = Effector(-25.0, 100.0, P, I, D, ET);//HIP3 (-y)
    effector[EFF_RL4] = Effector(-130.0, 1.0, P, I, D, ET);//KNEE (-y)
    effector[EFF_RL5] = Effector(-45.0, 75.0, P, I, D, ET);//ANKLE PITCH (-y)
    effector[EFF_RL6] = Effector(-25.0, 45.0, P, I, D, ET);//ANKLE ROLL (x)
    effector[EFF_RL7] = Effector(-1.0, 70.0, P, I, D, ET);//TOE


    // Inializing reflection logic
    reflectedEffector.resize( EFF_NUM );
    toReflectAngle.resize( EFF_NUM );
    reflectedEffector[EFF_H1] = EFF_H1;
    toReflectAngle[EFF_H1] = false;
    reflectedEffector[EFF_H2] = EFF_H2;
    toReflectAngle[EFF_H2] = false;

    reflectedEffector[EFF_LA1] = EFF_RA1;
    toReflectAngle[EFF_LA1] = false;
    reflectedEffector[EFF_LA2] = EFF_RA2;
    toReflectAngle[EFF_LA2] = true;
    reflectedEffector[EFF_LA3] = EFF_RA3;
    toReflectAngle[EFF_LA3] = true;
    reflectedEffector[EFF_LA4] = EFF_RA4;
    toReflectAngle[EFF_LA4] = true;

    reflectedEffector[EFF_RA1] = EFF_LA1;
    toReflectAngle[EFF_RA1] = false;
    reflectedEffector[EFF_RA2] = EFF_LA2;
    toReflectAngle[EFF_RA2] = true;
    reflectedEffector[EFF_RA3] = EFF_LA3;
    toReflectAngle[EFF_RA3] = true;
    reflectedEffector[EFF_RA4] = EFF_LA4;
    toReflectAngle[EFF_RA4] = true;

    reflectedEffector[EFF_LL1] = EFF_RL1;
    toReflectAngle[EFF_LL1] = false;
    reflectedEffector[EFF_LL2] = EFF_RL2;
    toReflectAngle[EFF_LL2] = true;
    reflectedEffector[EFF_LL3] = EFF_RL3;
    toReflectAngle[EFF_LL3] = false;
    reflectedEffector[EFF_LL4] = EFF_RL4;
    toReflectAngle[EFF_LL4] = false;
    reflectedEffector[EFF_LL5] = EFF_RL5;
    toReflectAngle[EFF_LL5] = false;
    reflectedEffector[EFF_LL6] = EFF_RL6;
    toReflectAngle[EFF_LL6] = true;
    reflectedEffector[EFF_LL7] = EFF_RL7;
    toReflectAngle[EFF_LL7] = false;

    reflectedEffector[EFF_RL1] = EFF_LL1;
    toReflectAngle[EFF_RL1] = false;
    reflectedEffector[EFF_RL2] = EFF_LL2;
    toReflectAngle[EFF_RL2] = true;
    reflectedEffector[EFF_RL3] = EFF_LL3;
    toReflectAngle[EFF_RL3] = false;
    reflectedEffector[EFF_RL4] = EFF_LL4;
    toReflectAngle[EFF_RL4] = false;
    reflectedEffector[EFF_RL5] = EFF_LL5;
    toReflectAngle[EFF_RL5] = false;
    reflectedEffector[EFF_RL6] = EFF_LL6;
    toReflectAngle[EFF_RL6] = true;
    reflectedEffector[EFF_RL7] = EFF_LL7;
    toReflectAngle[EFF_RL7] = false;

}

void BodyModel::initialiseComponents() {

    for(int i = 0; i < COMP_NUM; ++i) {
        component.push_back(Component(0, 0, VecPosition(0, 0, 0), VecPosition(0, 0, 0), VecPosition(0, 0, 0)));
    }

    int index;
    int parent;
    double mass;
    VecPosition translation;
    VecPosition anchor;
    VecPosition axis;

    //TORSO
    index = COMP_TORSO;
    parent = COMP_TORSO;
    mass = 1.2171;
    translation = VecPosition(0, 0, 0);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //NECK
    index = COMP_NECK;
    parent = COMP_TORSO;
    mass = 0.05;
    translation = VecPosition(0, 0, 0.09);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, 0, 1.0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //HEAD
    index = COMP_HEAD;
    parent = COMP_NECK;
    mass = 0.35;
    translation = VecPosition(0, 0, 0.065);
    anchor = VecPosition(0, 0, -0.005);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT SHOULDER
    index = COMP_LSHOULDER;
    parent = COMP_TORSO;
    mass = 0.07;
    translation = VecPosition(0, 0.098, 0.075);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT UPPERARM
    index = COMP_LUPPERARM;
    parent = COMP_LSHOULDER;
    mass = 0.15;
    translation = VecPosition(0.02, 0.01, 0);
    anchor = -translation;
    axis = VecPosition(0, 0, 1.0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT ELBOW
    index = COMP_LELBOW;
    parent = COMP_LUPPERARM;
    mass = 0.035;
    translation = VecPosition(0.07, -0.01, 0.009);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT LOWERARM
    index = COMP_LLOWERARM;
    parent = COMP_LELBOW;
    mass = 0.2;
    translation = VecPosition(0.05, 0, 0);
    anchor = -translation;
    axis = VecPosition(0, 0, 1.0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT SHOULDER
    index = COMP_RSHOULDER;
    parent = COMP_TORSO;
    mass = 0.07;
    translation = VecPosition(0, -0.098, 0.075);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT UPPERARM
    index = COMP_RUPPERARM;
    parent = COMP_RSHOULDER;
    mass = 0.15;
    translation = VecPosition(0.02, -0.01, 0);
    anchor = -translation;
    axis = VecPosition(0, 0, 1.0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT ELBOW
    index = COMP_RELBOW;
    parent = COMP_RUPPERARM;
    mass = 0.035;
    translation = VecPosition(0.07, 0.01, 0.009);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT LOWERARM
    index = COMP_RLOWERARM;
    parent = COMP_RELBOW;
    mass = 0.2;
    translation = VecPosition(0.05, 0, 0);
    anchor = -translation;
    axis = VecPosition(0, 0, 1.0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT HIP1
    index = COMP_LHIP1;
    parent = COMP_TORSO;
    mass = 0.09;
    translation = VecPosition(-0.01, 0.055, -0.115);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, 1.0, -1.0).normalize();
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT HIP2
    index = COMP_LHIP2;
    parent = COMP_LHIP1;
    mass = 0.125;
    translation = VecPosition(0, 0, 0);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT THIGH
    index = COMP_LTHIGH;
    parent = COMP_LHIP2;
    mass = 0.275;
    translation = VecPosition(0.01, 0, -0.04);
    anchor = -translation;
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT SHANK
    index = COMP_LSHANK;
    parent = COMP_LTHIGH;
    mass = 0.225;
    translation = VecPosition(0.005, 0, -0.125);
    anchor = VecPosition(-0.01, 0, 0.045);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT ANKLE
    index = COMP_LANKLE;
    parent = COMP_LSHANK;
    mass = 0.125;
    translation = VecPosition(-0.01, 0, -0.055);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //LEFT FOOT
    index = COMP_LFOOT;
    parent = COMP_LANKLE;
    mass = 0.2;
    translation = VecPosition(0.03, 0, -0.04);
    anchor = -translation;
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT HIP1
    index = COMP_RHIP1;
    parent = COMP_TORSO;
    mass = 0.09;
    translation = VecPosition(-0.01, -0.055, -0.115);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, -1.0, 1.0).normalize();
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT HIP2
    index = COMP_RHIP2;
    parent = COMP_RHIP1;
    mass = 0.125;
    translation = VecPosition(0, 0, 0);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT THIGH
    index = COMP_RTHIGH;
    parent = COMP_RHIP2;
    mass = 0.275;
    translation = VecPosition(0.01, 0, -0.04);
    anchor = -translation;
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT SHANK
    index = COMP_RSHANK;
    parent = COMP_RTHIGH;
    mass = 0.225;
    translation = VecPosition(0.005, 0, -0.125);
    anchor = VecPosition(-0.01, 0, 0.045);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT ANKLE
    index = COMP_RANKLE;
    parent = COMP_RSHANK;
    mass = 0.125;
    translation = VecPosition(-0.01, 0, -0.055);
    anchor = VecPosition(0, 0, 0);
    axis = VecPosition(0, -1.0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);

    //RIGHT FOOT
    index = COMP_RFOOT;
    parent = COMP_RANKLE;
    mass = 0.2;
    translation = VecPosition(0.03, 0, -0.04);
    anchor = -translation;
    axis = VecPosition(1.0, 0, 0);
    component[index] = Component(parent, mass, translation, anchor, axis);
}

void BodyModel::refresh() {


    for(int i = 0; i < HJ_NUM; ++i) {
        setCurrentAngle(i, joint[i].angle);
    }

    computeFallAngle();

    //Only needed for cout information.
    refreshTorso();
    refreshHead();
    refreshLeftArm();
    refreshRightArm();
    refreshLeftLeg();
    refreshRightLeg();
}

void BodyModel::refreshTorso() {

    component[COMP_TORSO].transformFromParent = HCTMatrix(bodyWorldInterface);
    component[COMP_TORSO].transformFromRoot = HCTMatrix(bodyWorldInterface);
}

void BodyModel::refreshComponent(const int &index) {

    component[index].transformFromParent.createIdentity();

    HCTMatrix translate = HCTMatrix(HCT_TRANSLATE, component[index].translation + component[index].anchor);
    //Hack: Assumes joint[index] = component[index + 1]
    HCTMatrix rotate = HCTMatrix(HCT_GENERALIZED_ROTATE, component[index].axis, joint[index - 1].angle);
    HCTMatrix backTranslate = HCTMatrix(HCT_TRANSLATE, -component[index].anchor);

    component[index].transformFromParent.multiply(translate);
    component[index].transformFromParent.multiply(rotate);
    component[index].transformFromParent.multiply(backTranslate);

    component[index].transformFromRoot = HCTMatrix(component[component[index].parent].transformFromRoot);
    component[index].transformFromRoot.multiply(component[index].transformFromParent);
}

void BodyModel::refreshHead() {

    refreshComponent(COMP_NECK);
    refreshComponent(COMP_HEAD);
}

void BodyModel::refreshLeftArm() {

    refreshComponent(COMP_LSHOULDER);
    refreshComponent(COMP_LUPPERARM);
    refreshComponent(COMP_LELBOW);
    refreshComponent(COMP_LLOWERARM);

}

void BodyModel::refreshRightArm() {

    refreshComponent(COMP_RSHOULDER);
    refreshComponent(COMP_RUPPERARM);
    refreshComponent(COMP_RELBOW);
    refreshComponent(COMP_RLOWERARM);
}

void BodyModel::refreshLeftLeg() {

    refreshComponent(COMP_LHIP1);
    refreshComponent(COMP_LHIP2);
    refreshComponent(COMP_LTHIGH);
    refreshComponent(COMP_LSHANK);
    refreshComponent(COMP_LANKLE);
    refreshComponent(COMP_LFOOT);
}

void BodyModel::refreshRightLeg() {

    refreshComponent(COMP_RHIP1);
    refreshComponent(COMP_RHIP2);
    refreshComponent(COMP_RTHIGH);
    refreshComponent(COMP_RSHANK);
    refreshComponent(COMP_RANKLE);
    refreshComponent(COMP_RFOOT);
}

double BodyModel::computeTorque(const int &effectorID) {

    effector[effectorID].updateErrors();

    double torque = effector[effectorID].k1 * effector[effectorID].currentError;
    torque += effector[effectorID].k2 * effector[effectorID].cumulativeError;
    torque += effector[effectorID].k3 * (effector[effectorID].currentError - effector[effectorID].previousError);

    return effector[effectorID].scale * torque;
}


// This function works according to Sam and Michael's. We currently don't sent the head joints to it.
double BodyModel::computeTorque(const int &effectorID, SimEffectorBlock* sim_effectors_, JointBlock* raw_joint_angles_, JointCommandBlock* raw_joint_commands_) {

    effector[effectorID].updateErrors();

    double torque = effector[effectorID].k1 * effector[effectorID].currentError;
    torque += effector[effectorID].k2 * effector[effectorID].cumulativeError;
    torque += effector[effectorID].k3 * (effector[effectorID].currentError - effector[effectorID].previousError);


    float m = -0.7 / 990;
    float b = 1.0 - m * 10;
    float command_time_factor = m * raw_joint_commands_->angle_time_ + b;

    return  effector[effectorID].scale * torque * command_time_factor;
}

void BodyModel::display() {

    cout << "*****************Body Model******************************\n";

    /*
    cout << "HJ_H1: " << getJointAngle(HJ_H1) << " ";
    cout << "HJ_H2: " << getJointAngle(HJ_H2) << "\n";

    cout << "HJ_LA1: " << getJointAngle(HJ_LA1) << " ";
    cout << "HJ_LA2: " << getJointAngle(HJ_LA2) << " ";
    cout << "HJ_LA3: " << getJointAngle(HJ_LA3) << " ";
    cout << "HJ_LA4: " << getJointAngle(HJ_LA4) << "\n";

    cout << "HJ_RA1: " << getJointAngle(HJ_RA1) << " ";
    cout << "HJ_RA2: " << getJointAngle(HJ_RA2) << " ";
    cout << "HJ_RA3: " << getJointAngle(HJ_RA3) << " ";
    cout << "HJ_RA4: " << getJointAngle(HJ_RA4) << "\n";

    cout << "HJ_LL1: " << getJointAngle(HJ_LL1) << " ";
    cout << "HJ_LL2: " << getJointAngle(HJ_LL2) << " ";
    cout << "HJ_LL3: " << getJointAngle(HJ_LL3) << " ";
    cout << "HJ_LL4: " << getJointAngle(HJ_LL4) << " ";
    cout << "HJ_LL5: " << getJointAngle(HJ_LL5) << " ";
    cout << "HJ_LL6: " << getJointAngle(HJ_LL6) << " ";
    cout << "HJ_LL7: " << getJointAngle(HJ_LL7) << "\n";

    cout << "HJ_RL1: " << getJointAngle(HJ_RL1) << " ";
    cout << "HJ_RL2: " << getJointAngle(HJ_RL2) << " ";
    cout << "HJ_RL3: " << getJointAngle(HJ_RL3) << " ";
    cout << "HJ_RL4: " << getJointAngle(HJ_RL4) << " ";
    cout << "HJ_RL5: " << getJointAngle(HJ_RL5) << " ";
    cout << "HJ_RL6: " << getJointAngle(HJ_RL6) << " ";
    cout << "HJ_RL7: " << getJointAngle(HJ_RL7) << "\n";

    cout << "GyroRates: " << gyroRates << ").\n";

    cout << "FRP Left: " << FRPCentreLeft << ", " << FRPForceLeft << "\n";
    cout << "FRP Right: " << FRPCentreRight << ", " << FRPForceRight << "\n";
    */

    /*
    HCTMatrix h1 = HCTMatrix(  component[COMP_LANKLE].transformFromRoot);
    HCTMatrix h2 = HCTMatrix(  component[COMP_LANKLE].transformToRoot);
    HCTMatrix h3 = HCTMatrix(h2);
    h3.multiply(h1);

    cout << "^^^^^^^^^^^^^^^H1^^^^^^^^^^^^^^^^^^^^\n";
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++){
        cout << h1.getCell(i, j) << "\t";
      }
      cout << "\n";
    }

    cout << "^^^^^^^^^^^^^^^H2^^^^^^^^^^^^^^^^^^^^\n";
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++){
        cout << h2.getCell(i, j) << "\t";
      }
      cout << "\n";
    }

    cout << "^^^^^^^^^^^^^^^H3^^^^^^^^^^^^^^^^^^^^\n";
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++){
        cout << h3.getCell(i, j) << "\t";
      }
      cout << "\n";
    }

    cout << "$$$$$$$$$$$$$$$$debug$$$$$$$$$$$$$$$$$$$\n";
    VecPosition xx = VecPosition(1.0, 0, 0);
    VecPosition yy = VecPosition(0, 1.0, 0);
    VecPosition zz = VecPosition(0, 0, 1.0);

    VecPosition xx1 = component[COMP_HEAD].transformFromParent.transform(xx);
    VecPosition yy1 = component[COMP_HEAD].transformFromParent.transform(yy);
    VecPosition zz1 = component[COMP_HEAD].transformFromParent.transform(zz);

    cout << "xx1: " << xx1 << "\n";
    cout << "yy1: " << yy1 << "\n";
    cout << "zz1: " << zz1 << "\n";
    */

}

void BodyModel::displayDerived() {

    VecPosition CG = worldModel->l2g(VecPosition(0, 0, 0));
    cout << "TORSO: " << CG << "\n";

    VecPosition rightFootCG = worldModel->l2g(HCTMatrix((component[COMP_RFOOT].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftArmCG = worldModel->l2g(HCTMatrix((component[COMP_LLOWERARM].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition rightArmCG = worldModel->l2g(HCTMatrix((component[COMP_RLOWERARM].transformFromRoot)).transform(VecPosition(0, 0, 0)));

    VecPosition leftFootCG = worldModel->l2g(HCTMatrix((component[COMP_LFOOT].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftAnkleCG = worldModel->l2g(HCTMatrix((component[COMP_LANKLE].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftShankCG = worldModel->l2g(HCTMatrix((component[COMP_LSHANK].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftThighCG = worldModel->l2g(HCTMatrix((component[COMP_LTHIGH].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftHip2CG = worldModel->l2g(HCTMatrix((component[COMP_LHIP2].transformFromRoot)).transform(VecPosition(0, 0, 0)));
    VecPosition leftHip1CG = worldModel->l2g(HCTMatrix((component[COMP_LHIP1].transformFromRoot)).transform(VecPosition(0, 0, 0)));

    cout << "LHIP1 CG: " << leftHip1CG << "\n";
    cout << "LHIP2 CG: " << leftHip2CG << "\n";
    cout << "LTHIGH CG: " << leftThighCG << "\n";
    cout << "LSHANK CG: " << leftShankCG << "\n";
    cout << "LANKLE CG: " << leftAnkleCG << "\n";
    cout << "LFOOT CG: " << leftFootCG << "\n";
    cout << "LARM CG: " << leftArmCG << "\n";
    cout << "RARM CG: " << rightArmCG << "\n";
    cout << "RFOOT CG: " << rightFootCG << "\n";

    /*
      worldModel->getRVSender()->drawCircle("lhip1cg", leftHip1CG.getX(), leftHip1CG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("lhip2cg", leftHip2CG.getX(), leftHip2CG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("lthighcg", leftThighCG.getX(), leftThighCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("lshankcg", leftShankCG.getX(), leftShankCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("lanklecg", leftAnkleCG.getX(), leftAnkleCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("lfootcg", leftFootCG.getX(), leftFootCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("rfootcg", rightFootCG.getX(), rightFootCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("larmcg", leftArmCG.getX(), leftArmCG.getY(), 0.01, RVSender::PINK);
      worldModel->getRVSender()->drawCircle("rarmcg", rightArmCG.getX(), rightArmCG.getY(), 0.01, RVSender::PINK);
    */
}

/**
 * Determines if it is possible to reach out the specified leg to the specified position and direction.
 * The roll and pitch are set to 0.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] xyz       Desired position of the foot w.r.t. the torso.
 * \param[in] yaw       Desired yaw of the foot w.r.t the torso.
 * \return              true iff a solution was found.
 */
bool BodyModel::canReachOutLeg(const int &legIndex, const VecPosition &xyz, const double &yaw) const {
    return canReachOutLeg(legIndex, xyz, VecPosition(0, 0, yaw));
}

/**
 * Reaches out (if possible) the specified leg to the specified position and direction.
 * The roll and pitch are set to 0.
 *
 * Sets the joint target angles if a solution is found.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] xyz       Desired position of the foot w.r.t. the torso.
 * \param[in] yaw       Desired yaw of the foot w.r.t the torso.
 * \return              true iff a solution was found and joint target angles were set.
 */
bool BodyModel::reachOutLeg(const int &legIndex, const VecPosition &xyz, const double &yaw) {
    return reachOutLeg(legIndex, xyz, VecPosition(0, 0, yaw));
}

/**
 * Determines if it is possible to reach out the specified leg to the specified 6DOF pose.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] xyz       Desired position of the foot w.r.t. the torso.
 * \param[in] rpy       Desired roll, pitch, and yaw of the foot w.r.t the torso.
 * \return              true iff a solution was found.
 */
bool BodyModel::canReachOutLeg(const int &legIndex, const VecPosition &xyz, const VecPosition &rpy) const {

    double a1, a2, a3, a4, a5, a6;

    bool possible = legInverseKinematics(legIndex, xyz, rpy, a1, a2, a3, a4, a5, a6);
    return possible;
}

/**
 * Reaches out (if possible) the specified leg to the specified 6DOF pose.
 *
 * Sets the joint target angles if a solution is found.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] xyz       Desired position of the foot w.r.t. the torso.
 * \param[in] rpy       Desired roll, pitch, and yaw of the foot w.r.t the torso.
 * \return              true iff a solution was found and joint target angles were set.
 */
bool BodyModel::reachOutLeg(const int &legIndex, const VecPosition &xyz, const VecPosition &rpy) {

    double a1, a2, a3, a4, a5, a6;

    bool possible = legInverseKinematics(legIndex, xyz, rpy, a1, a2, a3, a4, a5, a6);

    if(possible) {
        if(LEG_LEFT == legIndex) {
            setTargetAngle(EFF_LL1, a1);
            setTargetAngle(EFF_LL2, a2);
            setTargetAngle(EFF_LL3, a3);
            setTargetAngle(EFF_LL4, a4);
            setTargetAngle(EFF_LL5, a5);
            setTargetAngle(EFF_LL6, a6);
        }
        else if (LEG_RIGHT == legIndex) {
            setTargetAngle(EFF_RL1, a1);
            setTargetAngle(EFF_RL2, a2);
            setTargetAngle(EFF_RL3, a3);
            setTargetAngle(EFF_RL4, a4);
            setTargetAngle(EFF_RL5, a5);
            setTargetAngle(EFF_RL6, a6);
        } else {
            throw bad_leg_index(legIndex);
        }
    }
    else {
        //LOG_MSG("6DOF pose not reachable", xyz << rpy);
//    cout << "xyz: " << xyz << " rpy: " << rpy << "not reachable.\n";
    }

    return possible;
}

bool BodyModel::targetsReached() {

    bool valid = true;

    for(int i = 0; i < EFF_NUM; ++i) {
        // Don't do this for the head joints now that we're panning as
        // otherwise we might never reach our targets.
        if ((i == EFF_H1) || (i == EFF_H2)) {
            continue;
        }

        if(!(effector[i].targetReached())) {
            //      cout << i << " not reached\n";
            //      cout << effector[i].targetAngle << ", " << effector[i].currentAngle << ", " << effector[i].currentError << "\n";
            valid = false;
            //      return false;
        }
    }

    return valid;
}

VecPosition BodyModel::getFootCG(const int &legIndex, const double &ang1, const double &ang2, const double &ang3, const double &ang4, const double &ang5, const double &ang6) {

    HCTMatrix m = HCTMatrix();
    getFootTransform(legIndex, m, ang1, ang2, ang3, ang4, ang5, ang6);

    return m.transform(VecPosition(0, 0, 0));
}


void BodyModel::getFootTransform(const int &legIndex, HCTMatrix &m, const double &ang1, const double &ang2, const double &ang3, const double &ang4, const double &ang5, const double &ang6) const {

    m = HCTMatrix(component[COMP_TORSO].transformFromRoot);

    if(legIndex == LEG_LEFT) {

        m.multiply(component[COMP_LHIP1].translateMatrix);
        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LHIP1].axis, ang1));
        m.multiply(ll1);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LHIP2].axis, ang2));
        m.multiply(ll2);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LTHIGH].axis, ang3));
        m.multiply(ll3);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LSHANK].axis, ang4));
        m.multiply(ll4);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LANKLE].axis, ang5));
        m.multiply(ll5);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LFOOT].axis, ang6));
        m.multiply(component[COMP_LFOOT].backTranslateMatrix);
    }
    else if(legIndex == LEG_RIGHT) {

        m.multiply(component[COMP_RHIP1].translateMatrix);
        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RHIP1].axis, ang1));
        m.multiply(rl1);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RHIP2].axis, ang2));
        m.multiply(rl2);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RTHIGH].axis, ang3));
        m.multiply(rl3);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RSHANK].axis, ang4));
        m.multiply(rl4);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RANKLE].axis, ang5));
        m.multiply(rl5);

        m.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RFOOT].axis, ang6));
        m.multiply(component[COMP_RFOOT].backTranslateMatrix);
    }

}

VecPosition BodyModel::getFootCG(const int &legIndex) const {

    double a1,a2, a3, a4, a5, a6;

    if(legIndex == LEG_LEFT) {
        a1 = getCurrentAngle(EFF_LL1);
        a2 = getCurrentAngle(EFF_LL2);
        a3 = getCurrentAngle(EFF_LL3);
        a4 = getCurrentAngle(EFF_LL4);
        a5 = getCurrentAngle(EFF_LL5);
        a6 = getCurrentAngle(EFF_LL6);
    }
    else {
        a1 = getCurrentAngle(EFF_RL1);
        a2 = getCurrentAngle(EFF_RL2);
        a3 = getCurrentAngle(EFF_RL3);
        a4 = getCurrentAngle(EFF_RL4);
        a5 = getCurrentAngle(EFF_RL5);
        a6 = getCurrentAngle(EFF_RL6);
    }

    HCTMatrix m = HCTMatrix();
    getFootTransform(legIndex, m, a1, a2, a3, a4, a5, a6);

    return m.transform(VecPosition(0, 0, 0));
}


void BodyModel::getFootGlobalCGAndAxes(const int &legIndex, VecPosition &CG, VecPosition &xAxis, VecPosition &yAxis, VecPosition &zAxis) {

    double a1,a2, a3, a4, a5, a6;

    if(legIndex == LEG_LEFT) {
        a1 = getCurrentAngle(EFF_LL1);
        a2 = getCurrentAngle(EFF_LL2);
        a3 = getCurrentAngle(EFF_LL3);
        a4 = getCurrentAngle(EFF_LL4);
        a5 = getCurrentAngle(EFF_LL5);
        a6 = getCurrentAngle(EFF_LL6);
    }
    else {
        a1 = getCurrentAngle(EFF_RL1);
        a2 = getCurrentAngle(EFF_RL2);
        a3 = getCurrentAngle(EFF_RL3);
        a4 = getCurrentAngle(EFF_RL4);
        a5 = getCurrentAngle(EFF_RL5);
        a6 = getCurrentAngle(EFF_RL6);
    }

    HCTMatrix m = HCTMatrix();
    getFootTransform(legIndex, m, a1, a2, a3, a4, a5, a6);

    HCTMatrix m1 = HCTMatrix();
    m1.multiply(m);

    CG = worldModel->l2g(m1.transform(VecPosition(0, 0, 0)));

    //Adjustment used!

    xAxis = (worldModel->l2g(m1.transform(VecPosition(0, 1.0, 0))) - CG).normalize();
    yAxis = (worldModel->l2g(m1.transform(VecPosition(-1.0, 0, 0))) - CG).normalize();
    zAxis = (worldModel->l2g(m1.transform(VecPosition(0, 0, 1.0))) - CG).normalize();


}

void BodyModel::getFootGlobalCGAndAxesAngles(const int &legIndex, const vector<double> &angles, VecPosition &origin, VecPosition &xAxis, VecPosition &yAxis, VecPosition &zAxis) {

    HCTMatrix m = HCTMatrix();
    getFootTransform(legIndex, m, angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);

    HCTMatrix m1 = HCTMatrix();
    m1.multiply(m);

    origin = worldModel->l2g(m1.transform(VecPosition(0, 0, 0)));

    //Adjustment used!

    xAxis = (worldModel->l2g(m1.transform(VecPosition(0, 1.0, 0))) - origin).normalize();
    yAxis = (worldModel->l2g(m1.transform(VecPosition(-1.0, 0, 0))) - origin).normalize();
    zAxis = (worldModel->l2g(m1.transform(VecPosition(0, 0, 1.0))) - origin).normalize();

}

VecPosition BodyModel::getBallWRTLeftFoot(WorldModel *worldModel) {
    vector<double> leftLegAngles;
    leftLegAngles.resize(6);
    leftLegAngles[0] = getCurrentAngle(EFF_LL1);
    leftLegAngles[1] = getCurrentAngle(EFF_LL2);
    leftLegAngles[2] = getCurrentAngle(EFF_LL3);
    leftLegAngles[3] = getCurrentAngle(EFF_LL4);
    leftLegAngles[4] = getCurrentAngle(EFF_LL5);
    leftLegAngles[5] = getCurrentAngle(EFF_LL6);

    VecPosition leftOrigin, leftXAxis, leftYAxis, leftZAxis;
    getFootGlobalCGAndAxesAngles(LEG_LEFT, leftLegAngles, leftOrigin, leftXAxis, leftYAxis, leftZAxis);

    // Should we or should we not set ball global Z to 0.04? For now, no.
    VecPosition globalBall = worldModel->getBall();
    //globalBall.setZ(0.04);

    VecPosition relative;
    relative.setX((globalBall - leftOrigin).dotProduct(leftXAxis));
    relative.setY((globalBall - leftOrigin).dotProduct(leftYAxis));
    relative.setZ((globalBall - leftOrigin).dotProduct(leftZAxis));

    return relative;
}



VecPosition BodyModel::getBallWRTRightFoot(WorldModel *worldModel) {
    vector<double> rightLegAngles;
    rightLegAngles.resize(6);
    rightLegAngles[0] = getCurrentAngle(EFF_RL1);
    rightLegAngles[1] = getCurrentAngle(EFF_RL2);
    rightLegAngles[2] = getCurrentAngle(EFF_RL3);
    rightLegAngles[3] = getCurrentAngle(EFF_RL4);
    rightLegAngles[4] = getCurrentAngle(EFF_RL5);
    rightLegAngles[5] = getCurrentAngle(EFF_RL6);

    VecPosition rightOrigin, rightXAxis, rightYAxis, rightZAxis;
    getFootGlobalCGAndAxesAngles(LEG_RIGHT, rightLegAngles, rightOrigin, rightXAxis, rightYAxis, rightZAxis);

    // Should we or should we not set ball global Z to 0.04? For now, no.
    VecPosition globalBall = worldModel->getBall();
    //globalBall.setZ(0.04);

    VecPosition relative;
    relative.setX((globalBall - rightOrigin).dotProduct(rightXAxis));
    relative.setY((globalBall - rightOrigin).dotProduct(rightYAxis));
    relative.setZ((globalBall - rightOrigin).dotProduct(rightZAxis));

    return relative;

}

/** Constructs a 3x3 rotation matrix given roll, pitch, and yaw.
 *
 * \param[out] eerot - array of 9 elements into which to write the
 *                     rotation matrix in row-major order.
 * \param[in] rpy    - the roll, pitch, and yaw to apply encoded as
 *                     degrees around the (positive) x-, y-, and z-
 *                     axes, respectively.
 */
void applyRPY(double *eerot, const VecPosition &rpy) {
    HCTMatrix rot, roll, pitch, yaw;

    rot.createIdentity();
    roll.createRotateX(rpy.getX());
    pitch.createRotateY(rpy.getY());
    yaw.createRotateZ(rpy.getZ());

    // To apply roll, pitch, then yaw, we multiply
    // in reverse order
    rot.multiply(yaw);
    rot.multiply(pitch);
    rot.multiply(roll);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eerot[i * 3 + j] = rot.getCell(i, j);
        }
    }
}

/** Converts all the angles in the vector of solutions (joint angles)
 * to degrees.
 *
 * \param[in,out] solutions The solutions in radians are converted in-place
 *                          to degrees.
 */
void radToDeg(vector<joints_t> &solutions) {
    for (size_t i = 0; i < solutions.size(); ++i) {
        for (size_t j = 0; j < solutions[i].size(); ++j) {
            solutions[i][j] *= 180.0 / M_PI;
        }
    }
}

/**
 * Checks each proposed solution for validity.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] solution  Proposed joint angles, in degrees.
 * \return    true if any proposed joint angles are outside the corresponding
 *            effector's range; false otherwise.
 */
bool BodyModel::isInvalidSolution(const int &legIndex, const joints_t &solution) const {
    assert(solution.size() == 6);

    // pick effector ID based on left or right leg
    int eff_id = 0;
    if (LEG_LEFT == legIndex) {
        eff_id = EFF_LL1;
    } else if (LEG_RIGHT == legIndex) {
        eff_id = EFF_RL1;
    } else {
        throw bad_leg_index(legIndex);
    }

    // check each solution angle against the corresponding effector limits
    for (int i = 0; i < 6; ++i) {
        if (!effector[eff_id++].isAngleInRange(solution[i])) {
            return true;
        }
    }

    return false;
}

/**
 * Filters proposed solutions if they contain angles which are outside the
 * effector limits for some joint.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in,out] solutions The proposed solutions to be filtered.
 *                          All bad solutions are removed.
 */
void BodyModel::filterOutOfBounds(const int &legIndex,
                                  vector<joints_t> &solutions) const {
    size_t new_size = remove_if(solutions.begin(), solutions.end(), boost::bind(
                                    &BodyModel::isInvalidSolution, this, legIndex, _1)) - solutions.begin();
    solutions.resize(new_size);
}

/**
 * Picks the best solution out of the proposed, valid solutions.
 *
 * Currently not really implemented: in simply returns the first solution.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] solutions The proposed (and valid) solutions.
 * \return    The best solution.
 */
const joints_t& BodyModel::pickBestSolution(const int &legIndex, vector<
        joints_t> &solutions) const {
    // TODO: implement
    assert(solutions.size() > 0);
    if (solutions.size() > 1) {
        cerr << "Warning: " << __PRETTY_FUNCTION__ << " is unimplemented." << endl;
    }
    assert(!isInvalidSolution(legIndex, solutions[0]));
    return solutions[0];
}

/**
 * Tries to solve the inverse kinematics problem for a leg.
 *
 * \param[in] legIndex  which leg? must be LEG_LEFT or LEG_RIGHT
 * \param[in] xyz       Desired position of the foot w.r.t. the torso.
 * \param[in] rpy       Desired roll, pitch, and yaw of the foot w.r.t the torso.
 * \param[out] a1       Computed angle (in degrees) of joint 1.
 * \param[out] a2       Computed angle (in degrees) of joint 2.
 * \param[out] a3       Computed angle (in degrees) of joint 3.
 * \param[out] a4       Computed angle (in degrees) of joint 4.
 * \param[out] a5       Computed angle (in degrees) of joint 5.
 * \param[out] a6       Computed angle (in degrees) of joint 6.
 * \return              true iff a solution was found.
 *                      a1-a6 are not modified if no solution was found.
 */
bool BodyModel::legInverseKinematics(const int &legIndex,
                                     const VecPosition &xyz, const VecPosition &rpy, double &a1, double &a2,
                                     double &a3, double &a4, double &a5, double &a6) const {
    // convert parameters to ikfast version of them
    double eetrans[3] = { xyz.getX(), xyz.getY(), xyz.getZ() };
    double eerot[9];
    applyRPY(eerot, rpy);

    // get solutions
    vector<joints_t> solutions;
    bool bSuccess = false;
    if (LEG_LEFT == legIndex) {
        bSuccess = ikfast_left_foot_ik(eetrans, eerot, solutions);
    } else if (LEG_RIGHT == legIndex) {
        bSuccess = ikfast_right_foot_ik(eetrans, eerot, solutions);
    } else {
        throw bad_leg_index(legIndex);
    }

    // convert ikfast solutions to a usable form
    radToDeg(solutions);
    filterOutOfBounds(legIndex, solutions);

    // return best solution
    if (solutions.size() > 0) {
        assert(true == bSuccess);
        const joints_t &s = pickBestSolution(legIndex, solutions);
        assert(s.size() == 6);
        a1 = s[0];
        a2 = s[1];
        a3 = s[2];
        a4 = s[3];
        a5 = s[4];
        a6 = s[5];
    } else {
        // whatever "solutions" may have been found by ikfast turned out
        // to be out of bounds for at least one joint.
        bSuccess = false;
    }

    return bSuccess;
}

bool BodyModel::stabilize(const int legIndex, const VecPosition zmp) {
    if (legIndex != LEG_LEFT && legIndex != LEG_RIGHT) {
        throw bad_leg_index(legIndex);
    }
    double stab_pa1, stab_pa2, stab_oa1, stab_oa2, stab_pl2, stab_pl5, stab_pl6, stab_ol2, stab_ol5, stab_ol6;
    if (!applyStabilization(legIndex, zmp, stab_pa1, stab_pa2, stab_oa1, stab_oa2, stab_pl2, stab_pl5, stab_pl6, stab_ol2, stab_ol5, stab_ol6)) {
        return false;
    }
    setTargetAngle(legIndex == LEG_LEFT ? EFF_LA1 : EFF_RA1, stab_pa1);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_LA2 : EFF_RA2, stab_pa2);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_RA1 : EFF_LA1, stab_oa1);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_RA2 : EFF_LA2, stab_oa2);
    setTargetAngle(EFF_LA3, 0);
    setTargetAngle(EFF_LA4, 0);
    setTargetAngle(EFF_RA3, 0);
    setTargetAngle(EFF_RA4, 0);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_LL2 : EFF_RL2, stab_pl2);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_LL5 : EFF_RL5, stab_pl5);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_LL6 : EFF_RL6, stab_pl6);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_RL2 : EFF_LL2, stab_ol2);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_RL5 : EFF_LL5, stab_ol5);
    setTargetAngle(legIndex == LEG_LEFT ? EFF_RL6 : EFF_LL6, stab_ol6);
    return true;
}

bool BodyModel::canStabilize(const int legIndex, const VecPosition zmp) const {
    double stab_pa1, stab_pa2, stab_oa1, stab_oa2, stab_pl2, stab_pl5, stab_pl6, stab_ol2, stab_ol5, stab_ol6;
    return applyStabilization(legIndex, zmp, stab_pa1, stab_pa2, stab_oa1, stab_oa2, stab_pl2, stab_pl5, stab_pl6, stab_ol2, stab_ol5, stab_ol6);
}

/*
 * Move arm joints and (if necessary) ankles and hips so that the robot will be stable over the zmp on the given foot.
 * Return true iff a stabilization solution was found.
 */
bool BodyModel::applyStabilization(const int &legIndex, const VecPosition zmp, double &stab_pa1, double &stab_pa2, double &stab_oa1, double &stab_oa2, double &stab_pl2, double &stab_pl5, double &stab_pl6, double &stab_ol2, double &stab_ol5, double &stab_ol6) const {
    // TODO: We may want to change effector scales to limit the velocity of the center of mass also
    static const double EPSILON = 0.02; // Distance within which we say the CoM is over the ZMP - Quarter of width of foot
    if (legIndex != LEG_LEFT && legIndex != LEG_RIGHT) {
        throw bad_leg_index(legIndex);
    }

//	cout << "\tAttempting stabilization over the " << (legIndex == LEG_LEFT ? "left" : "right") << " leg." << endl;
    double a1 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL1 : EFF_RL1);
    double a2 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL2 : EFF_RL2);
    double a3 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL3 : EFF_RL3);
    double a4 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL4 : EFF_RL4);
    double a5 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL5 : EFF_RL5);
    double a6 = getTargetAngle(legIndex == LEG_LEFT ? EFF_LL6 : EFF_RL6);

    stab_pa1 = -90;
    stab_pa2 = 0;
    stab_oa1 = -90;
    stab_oa2 = 0;
    stab_pl2 = a2;
    stab_pl5 = a5;
    stab_pl6 = a6;
    stab_ol2 = getTargetAngle(legIndex == LEG_LEFT ? EFF_RL2 : EFF_LL2);
    stab_ol5 = getTargetAngle(legIndex == LEG_LEFT ? EFF_RL5 : EFF_LL5);
    stab_ol6 = getTargetAngle(legIndex == LEG_LEFT ? EFF_RL6 : EFF_LL6);

    // First make everything relative to the foot and find direction of motion for center of mass
    HCTMatrix footRelOrigin = HCTMatrix();
    getFootTransform(legIndex, footRelOrigin, a1, a2, a3, a4, a5, a6);
//	VecPosition absoluteZmp = worldModel->l2g(footRelOrigin.transform(VecPosition(0,0,0)) + zmp);
//	worldModel->getRVSender()->drawCircle(/*"zmp0.1", */absoluteZmp.getX(), absoluteZmp.getY(), 0.1, RVSender::LIGHTBLUE);
//	worldModel->getRVSender()->drawCircle(/*"zmp0.01", */absoluteZmp.getX(), absoluteZmp.getY(), 0.01, RVSender::LIGHTBLUE);
    HCTMatrix originRelFoot = footRelOrigin.getInverse();
    if (originRelFoot.isIdentity()) {  // Could not find an inverse
        cerr << "Tried to invert matrix with 0 determinant" << endl;
        return false; // This shouldn't happen since the matrix should have non-zero determinant
    }
    // For simplicity, we will assume that the foot will be flat on the ground
    BodyModel projected = BodyModel(this, legIndex, a1, a2, a3, a4, a5, a6);
    projected.component[COMP_TORSO].transformFromRoot = originRelFoot; // Everything is now relative to the foot in the new body model.
    projected.refreshHead();
    projected.refreshLeftArm();
    projected.refreshRightArm();
    projected.refreshLeftLeg();
    projected.refreshRightLeg();
    VecPosition centerOfMass = projected.getCenterOfMass();
    VecPosition dirToMove = zmp - centerOfMass; // Zero motion point, center of mass, and direction are all relative to the foot
    dirToMove.setZ(0);
    double bodyMass = 0;
    for (uint i = 0; i < component.size(); i++) {
        bodyMass += component[i].mass;
    }

    if (dirToMove.getMagnitude() < EPSILON) {
        return true;
    }

    // Now adjust center of mass by moving the arms, starting with the arm on the side of the plant foot (as it's less likely to collide with the torso)
    double angleBtwnTorsoXAndFootX = footRelOrigin.transform(VecPosition(1, 0, 0)).getTheta();
    VecPosition rotatedDirToMove = dirToMove.rotateAboutZ(-angleBtwnTorsoXAndFootX);
    applyStabilizationArm(legIndex == LEG_LEFT ? ARM_LEFT : ARM_RIGHT, stab_pa1, stab_pa2, rotatedDirToMove, bodyMass);
    projected.joint[legIndex == LEG_LEFT ? EFF_LA1 : EFF_RA1].angle = stab_pa1;
    projected.joint[legIndex == LEG_LEFT ? EFF_LA2 : EFF_RA2].angle = stab_pa2;
    projected.refreshLeftArm(); // Only need one of these, but it doesn't hurt to do both
    projected.refreshRightArm();
    centerOfMass = projected.getCenterOfMass();
    dirToMove = zmp - centerOfMass;
    dirToMove.setZ(0);

    applyStabilizationArm(legIndex == LEG_LEFT ? ARM_RIGHT : ARM_LEFT, stab_oa1, stab_oa2, rotatedDirToMove, bodyMass);
    projected.joint[legIndex == LEG_LEFT ? EFF_RA1 : EFF_LA1].angle = stab_oa1;
    projected.joint[legIndex == LEG_LEFT ? EFF_RA2 : EFF_LA2].angle = stab_oa2;
    projected.refreshRightArm(); // Only need one of these, but it doesn't hurt to do both
    projected.refreshLeftArm();
    centerOfMass = projected.getCenterOfMass();
    dirToMove = zmp - centerOfMass;
    dirToMove.setZ(0);


    // Finally, if necessary, further move center of mass by altering ankle/hip roll/pitch
    // TODO: Currently assume both feet are facing the same direction (if opposite is on the ground)
    if (dirToMove.getMagnitude() < EPSILON) {
        return true;
    }

    double footMass = component[legIndex == LEG_LEFT ? COMP_LFOOT : COMP_RFOOT].mass + component[legIndex == LEG_LEFT ? COMP_LANKLE : COMP_RANKLE].mass;
    // First pitch (x direction) then roll (y direction)
    double comMagX = hypot(centerOfMass.getX(), centerOfMass.getZ());
    double sinTheta = (((bodyMass / (bodyMass - footMass)) * dirToMove.getX()) / comMagX) + sin(Deg2Rad(a5));
    if (sinTheta > 1 || sinTheta < -1) {
        return false;
    } else {
        stab_pl5 = Rad2Deg(asin(sinTheta));
        stab_ol5 = stab_pl5;
    }

    bool oppositeOnGround = (legIndex == LEG_LEFT ? getFRPForceRight() : getFRPForceLeft()).getMagnitude() > 0.005;
    if (!oppositeOnGround) { // Yay! One foot on the ground makes this easy!
        double comMagY = hypot(centerOfMass.getY(), centerOfMass.getZ());
        sinTheta = -1 * (((bodyMass / (bodyMass - footMass)) * dirToMove.getY()) / comMagY) - sin(Deg2Rad(a6));
        if (sinTheta > 1 || sinTheta < -1) {
            return false;
        } else {
            stab_pl6 = Rad2Deg(asin(sinTheta));
        }
    } else { // Both feet being on the ground complicates the effect of ankle roll...

        // Note that this relies on the fact that hip1 and hip2 are at exactly the same point in space
        HCTMatrix footRelHipPlant = HCTMatrix(HCT_GENERALIZED_ROTATE, component[legIndex == LEG_LEFT ? COMP_LHIP1 : COMP_RHIP1].axis, joint[legIndex == LEG_LEFT ? EFF_LL1 : EFF_RL1].angle);
        footRelHipPlant.multiply(component[legIndex == LEG_LEFT ? COMP_LHIP1 : COMP_RHIP1].transformFromParent.getInverse()); // Cancel transform from torso to hip
        footRelHipPlant.multiply(footRelOrigin);
        VecPosition footFromHipPlant = footRelHipPlant.transform(VecPosition(0, 0, 0));
        double legLength = hypot(footFromHipPlant.getZ(), footFromHipPlant.getY()); // TODO: Assuming legs have same distance from hip to foot for now.

        double llMass, rlMass;
        VecPosition llCom = getCenterOfMass(LEG_LEFT, llMass);
        VecPosition rlCom = getCenterOfMass(LEG_RIGHT, rlMass);
        double llComMag = hypot(llCom.getZ(), llCom.getY());
        double rlComMag = hypot(rlCom.getZ(), rlCom.getY());

        sinTheta = ((dirToMove.getY() * bodyMass) / (legLength * (bodyMass - llMass - rlMass) + llComMag * llMass + rlComMag * rlMass));// + sin(a6);
        if (sinTheta < -1 || sinTheta > 1) {
            return false;
        } else {
            stab_pl6 = Rad2Deg(asin(sinTheta)) + a6;
            stab_ol6 = stab_pl6;
            stab_pl2 = -stab_pl6;
            stab_ol2 = -stab_pl6;
        }
    }

    return true;
}

void BodyModel::applyStabilizationArm(const int armIndex, double &stab_a1, double &stab_a2, VecPosition dirToMove, double bodyMass) const {
    // TODO: Need to account for torso's pitch once it is known (This applies for stabilization in general actually)
    if (armIndex != ARM_LEFT && armIndex != ARM_RIGHT) {
        cerr << "Received invalid arm index: " << armIndex << endl;
        return;
    }
    bool l = armIndex == ARM_LEFT;
    double armMass= 0;
    VecPosition armComRelShoulder = getCenterOfMass(armIndex, armMass);
    double initialMag = dirToMove.getMagnitude();
    dirToMove.normalize();
    stab_a1 = Rad2Deg(atan2(dirToMove.getX(), dirToMove.getY())) - 90;
    stab_a1 = stab_a1 > 180 ? stab_a1 - 360 : (stab_a1 < -180 ? stab_a1 + 360 : stab_a1);
    dirToMove *= initialMag * bodyMass / armMass;
    double sinTheta = 0;
    if (dirToMove.getX() == 0) {
        sinTheta = dirToMove.getY() / armComRelShoulder.getMagnitude();
    } else {
        sinTheta = dirToMove.getY() / (armComRelShoulder.getMagnitude() * cos(Deg2Rad(stab_a1)));
    }
    if (sinTheta > 1 || sinTheta < -1) { // Arms alone will not solve stabilization
        stab_a2 = ((sinTheta > 1) == (dirToMove.getX() > 0)) ? (effector[l ? EFF_LA2 : EFF_RA2].maxAngle + (l ? -5 : 0)) : (effector[l ? EFF_LA2 : EFF_RA2].minAngle  + (l ? 0 : 5));
    } else {
        stab_a2 = Rad2Deg(asin(sinTheta));
    }
    stab_a1 = max(min(stab_a1, effector[l ? EFF_LA1 : EFF_RA1].maxAngle), effector[l ? EFF_LA1 : EFF_RA1].minAngle);
    stab_a1 = (joint[l ? EFF_LA1 : EFF_RA1].angle > 0 ? 1 : -1) * abs(stab_a1); // Avoid large unnecessary changes
    stab_a2 = max(min(stab_a2, effector[l ? EFF_LA2 : EFF_RA2].maxAngle), effector[l ? EFF_LA2 : EFF_RA2].minAngle);
}

VecPosition BodyModel::transformCameraToOrigin(const VecPosition &v) {

    return (component[COMP_HEAD].transformFromRoot).transform(v);
}

/*
 * send reference variables to be set
 */
void BodyModel::
getReflection( const int& effID,
               const double& angle,
               int& reflectedEffID,
               double& reflectedAngle ) {

    reflectedEffID = reflectedEffector[effID];
    reflectedAngle = toReflectAngle[effID] ? -angle : angle;

}

/*
 * send reference variables to be set
 */
void BodyModel::
getReflection( const int& legIDX,
               const Pos6DOF& pos,
               int& reflectedLegIDX,
               Pos6DOF& reflectedPos ) {
    reflectedLegIDX = legIDX == LEG_LEFT ? LEG_RIGHT : LEG_LEFT;
    reflectedPos = pos.reflect();
}

bool BodyModel::setInitialHead() {

    bool reached;

    setTargetAngle(EFF_H1, 0);
    setTargetAngle(EFF_H2, -45);

    reached = getTargetReached(EFF_H1) && getTargetReached(EFF_H2);
    return reached;
}

bool BodyModel::setInitialArm(const int &arm) {

    bool reached;

    double ang1, ang2, ang3, ang4;

    ang1 = -50.0;
    ang2 = 30.0;
    ang3 = 0;
    ang4 = -50.0;

    if(arm == ARM_LEFT) {

        setTargetAngle(EFF_LA1, ang1);
        setTargetAngle(EFF_LA2, ang2);
        setTargetAngle(EFF_LA3, ang3);
        setTargetAngle(EFF_LA4, ang4);

        reached = getTargetReached(EFF_LA1) && getTargetReached(EFF_LA2) && getTargetReached(EFF_LA3) && getTargetReached(EFF_LA4);

    }
    else { //ARM_RIGHT

        setTargetAngle(EFF_RA1, ang1);
        setTargetAngle(EFF_RA2, -ang2);
        setTargetAngle(EFF_RA3, -ang3);
        setTargetAngle(EFF_RA4, -ang4);

        reached = getTargetReached(EFF_RA1) && getTargetReached(EFF_RA2) && getTargetReached(EFF_RA3) && getTargetReached(EFF_RA4);

    }

    return reached;
}

bool BodyModel::setInitialLeg(const int &leg) {

    bool reached;

    double ang1 = 0;
    double ang2 = 6.0;
    double ang3 = 18.0;
    double ang4 = -30.0;
    double ang5 = 18.0;
    double ang6 = -6.0;
    double ang7 = 0;

    if(leg == LEG_LEFT) {

        setTargetAngle(EFF_LL1, ang1);
        setTargetAngle(EFF_LL2, ang2);
        setTargetAngle(EFF_LL3, ang3);
        setTargetAngle(EFF_LL4, ang4);
        setTargetAngle(EFF_LL5, ang5);
        setTargetAngle(EFF_LL6, ang6);
        setTargetAngle(EFF_LL7, ang7);

        reached = getTargetReached(EFF_LL1) && getTargetReached(EFF_LL2) && getTargetReached(EFF_LL3) && getTargetReached(EFF_LL4) && getTargetReached(EFF_LL5) && getTargetReached(EFF_LL6) && (!hasToe() || getTargetReached(EFF_LL7)) ;

    }
    else {

        setTargetAngle(EFF_RL1, ang1);
        setTargetAngle(EFF_RL2, -ang2);
        setTargetAngle(EFF_RL3, ang3);
        setTargetAngle(EFF_RL4, ang4);
        setTargetAngle(EFF_RL5, ang5);
        setTargetAngle(EFF_RL6, -ang6);
        setTargetAngle(EFF_RL7, ang7);

        reached = getTargetReached(EFF_RL1) && getTargetReached(EFF_RL2) && getTargetReached(EFF_RL3) && getTargetReached(EFF_RL4) && getTargetReached(EFF_RL5) && getTargetReached(EFF_RL6) && (!hasToe() || getTargetReached(EFF_RL7));
    }

    return reached;
}

VecPosition BodyModel::getCenterOfMass() const {
    VecPosition com = VecPosition(0, 0, 0);
    double totalMass = 0;
    for (unsigned i = 0; i < COMP_NUM; i++) {
        Component bodyPart = component[i];
        double mass = bodyPart.mass;
        VecPosition location = bodyPart.transformFromRoot.transform(VecPosition(0, 0, 0));
        com += location * mass;
        totalMass += mass;
    }
    com /= totalMass;
    return com;
}

/**
 * Find the center of mass of an arm relative to its shoulder, or the leg relative to a foot
 * NOTE: The behavior for arms is different from the behavior for legs!
 */
VecPosition BodyModel::getCenterOfMass(int bodyPart, double &totalMass) const {
    VecPosition com = VecPosition(0, 0, 0);
    totalMass = 0;
    HCTMatrix transformFromPartRoot = HCTMatrix();
    if (bodyPart == ARM_LEFT) {
        transformFromPartRoot.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_LSHOULDER].axis, joint[EFF_LA1].angle));
        transformFromPartRoot.multiply(component[COMP_LUPPERARM].transformFromParent);
        com += (transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LUPPERARM].mass);
        totalMass += component[COMP_LUPPERARM].mass;

        transformFromPartRoot.multiply(component[COMP_LELBOW].transformFromParent);
        com += (transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LELBOW].mass);
        totalMass += component[COMP_LELBOW].mass;

        transformFromPartRoot.multiply(component[COMP_LLOWERARM].transformFromParent);
        com += (transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LLOWERARM].mass);
        totalMass += component[COMP_LLOWERARM].mass;
    } else if (bodyPart == ARM_RIGHT) {
        transformFromPartRoot.multiply(HCTMatrix(HCT_GENERALIZED_ROTATE, component[COMP_RSHOULDER].axis, joint[EFF_RA1].angle));
        transformFromPartRoot.multiply(component[COMP_RUPPERARM].transformFromParent);
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RUPPERARM].mass;
        totalMass += component[COMP_RUPPERARM].mass;

        transformFromPartRoot.multiply(component[COMP_RELBOW].transformFromParent);
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RELBOW].mass;
        totalMass += component[COMP_RELBOW].mass;

        transformFromPartRoot.multiply(component[COMP_RLOWERARM].transformFromParent);
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RLOWERARM].mass;
        totalMass += component[COMP_RLOWERARM].mass;
    } else if (bodyPart == LEG_LEFT) {
        transformFromPartRoot.multiply(component[COMP_LFOOT].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LANKLE].mass;
        totalMass += component[COMP_LANKLE].mass;

        transformFromPartRoot.multiply(component[COMP_LANKLE].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LSHANK].mass;
        totalMass += component[COMP_LSHANK].mass;

        transformFromPartRoot.multiply(component[COMP_LSHANK].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LTHIGH].mass;
        totalMass += component[COMP_LTHIGH].mass;

        transformFromPartRoot.multiply(component[COMP_LTHIGH].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LHIP2].mass;
        totalMass += component[COMP_LHIP2].mass;

        transformFromPartRoot.multiply(component[COMP_LHIP2].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_LHIP1].mass;
        totalMass += component[COMP_LHIP1].mass;
    } else if (bodyPart == LEG_RIGHT) {
        transformFromPartRoot.multiply(component[COMP_RFOOT].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RANKLE].mass;
        totalMass += component[COMP_RANKLE].mass;

        transformFromPartRoot.multiply(component[COMP_RANKLE].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RSHANK].mass;
        totalMass += component[COMP_RSHANK].mass;

        transformFromPartRoot.multiply(component[COMP_RSHANK].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RTHIGH].mass;
        totalMass += component[COMP_RTHIGH].mass;

        transformFromPartRoot.multiply(component[COMP_RTHIGH].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RHIP2].mass;
        totalMass += component[COMP_RHIP2].mass;

        transformFromPartRoot.multiply(component[COMP_RHIP2].transformFromParent.getInverse());
        com += transformFromPartRoot.transform(VecPosition(0, 0, 0)) * component[COMP_RHIP1].mass;
        totalMass += component[COMP_RHIP1].mass;
    } else {
        // We don't need support for the head at the moment
        return VecPosition(0, 0, 0);
    }
    com /= totalMass;
    return com;
}

bool BodyModel::hasToe() {
    return agentBodyType == 4;
}

bool BodyModel::isGazebo() {
    return agentBodyType == GAZEBO_AGENT_TYPE;
}

/**
 * Reflect a 6DOF foot pose between left and right foot.
 *
 * Specifically, negates y-axis, roll, and yaw.
 */
Pos6DOF
Pos6DOF::reflect() const {
    Pos6DOF reflected(*this);

    reflected.xyz *= VecPosition(1, -1, 1);
    reflected.rpy *= VecPosition(-1, 1, -1);

    return reflected;
}
std::ostream& operator<<(std::ostream &out, const Pos6DOF &pos) {
    return out << pos.xyz << " " << pos.rpy;
}
