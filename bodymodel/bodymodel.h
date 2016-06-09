#ifndef BODY_MODEL_H
#define BODY_MODEL_H

#include "../headers/headers.h"
#include "../math/vecposition.h"
#include "../math/hctmatrix.h"
#include "../worldmodel/worldmodel.h"
#include "../ikfast/ikfast.h"

#include <sstream>

// For UT Walk
class SimEffectorBlock;
class JointBlock;
class JointCommandBlock;

struct SIMJoint {

    double angle;

    SIMJoint() {
        angle = 0;
    }
};

/**
 * Pos6DOF represents a 6 degree-of-freedom pose.
 */
struct Pos6DOF
{
    VecPosition xyz;      //< The XYZ component of the pose
    VecPosition rpy;      //< The roll-pitch-yaw component of the pose

    Pos6DOF reflect() const;
};

std::ostream& operator<<(std::ostream &out, const Pos6DOF &pos);

struct Effector {

    double minAngle;
    double maxAngle;

    double currentAngle;
    double targetAngle;

    // Constants for PID control
    double k1, k2, k3;

    // Error terms
    double currentError;// corr. k1
    double cumulativeError;// corr. k2
    double previousError;// corr. k3

    // set point tolerance
    double errorTolerance;

    double scale;

    Effector(const double &minAngle, const double &maxAngle, const double &k1, const double &k2, const double &k3, const double &errorTolerance) {

        this->minAngle = minAngle;
        this->maxAngle = maxAngle;

        currentAngle = 0;
        targetAngle = 0;

        this->k1 = k1;
        this->k2 = k2;
        this->k3 = k3;

        resetErrors();
        this->errorTolerance = errorTolerance;

        scale = 1.0;

    }

    void resetErrors() {
        currentError = 0;
        previousError = 0;
        cumulativeError = 0;
    }

    void updateErrors() {

        previousError = currentError;
        currentError = targetAngle - currentAngle;
        cumulativeError += currentError;

    }

    void setTargetAngle(const double &angle) {

        if(angle < minAngle) {
            targetAngle = minAngle;
        }
        else if(angle > maxAngle) {
            targetAngle = maxAngle;
        }
        else {
            targetAngle = angle;
        }

    }

    void update(const double &angle) {

        currentAngle = angle;
    }

    bool targetReached() {

        return (fabs(targetAngle - currentAngle) < errorTolerance);
    }

    bool isAngleInRange(const double &angle) const {
        return (minAngle <= angle) && (angle <= maxAngle);
    }
};

struct Component {

    int parent;

    double mass;
    VecPosition translation;
    VecPosition anchor;
    VecPosition axis;

    HCTMatrix transformFromParent;
    HCTMatrix transformFromRoot;

    HCTMatrix translateMatrix;
    HCTMatrix backTranslateMatrix;

    Component(const int &parent, const double &mass, const VecPosition &translation, const VecPosition &anchor, const VecPosition &axis) {

        this->parent = parent;
        this->mass = mass;

        this->translation = translation;
        this->anchor = anchor;
        this->axis = axis;

        transformFromParent = HCTMatrix();
        transformFromRoot = HCTMatrix();

        translateMatrix = HCTMatrix(HCT_TRANSLATE, VecPosition(translation) + VecPosition(anchor));
        backTranslateMatrix = HCTMatrix(HCT_TRANSLATE, -VecPosition(anchor));
    }
};


class BodyModel {

private:

    WorldModel *worldModel;

    // Joints
    std::vector<SIMJoint> joint;

    // Effectors
    std::vector<Effector> effector;

    // Reflectors
    std::vector<int> reflectedEffector;
    std::vector<bool> toReflectAngle;

    // Components
    std::vector<Component> component;

    VecPosition gyroRates;
    VecPosition accelRates;
    VecPosition FRPCentreLeft;
    VecPosition FRPForceLeft;
    VecPosition FRPCentreRight;
    VecPosition FRPForceRight;
    VecPosition FRPCentreLeft1;
    VecPosition FRPForceLeft1;
    VecPosition FRPCentreRight1;
    VecPosition FRPForceRight1;

    HCTMatrix bodyWorldInterface;

    HCTMatrix ll1, ll2, ll3, ll4, ll5;
    HCTMatrix rl1, rl2, rl3, rl4, rl5;

    double fallAngle;

    bool fUseOmniWalk;


    void refreshTorso();
    void refreshComponent(const int &index);
    void refreshHead();
    void refreshLeftArm();
    void refreshRightArm();
    void refreshLeftLeg();
    void refreshRightLeg();

    inline void computeFallAngle() {

        VecPosition zAxis = VecPosition(0, 0, 1.0);
        VecPosition bodyZAxis = (worldModel->l2g(VecPosition(0, 0, 1.0)) - worldModel->l2g(VecPosition(0, 0, 0))).normalize();
        fallAngle = VecPosition(0, 0, 0).getAngleBetweenPoints(zAxis, bodyZAxis);
    }

public:

    BodyModel(WorldModel *worldModel);
    BodyModel(const BodyModel *current, const int legIndex, const double a1, const double a2, const double a3, const double a4, const double a5, const double a6);
    ~BodyModel();

    void initialiseEffectors();
    void initialiseComponents();

    inline double getJointAngle(const int &i) {
        return joint[i].angle;
    }
    inline void setJointAngle(const int &i, const double &a) {
        joint[i].angle = a;
    }

    void display();
    void displayDerived();

    inline double getCurrentAngle(const int &EffectorID) const {
        return effector[EffectorID].currentAngle;
    }
    inline void setCurrentAngle(const int &EffectorID, const double &angle) {
        effector[EffectorID].currentAngle = angle;
    }

    inline double getTargetAngle(const int &EffectorID) const {
        return effector[EffectorID].targetAngle;
    }
    inline void setTargetAngle(const int &EffectorID, const double &angle) {
        effector[EffectorID].setTargetAngle(angle);
    }
    inline void increaseTargetAngle(const int &EffectorID, const double &increase) {
        effector[EffectorID].setTargetAngle(effector[EffectorID].targetAngle + increase);
    };

    inline double getScale(const int &EffectorID) {
        return effector[EffectorID].scale;
    }
    inline void setScale(const int &EffectorID, double s) {
        effector[EffectorID].scale = s;
    }

    inline bool getTargetReached(const int &EffectorID) {
        return effector[EffectorID].targetReached();
    }

    double computeTorque(const int &effectorID);

    // For UT Walk
    double computeTorque(const int &effID, SimEffectorBlock* sim_effectors_, JointBlock* raw_joint_angles_, JointCommandBlock* raw_joint_commands_);
    inline void setUseOmniWalk(bool fUseOmniWalk) {
        this->fUseOmniWalk = fUseOmniWalk;
    }
    inline bool useOmniWalk() {
        return fUseOmniWalk;
    }

    inline VecPosition getGyroRates() {
        return gyroRates;
    }
    inline void setGyroRates(const double &rateX, const double &rateY, const double &rateZ) {
        gyroRates = VecPosition(rateX, rateY, rateZ);
    }

    inline VecPosition getAccelRates() {
        return accelRates;
    }
    inline void setAccelRates(const double &rateX, const double &rateY, const double &rateZ) {
        accelRates = VecPosition(rateX, rateY, rateZ);
    }


    inline VecPosition getFRPCentreLeft() const {
        return FRPCentreLeft;
    }
    inline VecPosition getFRPForceLeft() const {
        return FRPForceLeft;
    }
    inline VecPosition getFRPCentreRight() const {
        return FRPCentreRight;
    }
    inline VecPosition getFRPForceRight() const {
        return FRPForceRight;
    }
    inline VecPosition getFRPCentreLeft1() const {
        return FRPCentreLeft1;
    }
    inline VecPosition getFRPForceLeft1() const {
        return FRPForceLeft1;
    }
    inline VecPosition getFRPCentreRight1() const {
        return FRPCentreRight1;
    }
    inline VecPosition getFRPForceRight1() const {
        return FRPForceRight1;
    }
    inline void setFRPLeft(const VecPosition &centre, const VecPosition &force) {
        FRPCentreLeft = VecPosition(centre);
        FRPForceLeft = VecPosition(force);
    }
    inline void setFRPRight(const VecPosition &centre, const VecPosition &force) {
        FRPCentreRight = VecPosition(centre);
        FRPForceRight = VecPosition(force);
    }
    inline void setFRPLeft1(const VecPosition &centre, const VecPosition &force) {
        FRPCentreLeft1 = VecPosition(centre);
        FRPForceLeft1 = VecPosition(force);
    }
    inline void setFRPRight1(const VecPosition &centre, const VecPosition &force) {
        FRPCentreRight1 = VecPosition(centre);
        FRPForceRight1 = VecPosition(force);
    }

    void refresh();

    bool canReachOutLeg(const int &legIndex, const VecPosition &xyz, const double &yaw) const;
    bool canReachOutLeg(const int &legIndex, const VecPosition &xyz, const VecPosition &rpy = VecPosition(0.0, 0.0, 0.0)) const;
    bool reachOutLeg(const int &legIndex, const VecPosition &xyz, const double &yaw);
    bool reachOutLeg(const int &legIndex, const VecPosition &xyz, const VecPosition &rpy = VecPosition(0.0, 0.0, 0.0));
    bool targetsReached();

    VecPosition getFootCG(const int &legIndex, const double &ang1, const double &ang2, const double &ang3, const double &ang4, const double &ang5, const double &ang6);

    VecPosition getFootCG(const int &legIndex) const;

    void getFootTransform(const int &legIndex, HCTMatrix &m, const double &ang1, const double &ang2, const double &ang3, const double &ang4, const double &ang5, const double &ang6) const;

    void getFootGlobalCGAndAxes(const int &legIndex, VecPosition &CG, VecPosition &xAxis, VecPosition &yAxis, VecPosition &zAxis);
    void getFootGlobalCGAndAxesAngles(const int &legIndex, const vector<double> &angles, VecPosition &origin, VecPosition &xAxis, VecPosition &yAxis, VecPosition &zAxis);

    VecPosition getBallWRTLeftFoot(WorldModel *worldModel);
    VecPosition getBallWRTRightFoot(WorldModel *worldModel);


    bool legInverseKinematics(const int &legIndex, const VecPosition &xyz, const VecPosition &rpy, double &a1, double &a2, double &a3, double &a4, double &a5, double &a6) const;
    bool stabilize(const int legIndex, const VecPosition zmp);
    bool canStabilize(const int legIndex, const VecPosition zmp) const;
    bool applyStabilization(const int &legIndex, const VecPosition zmp, double &stab_ra1, double &stab_ra2, double &stab_la1, double &stab_la2, double &stab_pl2, double &stab_pl5, double &stab_pl6, double &stab_ol2, double &stab_ol5, double &stab_ol6) const;
    void applyStabilizationArm(const int armIndex, double &stab_shoulderY, double &stab_shoulderZ, VecPosition dirToMove, double bodyMass) const;

    inline double getFallAngle() {
        return fallAngle;
    }

    VecPosition transformCameraToOrigin(const VecPosition &v);

    void getReflection( const int& effID,
                        const double& angle,
                        int& reflectedEffID,
                        double& reflectedAngle );

    void getReflection( const int& legIDX,
                        const Pos6DOF& pos,
                        int& reflectedLegIDX,
                        Pos6DOF& reflectedPos );

    VecPosition getCenterOfMass() const;
    VecPosition getCenterOfMass(int bodyPart, double &totalMass) const;


    // moved from NaoBehavior
    bool setInitialHead();
    bool setInitialArm(const int &arm);
    bool setInitialLeg(const int &leg);

    bool hasToe();
    bool isGazebo();

protected:
    void filterOutOfBounds(const int &legIndex, vector<joints_t> &solutions) const;
    const joints_t& pickBestSolution(const int &legIndex, vector<joints_t> &solutions) const;
    bool isInvalidSolution(const int &legIndex, const joints_t &solution) const;
};

#endif // BODY_MODEL_H


