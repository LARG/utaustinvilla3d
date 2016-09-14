#ifndef SKILL_H
#define SKILL_H

#include <boost/shared_ptr.hpp>
#include "../bodymodel/bodymodel.h"
#include "curve3d.h"

using namespace boost;

std::ostream& operator<<(std::ostream &out, const Pos6DOF &pos);

/////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////
class Macro {
private:
protected:
public:
    Macro() {}
    virtual ~Macro() {}

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel) = 0;
    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel) = 0;
    virtual void display() = 0;

    virtual bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
        // TODO: make this method pure-virtual and force all subclasses to override
        return true;
    }
};

class IncTar : public Macro {

private:
protected:

    // These two vectors define a mapping between effector->increment
    vector<int> effectorIDs;
    vector<double> increments;
//    int effID;
//    double increment;

public:

    IncTar(const vector<int> &effectorIDs_, const vector<double> &increments_);
    ~IncTar() {}

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};

class SetTar : public Macro {

private:
protected:

    // These two vectors define a mapping between effector->increment
    vector<int> effectorIDs;
    vector<double> targetAngles;
//    int effID;
//    double increment;

public:

    SetTar(const vector<int> &effectorIDs_, const vector<double> &targetAngles_);
    ~SetTar() {}

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};

/**
 * SetFoot is a Macro which sets a foot at a specified 6DOF pose.
 * The XYZ component of the pose is relative to the ball.
 * The RPY component is in absolute degrees.
 */
class SetFoot: public Macro {

private:
protected:

    // These define the leg and the foot position
    const int legIDX;             //< LEG_LEFT or LEG_RIGHT
    const Pos6DOF targetPos;      //< Desired XYZ offset from ball and absolute RPY angles.

public:

    SetFoot(const int &legIDX_, const Pos6DOF &targetPos_);
    ~SetFoot() {
    }

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const;

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};

/**
 * A Curve is a Macro describing a curve that a foot is to follow.
 *
 * The curve is defined by a sequence of control points in XYZ and RPY.
 *
 * The first XYZ control point is relative to the ball's position when the macro first
 * begins executing. The following XYZ control points
 * are relative to the first control point. Essentially, the whole curve is relative to the
 * ball's starting position, but this way of parameterizing it seemed more convenient.
 *
 * All RPY control points are absolute roll, pitch, and yaw Euler angles defining the foot orientation.
 */
class Curve : public Macro {

private:
protected:

    const int legIDX;
    boost::shared_ptr<Curve3D> curve;              // XYZ curve, relative to the ball
    boost::shared_ptr<Curve3D> rpy_curve;          // RPY curve
    VecPosition curveOffsetWrtTorso;
    const VecPosition curveOffsetWrtBall;
    float t;

public:

    Curve(const int &legIdx_, const vector<Pos6DOF> &controlPoints_);
    virtual ~Curve();

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);
    virtual bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const;
    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);
    virtual void display();
    virtual void setTimeParam(const float & t_);
};

class Reset : public Macro {

private:
protected:
    vector<int> components;
public:

    Reset(const std::vector<int>& components_);
    ~Reset() {}

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};

class SetScale : public Macro {

private:
protected:

    // These two vectors define a mapping between effector->increment
    vector<int> effectorIDs;
    vector<double> targetScales;
//    int effID;
//    double increment;

public:

    SetScale(const vector<int> &effectorIDs_, const vector<double> &targetScales_);
    ~SetScale() {}

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};

/**
 * Stabilize is a Macro which shifts the center of mass over the given foot.
 */
class Stabilize: public Macro {

private:
protected:
    const int legIndex;             //< LEG_LEFT or LEG_RIGHT
    VecPosition zmp;		   //Position over which to balance relative to the foot

public:
    Stabilize(const int &legIndex, const VecPosition zmp);
    ~Stabilize() {
    }

    virtual void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    virtual bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const;

    virtual boost::shared_ptr<Macro> getReflection(BodyModel *bodyModel);

    virtual void display();
};



/////////////////////////////////////////////////////
// KeyFrames
/////////////////////////////////////////////////////

class KeyFrame {

private:
protected:
    vector< boost::shared_ptr<Macro> > macros;

    bool toWaitTime; // Default: false.
    double waitTime; // Default: 0.

    bool toWaitTargets; // Default: false.
    double maxWaitTime; // Default: INF.

    void updateCurveMacros(BodyModel *bodyModel, const WorldModel *worldModel, const double & t);

public:

    KeyFrame();
    ~KeyFrame() {};

    void appendMacro(boost::shared_ptr<Macro> macro);
    void execute(BodyModel *bodyModel, const WorldModel *worldModel); // Mainly a set.

    // Setting values to specific data members.
    // We want to be able to manually set any subset of them.
    void setToWaitTime( bool value );
    void setWaitTime( double value );
    void setToWaitTargets( bool value );
    void setMaxWaitTime( double value );

    bool done(BodyModel *bodyModel, const WorldModel *worldModel, const double &setTime);
    boost::shared_ptr<KeyFrame> getReflection(BodyModel *bodyModel);

    virtual bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const;

    void display();

};



///////////////////////////////////////////////////////
// Skill
///////////////////////////////////////////////////////
class Skill {

private:
protected:
    vector< boost::shared_ptr<KeyFrame> > keyFrames;
    int currentKeyFrame;
    bool currentKeyFrameSet;
    double currentKeyFrameSetTime;

public:

    Skill();
    ~Skill() {};

    void reset();

    bool done(BodyModel *bodyModel, const WorldModel *worldModel);

    void execute(BodyModel *bodyModel, const WorldModel *worldModel);

    bool canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const;

    void appendKeyFrame( boost::shared_ptr<KeyFrame> keyFrame );

    boost::shared_ptr<Skill> getReflection(BodyModel *bodyModel);

    void display();

    int getCurrentKeyFrame();
};


#endif

