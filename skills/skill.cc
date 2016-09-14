#include <boost/foreach.hpp>
#include "skill.h"
#include "../ikfast/ikfast.h"

#define foreach         BOOST_FOREACH

/////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////

// IncTar
// -------
IncTar::IncTar(const vector<int> &effectorIDs_, const vector<double> &increments_) :
    effectorIDs(effectorIDs_), increments(increments_) {
    // assuming two vectors are the same length
}


void IncTar::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    // assuming two vectors are the same length
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        bodyModel->increaseTargetAngle( effectorIDs[i], increments[i] );
    }
}

boost::shared_ptr<Macro>
IncTar::getReflection(BodyModel *bodyModel) {

    vector<int> newEffIDs;
    vector<double> newIncs;

    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        int newEffID;
        double newIncrement;
        bodyModel->getReflection( effectorIDs[i],
                                  increments[i],
                                  newEffID,
                                  newIncrement );
        newEffIDs.push_back( newEffID );
        newIncs.push_back( newIncrement );
    }

    return boost::shared_ptr<Macro>( new IncTar(newEffIDs, newIncs) );
}

void IncTar::display() {
    cerr << "Displaying macro: IncTar" << endl;
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        cerr << "effID[" << i << "]=" << effectorIDs[i]
             << " increment[" << i << "]=" << increments[i] << endl;
    }
}

// SetTar
// -------
SetTar::SetTar(const vector<int> &effectorIDs_, const vector<double> &targetAngles_) :
    effectorIDs(effectorIDs_), targetAngles(targetAngles_) {
    // assuming two vectors are the same length
}


void SetTar::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    // assuming two vectors are the same length
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        bodyModel->setTargetAngle( effectorIDs[i], targetAngles[i] );
    }
}

boost::shared_ptr<Macro>
SetTar::getReflection(BodyModel *bodyModel) {

    vector<int> newEffIDs;
    vector<double> newAngs;

    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        int newEffID;
        double newAngle;
        bodyModel->getReflection( effectorIDs[i],
                                  targetAngles[i],
                                  newEffID,
                                  newAngle );
        newEffIDs.push_back( newEffID );
        newAngs.push_back( newAngle );
    }

    return boost::shared_ptr<Macro>( new SetTar(newEffIDs, newAngs) );
}

void SetTar::display() {
    cerr << "Displaying macro: SetTar" << endl;
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        cerr << "effID[" << i << "]=" << effectorIDs[i]
             << " Angle[" << i << "]=" << targetAngles[i] << endl;
    }
}

// SetFoot
// -------
/**
 * Constructor.
 *
 * \param legIDX_     LEG_LEFT or LEG_RIGHT
 * \param targetPos_  The desired pose of the foot. XYZ component is an offset from the ball;
 *                    RPY component is absolute Euler angles.
 */
SetFoot::SetFoot(const int &legIDX_, const Pos6DOF &targetPos_) :
    legIDX(legIDX_), targetPos(targetPos_) {
    assert(LEG_LEFT == legIDX || LEG_RIGHT == legIDX);
}

/**
 * Reaches out the leg so that the foot gets to the target pose (if possible).
 *
 * \param bodyModel a BodyModel
 * \param worldModel a WorldModel
 */
void SetFoot::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    if (!bodyModel->reachOutLeg(legIDX, worldModel->getBallWrtTorso() + targetPos.xyz, targetPos.rpy)) {
        //LOG("setfoot failed...");
    }
}

/**
 * Checks if the SetFoot Macro can be executed given a bodyModel and worldModel.
 * Accomplished by checking if the bodyModel can reach out the leg to the desired pose.
 *
 * \param bodyModel a BodyModel
 * \param worldModel a WorldModel.
 * \return true iff the bodyModel can reach out the leg
 */
bool SetFoot::canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
    bool ret = bodyModel->canReachOutLeg(legIDX, worldModel->getBallWrtTorso() + targetPos.xyz, targetPos.rpy);
    if(!ret) {
        //LOG_MSG("Can't execute SetFoot", targetPos);
    }
    return ret;
}

/**
 * Reflets the Macro from between left and right foot.
 *
 * \bodyModel a BodyModel
 * \return a boost::shared_ptr to a left/right reflected version of this SetFoot Macro.
 */
boost::shared_ptr<Macro> SetFoot::getReflection(BodyModel *bodyModel) {

    int newLegIDX;
    Pos6DOF newPos;

    bodyModel->getReflection(legIDX, targetPos, newLegIDX, newPos);

    return boost::shared_ptr<Macro> (new SetFoot(newLegIDX, newPos));
}

/**
 * Displays the macro's target foot and target pose.
 */
void SetFoot::display() {
    cerr << "Displaying macro: SetFoot" << endl;
    cerr << "legIDX=" << legIDX << " Pos6DOF=" << targetPos << endl;
}

// Curve
// ------
Curve::Curve(const int &legIDX_, const vector<Pos6DOF> &controlPoints_) :
    legIDX(legIDX_),
    curveOffsetWrtBall(controlPoints_[0].xyz),    // first control point is relative to the ball
    // the rest of the control points are relative to the first
    t(0.0f) {
    vector<VecPosition> xyz;
    vector<VecPosition> rpy;
    foreach( const Pos6DOF &cp, controlPoints_ ) {
        xyz.push_back(cp.xyz);
        rpy.push_back(cp.rpy);
    }
    xyz[0].setVecPosition(0, 0, 0);
    curve.reset(new HermiteSpline3D(xyz, .8f));
    rpy_curve.reset(new UniformBSpline3D(rpy));
//  rpy_curve.reset(new HermiteSpline3D(rpy, 1));
//  LOG(rpy);
}

Curve::~Curve() {
}

void Curve::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
//#ifdef DRAW_CURVE_TO_RVIZ
    /* draws out curve to RoboViz
    float step = (1.0f / 50);
    VecPosition st;
    char buf[512];
    sprintf(buf, "optimization.curve");
    string str = string(buf);

    for( float i = 0; i < 1 - step; i += step) {
      if( i == 0 )
      st = curve->getPoint(i) + worldModel->l2g(curveOffsetWrtTorso);

      VecPosition en = worldModel->l2g(curve->getPoint(i + step)) + curveOffsetWrtTorso;
      worldModel->getRVSender()->drawLine( st.getX(), st.getY(), st.getZ(),
                          en.getX(), en.getY(), en.getZ(), 2, 1, 0, 0, &str);
      st = en;
    }
    worldModel->getRVSender()->swapBuffers(&str);
    //end draw out to roboviz*/
//#endif // DRAW_CURVE_TO_RVIZ

    if(t < .8) {
        // When the Macro first starts executing, find the current offset of the
        // ball relative to the torso, apply the curve offset relative to the ball,
        // and set that as the offset of the curve relative to the torso
        VecPosition ballWrtTorso = worldModel->getBallWrtTorso();
        curveOffsetWrtTorso = ballWrtTorso + curveOffsetWrtBall;

        //LOG(ballWrtTorso);
        //LOG(curveOffsetWrtBall);
        //LOG(curveOffsetWrtTorso);
    }

    VecPosition pos = curve->getPoint(t);
    VecPosition rpy = rpy_curve->getPoint(t);

//  LOG(pos);
//  LOG(rpy);

    if(!bodyModel->reachOutLeg(legIDX, pos + curveOffsetWrtTorso, rpy)) {
        //LOG(pos);
    }
}

bool Curve::canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
    return true;

    /*
      bool ret = true;

      VecPosition curveOffsetWrtTorso = worldModel->getBallWrtTorso() + curveOffsetWrtBall;

      const vector<VecPosition> &xyzCtrlPoints = curve->getControlPoints();
      const vector<VecPosition> &rpyCtrlPoints = rpy_curve->getControlPoints();

      double threshold = .5;
      // if you can't reach the ball, increase threshold for precise kicks
      if( !bodyModel->canReachOutLeg(legIDX, worldModel->getBallWrtTorso(), 0 ) ) {
        threshold = .5;
      }

      for(size_t i=0; i<xyzCtrlPoints.size(); ++i) {
        if(!bodyModel->canReachOutLeg(legIDX, xyzCtrlPoints[i] + curveOffsetWrtTorso, rpyCtrlPoints[i])) {
          //LOG_MSG("Can't reach Curve control point",i);
          //LOG(legIDX);
          //LOG(xyzCtrlPoints[i]);
          //LOG(curveOffsetWrtTorso);
          //LOG(xyzCtrlPoints[i] + curveOffsetWrtTorso);
          //LOG(rpyCtrlPoints[i]);
          return false;
        }
      }
      double tot = 20;
      double pass = 0;

      for( double u = 0; u < 1; u += (1.0/tot) ) {
         VecPosition pos = curve->getPoint(u);
         VecPosition rpy = rpy_curve->getPoint(u);
         if(bodyModel->canReachOutLeg(legIDX, pos + curveOffsetWrtTorso, rpy)) {
           ++pass;
         }
      }

      int teamAgent = worldModel->getUNum() - 1;
      //worldModel->getRVSender()->drawAgentAnnotation(sprintf("%f",(pass/tot)), (char)(teamAgent), 1, 0, 0);

      char buf[512];
      sprintf(buf, "%f", pass/tot);
      string str = string(buf);
      worldModel->getRVSender()->drawAgentAnnotation(&str, (char)(teamAgent), 1, 0, 0);

      if( pass / tot < threshold ) {
        return false;
      }
      return true;
    */
}

boost::shared_ptr<Macro> Curve::getReflection(BodyModel *bodyModel) {
    int newLegIDX = -1;

    vector<VecPosition> controlPoints = curve->getControlPoints();
    controlPoints[0] = curveOffsetWrtBall;
    const vector<VecPosition> &rpyControlPoints = rpy_curve->getControlPoints();
    vector<Pos6DOF> newControlPoints(controlPoints.size());

    for (size_t i = 0; i < controlPoints.size(); ++i) {
        Pos6DOF targetPos = {controlPoints[i], rpyControlPoints[i]};
        Pos6DOF newPos;

        bodyModel->getReflection(legIDX, targetPos, newLegIDX, newPos);
        newControlPoints[i] = newPos;
    }

    return boost::shared_ptr<Macro> (new Curve(newLegIDX, newControlPoints));
}

void Curve::display() {
    cerr << "Displaying macro: Bezier" << endl;
    cerr << "legIDX=" << legIDX << " controlPoints="
         << curve->getControlPoints() << endl;
}

void Curve::setTimeParam(const float & t_) {
    t = t_;
}

// Reset
// -----
Reset::Reset(const std::vector<int>& components_) :
    components( components_ ) {
}

void Reset::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    vector<int>::iterator begin( components.begin() ), end( components.end() );
    for( ; begin != end; ++begin ) {
        int currentJoint = *begin;
        if( currentJoint == HEAD ) {
            bodyModel->setInitialHead();
        } else if( currentJoint == ARM_LEFT ) {
            bodyModel->setInitialArm(ARM_LEFT);
        } else if( currentJoint == ARM_RIGHT ) {
            bodyModel->setInitialArm(ARM_RIGHT);
        } else if( currentJoint == LEG_LEFT ) {
            bodyModel->setInitialLeg(LEG_LEFT);
        } else if( currentJoint == LEG_RIGHT ) {
            bodyModel->setInitialLeg(LEG_RIGHT);
        }
    }
}

boost::shared_ptr<Macro>
Reset::getReflection(BodyModel *bodyModel) {

    vector<int> newComponents = components;

    for(unsigned i = 0; i < components.size(); ++i ) {

        if( components[i] == ARM_LEFT ) {
            newComponents[i] = ARM_RIGHT;
        } else if ( components[i] == ARM_RIGHT ) {
            newComponents[i] = ARM_LEFT;
        } else if ( components[i] == LEG_LEFT ) {
            newComponents[i] = LEG_RIGHT;
        } else if ( components[i] == LEG_RIGHT ) {
            newComponents[i] = LEG_LEFT;
        }
    }
    return boost::shared_ptr<Macro>( new Reset( newComponents ) );
}

void Reset::
display() {
    cerr << "Displaying macro: Reset" << endl;
    vector<int>::iterator it( components.begin() ), end( components.end() );
    for( ; it != end; ++it ) {
        cerr << *it << ' ';
    }
    cerr << endl;
}


// SetScale
// --------

SetScale::SetScale(const vector<int> &effectorIDs_, const vector<double> &targetScales_) :
    effectorIDs(effectorIDs_), targetScales(targetScales_) {
    // assuming two vectors are the same length
}


void SetScale::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    // assuming two vectors are the same length
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        bodyModel->setScale( effectorIDs[i], targetScales[i] );
    }
}

boost::shared_ptr<Macro>
SetScale::getReflection(BodyModel *bodyModel) {

    vector<int> newEffIDs;

    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        int newEffID;
        double dummyAngle1, dummyAngle2;
        bodyModel->getReflection( effectorIDs[i],
                                  dummyAngle1,
                                  newEffID,
                                  dummyAngle2 );
        newEffIDs.push_back( newEffID );
    }

    return boost::shared_ptr<Macro>( new SetScale(newEffIDs, targetScales) );
}

void SetScale::display() {
    cerr << "Displaying macro: SetScale" << endl;
    for( size_t i = 0; i < effectorIDs.size(); ++i ) {
        cerr << "effID[" << i << "]=" << effectorIDs[i]
             << " Scale[" << i << "]=" << targetScales[i] << endl;
    }
}

// Stabilize
// -------
/**
 * Constructor.
 *
 * \param legIndex     LEG_LEFT or LEG_RIGHT
 */
Stabilize::Stabilize(const int &legIndex, const VecPosition zmp) :
    legIndex(legIndex), zmp(zmp) {
    this->zmp.setZ(0);
    assert(LEG_LEFT == legIndex || LEG_RIGHT == legIndex);
}

/**
 * Shifts the center of mass over the given foot (if possible).
 */
void Stabilize::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    if (!bodyModel->stabilize(legIndex, zmp)) {
        //LOG("Could not stabilize...");
    }
}

/**
 * Checks if the Stabilize Macro can be executed given a bodyModel and worldModel.
 */
bool Stabilize::canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
    bool ret = bodyModel->canStabilize(legIndex, zmp);
    if(!ret) {
        //LOG_MSG("Can't execute SetFoot", targetPos);
    }
    return ret;
}

/**
 * Reflets the Macro from between left and right foot.
 */
boost::shared_ptr<Macro> Stabilize::getReflection(BodyModel *bodyModel) {
    return boost::shared_ptr<Macro> (
               new Stabilize(legIndex == LEG_LEFT ? LEG_RIGHT : LEG_LEFT,
                             VecPosition(zmp.getX(), -1.0 * zmp.getY(), 0)));
}

/**
 * Displays the macro's target foot.
 */
void Stabilize::display() {
    cerr << "Displaying macro: Stabilize" << endl;
    cerr << "legIndex=" << legIndex << endl;
    cerr << "zmp=" << zmp << endl;
}



/////////////////////////////////////////////////////////
// KeyFrame
/////////////////////////////////////////////////////////
KeyFrame::KeyFrame() {
    // sets the default values
    toWaitTime = false;
    waitTime = 0;
    toWaitTargets = false;
    maxWaitTime = INF;
}

bool KeyFrame::done(BodyModel *bodyModel, const WorldModel *worldModel, const double &setTime) {
    const double currentTime = worldModel->getTime();
    // if invalid time - not done
    if( setTime < 0 ) {
        return false;
    }

    // Exceptional case. Break out of keyframe always.
    double elapsedTime = currentTime - setTime;
    if(elapsedTime >= maxWaitTime) {
        return true;
    }

    // If we have to wait time, have we waited long enough?
    double epsilon = 1e-4;
    if(toWaitTime && elapsedTime < waitTime - epsilon ) {
        updateCurveMacros(bodyModel, worldModel, elapsedTime / waitTime);
        return false;
    }

    // If we have to wait for targets to be reached, have they been reached?
    if(toWaitTargets && !(bodyModel->targetsReached())) {
        return false;
    }

    // We have finished time (if at all) and reached targets (if at all).
    return true;
}

void KeyFrame::updateCurveMacros(BodyModel *bodyModel, const WorldModel *worldModel, const double &t) {
    assert(t>=0.0);
    assert(t<=1.0);
    for(size_t i = 0; i < macros.size(); ++i) {
        // Try to cast the macro to a Bezier macro
        Curve *curve = dynamic_cast<Curve *>(macros[i].get());
        // If cast was successful, update the Bezier macro
        if(curve) {
            curve->setTimeParam(t);
            curve->execute(bodyModel, worldModel);
        }
    }
}

void KeyFrame::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    updateCurveMacros(bodyModel, worldModel, 0.0);
    for(unsigned m = 0; m < macros.size(); m++) {
        macros[m]->execute(bodyModel, worldModel);
    }
}

bool KeyFrame::canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
    bool ret = true;
    for(size_t i=0; i<macros.size(); ++i) {
        if(!macros[i]->canExecute(bodyModel, worldModel)) {
            ret = false;
            //LOG_MSG("Can't execute Macro", i);
        }
    }
    return ret;
}

void KeyFrame::setToWaitTime( bool value ) {
    toWaitTime = value;
}

void KeyFrame::setWaitTime( double value ) {
    waitTime = value;
}

void KeyFrame::setToWaitTargets( bool value ) {
    toWaitTargets = value;
}

void KeyFrame::setMaxWaitTime( double value ) {
    maxWaitTime = value;
}

void KeyFrame::appendMacro(boost::shared_ptr<Macro> macro) {
    macros.push_back( macro );
}

boost::shared_ptr<KeyFrame> KeyFrame::getReflection(BodyModel *bodyModel) {

    boost::shared_ptr<KeyFrame> reflection(new KeyFrame());

    reflection->setToWaitTime(this->toWaitTime);
    reflection->setWaitTime(this->waitTime);
    reflection->setToWaitTargets(this->toWaitTargets);
    reflection->setMaxWaitTime(this->maxWaitTime);

    (reflection->macros).clear();
    for(size_t m = 0; m < this->macros.size(); m++) {
        reflection->appendMacro(this->macros[m]->getReflection(bodyModel));
    }

    return reflection;
}


void KeyFrame::display() {
    cerr << "Displaying Frame" << endl;
    cerr << "toWaitTime=" << toWaitTime << endl;
    cerr << "waitTime=" << waitTime << endl;
    cerr << "toWaitTargets=" << toWaitTargets << endl;
    cerr << "maxWaitTime=" << maxWaitTime << endl;
    cerr << "Macros:\n";
    vector< boost::shared_ptr<Macro> >::iterator it( macros.begin() ), end( macros.end() );
    for( ; it != end; ++it ) {
        (*it)->display();
    }
}

////////////////////////////////////////////////////////////////
// Skill
////////////////////////////////////////////////////////////////
Skill::Skill() {
    keyFrames.clear();
    reset();
    currentKeyFrame = -1; // set to invalid
}

void Skill::reset() {
    currentKeyFrame = 0;
    currentKeyFrameSet = false;
    currentKeyFrameSetTime = -1;
}

bool Skill::done(BodyModel *bodyModel, const WorldModel *worldModel) {

    return ((currentKeyFrame == (int)(keyFrames.size() - 1)) && keyFrames[currentKeyFrame]->done(bodyModel, worldModel, currentKeyFrameSetTime));
}

void Skill::execute(BodyModel *bodyModel, const WorldModel *worldModel) {
    const double &currentTime = worldModel->getTime();
    // Check. Can be removed.
    if(done(bodyModel, worldModel)) {
        // TODO: check why it prints it before kickoff - just a matter of initialization????
//    cerr << "Ideally, this should not be printed.\n";
        return ;
    }

    if(keyFrames[currentKeyFrame]->done(bodyModel, worldModel, currentKeyFrameSetTime)) {
        currentKeyFrame += 1;
        currentKeyFrameSet = false;
        currentKeyFrameSetTime = -1;
    }

    if(!currentKeyFrameSet) {
        keyFrames[currentKeyFrame]->execute(bodyModel, worldModel);
        currentKeyFrameSet = true;
        currentKeyFrameSetTime = currentTime;
    }

    //cerr << "Skill::execute()  currentKeyFrame=" << currentKeyFrame << " currentTime=" << currentTime << "currentKeyFrameSetTime=" << currentKeyFrameSetTime << " done=" << keyFrames[currentKeyFrame]->done(bodyModel, currentKeyFrameSetTime, currentTime) << " currentKeyFrameSet=" << currentKeyFrameSet << endl;
}

/**
 * Checks if the skill can be executed given a bodyModel and worldModel.
 * Accomplished by iterating through all the skill's KeyFrames and checking that they
 * are executable.
 *
 * \param bodyModel a BodyModel
 * \param worldModel a WorldModel.
 * \return true iff all KeyFrames are executable.
 */
bool Skill::canExecute(const BodyModel *bodyModel, const WorldModel *worldModel) const {
    // HACK alomo01 - return true -- hopefully no one is using this still...
    //return true;

    bool ret = true;
    for(size_t i=0; i<keyFrames.size(); ++i) {
        if(!keyFrames[i]->canExecute(bodyModel, worldModel)) {
            ret = false;
            //LOG_MSG("Can't execute KeyFrame", i);
        }
    }
    return ret;

}

void Skill::
appendKeyFrame( boost::shared_ptr<KeyFrame> keyFrame ) {
    keyFrames.push_back( keyFrame );
    reset(); // just in case, we do it for each key frame added
}

boost::shared_ptr<Skill> Skill::getReflection(BodyModel *bodyModel) {

    boost::shared_ptr<Skill> reflection(new Skill());

    reflection->currentKeyFrame = this->currentKeyFrame;
    reflection->currentKeyFrameSet = this->currentKeyFrameSet;
    reflection->currentKeyFrameSetTime = this->currentKeyFrameSetTime;

    (reflection->keyFrames).clear();
    for(size_t k = 0; k < this->keyFrames.size(); k++) {
        reflection->appendKeyFrame(this->keyFrames[k]->getReflection(bodyModel));
    }
    /*
      // TODO: find a better way to reflect -
      // The problem with the commented code in the above line is:
      // When reflecting, it reflects left foot to right and right to left,
      // and thus the robot can start a skill in the right leg, instead of always
      // in the left. Below is a temp fix:
      int size = this->keyFrames.size();
      if (size % 2 != 0)
        throw "Uneven number of frames";
      for(int k = size / 2; k < size; k++){
        reflection->appendKeyFrame(this->keyFrames[k]->getReflection(bodyModel));
      }
      for(int k = 0; k < size / 2; k++){
        reflection->appendKeyFrame(this->keyFrames[k]->getReflection(bodyModel));
      }

    */



    return reflection;
}

void Skill::display() {
    cerr << "Displaying skill" << endl;
    vector< boost::shared_ptr<KeyFrame> >::iterator it( keyFrames.begin() ), end( keyFrames.end() );
    for( ; it != end; ++it ) {
        (*it)->display();
    }
}

int Skill::getCurrentKeyFrame() {
    return currentKeyFrame;
}
