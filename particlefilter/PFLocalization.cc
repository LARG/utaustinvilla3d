#include "PFLocalization.h"
#include <iostream>
#include <iomanip>
#include "MyRandom.h"

#include <motion/MotionModule.h>

PFLocalization::PFLocalization(WorldModel* worldModel_, BodyModel* bodyModel_, MotionCore *core_) :
    m_boundary( GRASS ) {

    worldModel = worldModel_;
    bodyModel = bodyModel_;
    core = core_;

    partParams.draw = false;
    partParams.USE_ALT_LINE_LOC = false;
    partParams.normalizeLineProb = true;
    partParams.numLinesToKeep = 2;
    partParams.SIGMA_SCALE = 2.0;
    partParams.NUM_PARTICLES=500;

    partParams.RESAMPLE_FREQ = 1;
    partParams.RESEED_FREQ = 15;
    partParams.DELTA_DIST = 0.15;
    partParams.DELTA_ANG = DEG_T_RAD * 30;
    partParams.DEGRADE_FACTOR = 0.99;

    partParams.RESET_PROB_EVERY = true;

    partParams.RANDOM_PROB = 0.05;
    partParams.NUM_RANDOM = 10;
    partParams.RANDOM_FREQ = 15;

    partParams.SIDE_BEAM_PROB = 0.0;
    partParams.REQUEST_BEAM_RESEED_PERC = 0.99;
    partParams.TEAMMATE_SIGHTING_RESEED_PERC = 0.75;

    // history
    partParams.OBSERVATION_LENGTH = 30;
    partParams.MAX_HIST_DIST = 150;
    partParams.MAX_HIST_BEAR = DEG_T_RAD * 10.0;

    // reseed
    partParams.OLDEST_RESEED_FRAME = 30;
    partParams.MIN_RESEED_PROB = 0.8;
    partParams.MAX_RESEED_PROB = 0.9;
    partParams.MAX_RESEED_PARTICLES = 5;
    partParams.MIN_RESEED_BEARING_DIFF = DEG_T_RAD * 0.75;
    partParams.RESEED_PROB_BOOST = 0.0;
    partParams.DO_ONE_LM_RESEED = true;
    partParams.NUM_ONE_LM_PARTS = 3;
    partParams.USE_INT_FOR_RESEED = false;

    // flags
    partParams.LOC_DEBUG = false;

    partParams.USE_LANDMARKS = true;
    partParams.USE_LINES = true;
    partParams.USE_INTERSECTIONS = true;
    partParams.USE_GOALPOSTS = true;
    partParams.USE_DISTANCES = true;
    partParams.USE_BEARINGS = true;
    partParams.USE_NEGATIVE_INFO = false;

    partParams.USE_RANDOM = false;
    partParams.USE_CIRCLE = true;

    // sigma's
    partParams.GOAL_DIST_SIGMA = 0.1;
    partParams.GOAL_BEAR_SIGMA = 0.1;

    partParams.USE_BALL = true;

    partParams.NO_LINE_MATCH_PROB = 0.1; // -- dont penalize for bad int/lines obs
    partParams.NO_INT_MATCH_PROB = 0.1;
    partParams.MIN_LINE_PROB = 0.1;
    partParams.MIN_INT_PROB = 0.1;

    partParams.IGNORE_CENTER_LINES = false;
    partParams.IGNORE_CENTER_DIST = 1000;
    partParams.IGNORE_CENTER_BEAR = DEG_T_RAD * 60.0;

    // negative info parameters
    partParams.NUM_MISSED_FRAMES = 5;
    partParams.MISS_PROB = 0.2;
    partParams.IMAGE_BUFFER = 15;

    partParams.WRITE_RESULT_FILE = false;

    // tables for odometry - vector displacement (w.r.t current direction) in cm, per skill execution
    // We look at the skill length and divided by 0.02 to normalize it to 1 cycle of execution

    skillVectorDisplacement[SKILL_WALK_OMNI] = VecPosition( 0, 0, 0 );

    // tables for odometry - angular displacement (w.r.t current direction)

    skillAngularDisplacementDegrees[SKILL_WALK_OMNI] = 0;




    // Indices of landmarks from which we update our location
    landmarkIndices.push_back( FLAG_1_L );
    landmarkIndices.push_back( FLAG_1_R );
    landmarkIndices.push_back( FLAG_2_L );
    landmarkIndices.push_back( FLAG_2_R );
    landmarkIndices.push_back( GOALPOST_1_L );
    landmarkIndices.push_back( GOALPOST_1_R );
    landmarkIndices.push_back( GOALPOST_2_L );
    landmarkIndices.push_back( GOALPOST_2_R );


    particles = new Particle[partParams.NUM_PARTICLES];
    // init temp particles
    tmpParticles = new Particle[partParams.NUM_PARTICLES];

    // init particles
    resetParticles();

    //	// middle line
    //	(0,-HALF_FIELD_Y); (0,HALF_FIELD_Y)
    //
    //	// ground lines
    //	(HALF_FIELD_X,-HALF_FIELD_Y); (HALF_FIELD_X,HALF_FIELD_Y)
    //	(-HALF_FIELD_X,-HALF_FIELD_Y); (-HALF_FIELD_X,HALF_FIELD_Y)
    //
    //	// side lines
    //	(HALF_FIELD_X,HALF_FIELD_Y);(-HALF_FIELD_X,HALF_FIELD_Y)
    //	(HALF_FIELD_X,-HALF_FIELD_Y);(-HALF_FIELD_X,-HALF_FIELD_Y)


    //coordinates of actual lines
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X, HALF_FIELD_Y),SIM::Point2D(HALF_FIELD_X, HALF_FIELD_Y)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(HALF_FIELD_X, -HALF_FIELD_Y),SIM::Point2D(HALF_FIELD_X, HALF_FIELD_Y)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X, -HALF_FIELD_Y),SIM::Point2D(HALF_FIELD_X, -HALF_FIELD_Y)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X, -HALF_FIELD_Y),SIM::Point2D(-HALF_FIELD_X, HALF_FIELD_Y)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(0, -HALF_FIELD_Y),SIM::Point2D(0, HALF_FIELD_Y)));


    actualLines.push_back(SIM::Line2D(SIM::Point2D(HALF_FIELD_X-PENALTY_X,PENALTY_Y/2.0),SIM::Point2D(HALF_FIELD_X-PENALTY_X,-PENALTY_Y/2.0)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(HALF_FIELD_X-PENALTY_X,PENALTY_Y/2.0),SIM::Point2D(HALF_FIELD_X,PENALTY_Y/2.0)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(HALF_FIELD_X-PENALTY_X,-PENALTY_Y/2.0),SIM::Point2D(HALF_FIELD_X,-PENALTY_Y/2.0)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X+PENALTY_X,PENALTY_Y/2.0),SIM::Point2D(-HALF_FIELD_X+PENALTY_X,-PENALTY_Y/2.0)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X+PENALTY_X,PENALTY_Y/2.0),SIM::Point2D(-HALF_FIELD_X,PENALTY_Y/2.0)));
    actualLines.push_back(SIM::Line2D(SIM::Point2D(-HALF_FIELD_X+PENALTY_X,-PENALTY_Y/2.0),SIM::Point2D(-HALF_FIELD_X,-PENALTY_Y/2.0)));

    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(2,0),SIM::Point2D(1.618033989, 1.175570505)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(1.618033989, 1.175570505),SIM::Point2D(0.618033989, 1.902113033)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(0.618033989, 1.902113033),SIM::Point2D(-0.618033989, 1.902113033)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-0.618033989, 1.902113033),SIM::Point2D(-1.618033989, 1.175570505)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-1.618033989, 1.175570505),SIM::Point2D(-2,0)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-2,0),SIM::Point2D(-1.618033989, -1.175570505)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-1.618033989, -1.1755705053),SIM::Point2D(-0.618033989, -1.902113033)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-1.618033989, -1.1755705053),SIM::Point2D(-0.618033989, -1.902113033)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(-0.618033989, -1.902113033),SIM::Point2D(1.618033989, -1.175570505)));
    //		actualLines.push_back(SIM::Line2D(SIM::Point2D(1.618033989, -1.175570505),SIM::Point2D(2,0)));


    fieldofview = 110;
    //	kRatio = tan(Deg2Rad(90-fieldofview/2));
    leftViewCone = SIM::Line2D(SIM::Point2D(), SIM::Point2D(100, 100*tan(Deg2Rad(fieldofview/2))));
    rightViewCone = SIM::Line2D(SIM::Point2D(), SIM::Point2D(100, -100*tan(Deg2Rad(fieldofview/2))));

    numActualLines = 11;
    matchings = new int[numActualLines]();
}

PFLocalization::~PFLocalization() {
    delete [] particles;
    delete [] tmpParticles;
    delete [] matchings;
    if (partParams.WRITE_RESULT_FILE) {
        fclose(resultFile);
    }
}

bool PFLocalization::atLeastOneLineSeen() {
    return ((int) myObservedLines.size()) > 0;
    //	return ((int) worldModel->getWorldObject(LINE)->lines.size()) > 0;
}

bool lineGreaterThan(SIM::Line2D a, SIM::Line2D b) {
    double length1 = a.getStart().getDistanceTo(a.getEnd());
    double length2 = b.getStart().getDistanceTo(b.getEnd());
    return length1 > length2;
}

void PFLocalization::fillMyObservedLines() {
    vector<VisionObject> myLines = worldModel->getWorldObject(LINE)->lines;
    vector<SIM::Line2D>shortLines;
    myObservedLines.clear();
    for (int i=0; i < (int) myLines.size(); i++) {
        VecPosition _observedLineStart = myLines[i].polar.getCartesianFromPolar();
        VecPosition _observedLineEnd = myLines[i].polar2.getCartesianFromPolar();
        SIM::Point2D observedLineStart =  SIM::Point2D(_observedLineStart.getX(), _observedLineStart.getY());
        SIM::Point2D observedLineStop =  SIM::Point2D(_observedLineEnd.getX(), _observedLineEnd.getY());
        SIM::Line2D observedLine = SIM::Line2D(observedLineStart, observedLineStop);
        if (observedLineStart.getDistanceTo(observedLineStop) > 1.5) {
            myObservedLines.push_back(observedLine);
        } else {
            shortLines.push_back(observedLine);
        }
    }

    if (myObservedLines.size() == 0 && shortLines.size() == 2) {
        myObservedLines.push_back(shortLines[0]);
        myObservedLines.push_back(shortLines[1]);
    }
    else if (myObservedLines.size() == 0 && shortLines.size() == 1) {
        myObservedLines.push_back(shortLines[0]);
    }
    else if (myObservedLines.size() == 1 && (shortLines.size() == 2 || shortLines.size() == 1)) {
        myObservedLines.push_back(shortLines[0]);
    }

    if ((int) myObservedLines.size() > partParams.numLinesToKeep) {
        sort(myObservedLines.begin(), myObservedLines.end(), lineGreaterThan);
        myObservedLines.resize(partParams.numLinesToKeep);
    }
}

void PFLocalization::processFrame() {
    static int processingIteration = 0;
    ++processingIteration;

    timePassed = worldModel->getTime() - timeLast;
    timeLast = worldModel->getTime();

    updateParticlesFromOdometry();
    fillMyObservedLines();

    if( atleastOneObjSeen() || atLeastOneLineSeen()) {
        if (!worldModel->isFallen()) {
            // [patmac] this should really be based on the variance of particles
            worldModel->setLocalized(true);
        }
        updateParticlesFromObservations();
    }
    //	} else if (partParams.USE_NEGATIVE_INFO){
    //		updateParticlesFromNegaInfoOnly();
    //	}

    // update pose
    SIM::Point2D pos;
    SIM::AngRad ang;
    bool conf;
    estimateRobotPose(pos, ang, conf); // compute position
    worldModel->setMyLastPosition(worldModel->getMyPosition());
    worldModel->setMyLastAngRad(worldModel->getMyAngRad());
    worldModel->setMyPosition( pos );
    worldModel->setMyAngRad( ang );
    worldModel->setMyConfidence( conf );


    if( atleastOneObjSeen() || atLeastOneLineSeen()) {
        if( processingIteration % partParams.RESAMPLE_FREQ == 0 ) {
            resampleParticles();
            bool useProbForRandWalk = false;
            randomWalkParticles( useProbForRandWalk ); // it's like sampling from the distribution given the observation
        }
    }

    // update ball
    filterBall();

    // update relative object locations
    // estimateRelativeObjectLocations(pos, ang);// use position
    // print particles (for debug)

#ifdef ALLOW_LOC_DEBUG
    if (partParams.LOC_DEBUG)
        printParticles();
#endif

    if (partParams.WRITE_RESULT_FILE)
        writeResults();
}

double PFLocalization::
getSim(double value, double sigma ) {

    double normalizedDeviation = value / sigma;

    double gaussExponent = -0.5 * ( normalizedDeviation * normalizedDeviation );

    //  return std::exp( gaussExponent );
    return ( gaussExponent );

}


// Gets the similarity measure based on seen and expected angles of
// the landmarks. smaller sigma means less discrimination
double PFLocalization::
getBearingSim( SIM::AngRad ang1, SIM::AngRad ang2, double sigma ) {

    double normalizedDeviation = ( ang1 - ang2 ) / sigma;

    double gaussExponent = -0.5 * ( normalizedDeviation * normalizedDeviation );

    //  return std::exp( gaussExponent );
    return ( gaussExponent );

}


// Gets the similarity measure based on seen and expected distances to
// two objects. smaller sigma means less discrimination
double PFLocalization::
getDistanceSim( double expectedDistance, double observedDistance,
                double sigma ) {

    double normalizedDeviation = ( expectedDistance - observedDistance ) / sigma;

    double gaussExponent = -0.5 * ( normalizedDeviation * normalizedDeviation );

    //  return std::exp( gaussExponent );
    return ( gaussExponent );

}

/** Set particle probabilities to one and place randomly */
void PFLocalization::resetParticles() {

    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {
        Particle *part = &particles[i];
        part->setProbability(1);
        part->placeRandomly(m_boundary);
    }

}

void PFLocalization::setParticleProbabilities(double newProb) {
    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {
        Particle *part = &particles[i];
        part->setProbability(newProb);
    }
}

VecPosition PFLocalization::getOdometryDisplacementEstimateXY() {
    const double cycle = 0.02;
    const double mm2m = 0.001;
    const double halfStepFactor = 0.5;
    const double tuningX = 0.8;
    const double tuningY = 0.3;
    const double factor = halfStepFactor *
                          cycle *
                          mm2m *
                          1.0/core->motion_->getPhaseLength();

    return VecPosition(tuningX*factor*core->motion_->getStepSizeTranslationX(), tuningY*factor*core->motion_->getStepSizeTranslationY(), 0);
}

// TODO: implement this function
void PFLocalization::updateParticlesFromOdometry() {
    // compute accumulated displacement
    VecPosition estimatedPosDisp( 0, 0, 0 );
    double estimatedAngleDispDeg = 0;
    const vector<SkillType>& executedSkills = worldModel->getExecutedSkills();
    vector<SkillType>::const_iterator begin( executedSkills.begin() );
    vector<SkillType>::const_iterator end( executedSkills.end() );

//cerr << "skills executed: " << executedSkills.size() << endl;;
    for( ; begin != end; ++begin ) {
        // cerr << "Executed skill: " << EnumParser<SkillType>::getStringFromEnum(*begin)  << endl;

        VecPosition currentDirection = getDirectionVector( Deg2Rad( estimatedAngleDispDeg ) );
        VecPosition lateralDirection = VecPosition(0, 0, 1.0).crossProduct(currentDirection);
        SkillType skill = *begin;

        // if skill exists in table - get values, otherwise it is 0
        if( skillVectorDisplacement.find( skill ) != skillVectorDisplacement.end() ) {
            VecPosition disp = skillVectorDisplacement[skill];
            if (skill == SKILL_WALK_OMNI) {
                disp = getOdometryDisplacementEstimateXY();
                /*
                    const double cycle = 0.02;
                    const double mm2m = 0.001;
                    const double halfStepFactor = 0.5;
                    const double tuningX = 0.8;
                    const double tuningY = 0.3;
                    const double factor = halfStepFactor *
                                          cycle *
                                          mm2m *
                                          1.0/core->motion_->getPhaseLength();

                    disp = VecPosition(tuningX*factor*core->motion_->getStepSizeTranslationX(), tuningY*factor*core->motion_->getStepSizeTranslationY(), 0);
                */
                //cout << "disp = " << disp << endl;
            }
            estimatedPosDisp += currentDirection * disp.getX() + lateralDirection * disp.getY();
        }
        // if skill exists in table - get values, otherwise it is 0
        if( skillAngularDisplacementDegrees.find( skill ) != skillAngularDisplacementDegrees.end() ) {
            double angDispDeg = skillAngularDisplacementDegrees[skill];
            if (skill == SKILL_WALK_OMNI) {
                const double factor = .02*1.0/core->motion_->getPhaseLength();
                const double tuningAng = 0.2;
                angDispDeg = Rad2Deg(tuningAng*factor*core->motion_->getStepSizeRotation());
                //cout << "ang disp = " << angDispDeg << endl;
            }
            estimatedAngleDispDeg += angDispDeg;
        }
    }


    SIM::Point2D disp2D( estimatedPosDisp.getX(), estimatedPosDisp.getY() );

    worldModel->setLastOdometryPos(disp2D);
    worldModel->setLastOdometryAngDeg(estimatedAngleDispDeg);

    //cerr << "displacement=" << estimatedPosDisp << "estimatedAngleDispDeg="<< estimatedAngleDispDeg << endl;
    // move particles
    for (int i = 0; i < partParams.NUM_PARTICLES; ++i) {
        Particle *part = &particles[i];
//      SIM::Point2D disp2D( estimatedPosDisp.getX(), estimatedPosDisp.getY() );
        part->moveRelative(disp2D, Deg2Rad( estimatedAngleDispDeg ), m_boundary);
        //part->degradeProbability( partParams.DEGRADE_FACTOR );
    }

    worldModel->resetExecutedSkills();
    //  cerr << "CURRENTLY NO UPDATE FROM ODOMETRY!! THE CODE MIGHT NOT BE WORKING...\n";
}

void PFLocalization::updateParticlesFromNegaInfoOnly() {
    headAnglePan = bodyModel->getJointAngle(EFF_H1);
    headPanOffset = Deg2Rad((int)headAnglePan % 360);
    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {
        Particle* part = &particles[i];
        double newlogprob = updateParticleFromNegativeInfo(part, i, false);
        part->degradeByLogProbability(newlogprob);
    }
}

void PFLocalization::updateParticlesFromObservations() {
    headAnglePan = bodyModel->getJointAngle(EFF_H1);
    headPanOffset = Deg2Rad((int)headAnglePan % 360);
    //cout << "headAnglePan: " << headAnglePan << endl;
    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {

#ifdef ALLOW_LOC_DEBUG
        if (partParams.LOC_DEBUG) {
            //      memory->log(85, "Update Particle %d", i);
            //      memory->log(90, "((%5.0f, %5.0f), %5.0f, %5.2f)",
            //      memory->log(85, "Update Particle %d", i);
            //      memory->log(90, "((%5.0f, %5.0f), %5.0f, %5.2f)",
            //		  current->localizationMem->particles[i].M_pos.x,
            //		  current->localizationMem->particles[i].M_pos.y,
            //		  Rad2Deg(current->localizationMem->particles[i].M_ang),
            //		  current->localizationMem->particles[i].M_prob);
        }
#endif
        //cout << "updating particle: " << i << endl;
        updateParticleFromObservations(&particles[i], i);
        if (partParams.draw and particles[i].getLogProbability() <= -20.0)
            worldModel->getRVSender()->drawCircle(particles[i].getPosition().getX(), particles[i].getPosition().getY(), 0.03, RVSender::MAGENTA);
        if (partParams.draw and particles[i].getLogProbability() > -20.0)
            worldModel->getRVSender()->drawCircle(particles[i].getPosition().getX(), particles[i].getPosition().getY(), 0.03, RVSender::BLUE);
    }

    if (!partParams.draw)
        return;

    Particle gdParticle = Particle(1.0, SIM::Point2D(worldModel->getMyPositionGroundTruth().getX(),
                                   worldModel->getMyPositionGroundTruth().getY()), Deg2Rad(worldModel->getMyAngDegGroundTruth()), m_boundary);
    Particle invgdParticle = Particle(1.0, SIM::Point2D(worldModel->getMyPositionGroundTruth().getX(),
                                      worldModel->getMyPositionGroundTruth().getY()), Deg2Rad(worldModel->getMyAngDegGroundTruth()) + 3.14159265359, m_boundary);
    Particle zeroParticle = Particle(1.0, SIM::Point2D(), 0.0, m_boundary);

    for (int i=0; i< (int) actualLines.size(); i++) {
        SIM::Point2D closest = gdParticle.getNearestPointOnLineSeg(actualLines[i]);
        worldModel->getRVSender()->drawCircle(closest.getX(), closest.getY(), 0.2, RVSender::ORANGE);
    }

    for (int i=0; i< (int) myObservedLines.size(); i++) {
        SIM::Point2D start = myObservedLines[i].getStart().relativeToGlobal(gdParticle.getPosition(), gdParticle.getAngle());
        SIM::Point2D end = myObservedLines[i].getEnd().relativeToGlobal(gdParticle.getPosition(), gdParticle.getAngle());
        //		SIM::Point2D localStart, localEnd;
        //		bool failure;
        //		findIntersectionPoints(start, end, localStart, localEnd, failure);
        SIM::Line2D tempLine = SIM::Line2D(start, end);
        SIM::Point2D closest = gdParticle.getNearestPointOnLineSeg(tempLine);
        worldModel->getRVSender()->drawLine(start.getX(), start.getY(), end.getX(), end.getY(), RVSender::VIOLET);
        worldModel->getRVSender()->drawCircle(closest.getX(), closest.getY(), 0.1, RVSender::GREEN);

    }
}

// update particle probabilities from observations
void PFLocalization::updateParticleFromObservations(Particle *part, int counter) {

    double newLogProb = 0;

    // observations of distinct landmarks
    if (partParams.USE_LANDMARKS) {
        newLogProb += updateParticleFromLandmarks(part);
    }

    if (partParams.USE_LINES) {
        newLogProb += updateParticleFromLines(part, false);
    }

    if (partParams.USE_NEGATIVE_INFO) {
        newLogProb += updateParticleFromNegativeInfo(part, counter, false);
        //		newLogProb += updateParticleFromNegativeInfo2(part, counter, false);
    }

    // update particle probability
#ifdef ALLOW_LOC_DEBUG
    if (partParams.LOC_DEBUG)
        cerr << "-I-  Updating particle with prob " << exp( newLogProb );
#endif
    part->degradeByLogProbability( newLogProb );
    // cerr << "probability: " << part->getProbability() << endl;

}

/*
 * Daniel added: HELPER FUNCTIONS FOR LOCALIZATION
 */

double PFLocalization::getProbKnownObjects(
    const SIM::Point2D& partLoc, const SIM::AngRad& partOrient,
    double bearSigma, double distSigma ) {
    double newLogProb = 0;

    vector<int>::const_iterator begin( landmarkIndices.begin() ),
           end( landmarkIndices.end() );
    for( ; begin != end; ++begin ) {
        int i = *begin;
        const WorldObject* currentObj = worldModel->getWorldObject( i );
        if ( currentObj->currentlySeen ) {
            SIM::Point2D currentObjLoc( currentObj->pos.getX(), currentObj->pos.getY() );
            double expectedDistance = partLoc.getDistanceTo(currentObjLoc);
            double expectedBearing = partLoc.getBearingTo(currentObjLoc, partOrient);
            double visionDistance = currentObj->vision.polar.getX(); // x is r, distance
            double visionBearing = Deg2Rad( currentObj->vision.polar.getY() );
            // TODO: add phi? (vertical angle)
            newLogProb += getBearingSim( expectedBearing, visionBearing, bearSigma );
            newLogProb += getDistanceSim( expectedDistance, visionDistance, distSigma );
        }
    } // lm loop
    return newLogProb;
}


/*
 * LANDMARKS
 */

double PFLocalization::updateParticleFromLandmarks(Particle *part) {

    // update particle from vision of distinct landmarks (blue goal, blue left post, blue right post, and same for yellow)
    double newLogProb = 0;

    SIM::Point2D partLoc = part->getPosition(); // M_pos;
    SIM::AngRad partOrient = part->getAngle(); // M_ang

    newLogProb += getProbKnownObjects(	partLoc, partOrient,
                                        partParams.GOAL_BEAR_SIGMA*partParams.SIGMA_SCALE,
                                        partParams.GOAL_DIST_SIGMA*partParams.SIGMA_SCALE);

    return newLogProb;
}

bool PFLocalization::pointWithinView(SIM::Point2D pt, double limit) {
    if (pt.getX() > 0) {
        double angle = Rad2Deg(atan2(pt.getY(), pt.getX()));
        if (angle < limit and angle > -limit)
            return true;
    }
    return false;
}

double PFLocalization::updateParticleFromNegativeInfo(Particle *part, int counter, bool debug) {
    double newLogProb = 0;
    int numBadLandMarks = 0;

    if (debug) {
        cout << "position: " << part->getPosition() << " angle: " << Rad2Deg(part->getAngle()) << " head pan: " << headAnglePan << endl;
    }
    vector<int>::const_iterator begin( landmarkIndices.begin() ),
           end( landmarkIndices.end() );
    for( ; begin != end; ++begin ) {
        int i = *begin;
        const WorldObject* currentObj = worldModel->getWorldObject( i );
        if ( !currentObj->currentlySeen ) {
            SIM::Point2D _pt = SIM::Point2D(currentObj->pos.getX(), currentObj->pos.getY());
            SIM::Point2D pt = _pt.globalToRelative(part->getPosition(), part->getAngle() + headPanOffset);
            double distToLandmark = SIM::Point2D().getDistanceTo(pt);
            if (pointWithinView(pt, 45) and distToLandmark >= 1.25) {
                numBadLandMarks += 1;
                if (debug) {
                    cout << "expected but not seen: " << WorldObjType2Str[i] << endl;
                }
            }
        } else {
            if (debug) {
                cout << "we did see: " << WorldObjType2Str[i] << endl;
            }
        }
    }
    return -numBadLandMarks*20;
}

double PFLocalization::updateParticleFromNegativeInfo2(Particle *part, int counter, bool debug) {
    double logProb = 0.0;
    if (debug) {
        cout << "matchings: ";
        for (int i=0; i< numActualLines; i++) {
            cout << matchings[i] << " ";
        }
        cout << endl;
    }
    for (int i=0; i< numActualLines; i++) {
        if (matchings[i] != 0)
            continue;
        SIM::Line2D expectedLine = actualLines[i];
        SIM::Point2D localStart = expectedLine.getStart().globalToRelative(part->getPosition(), part->getAngle() + headPanOffset);
        SIM::Point2D localEnd = expectedLine.getEnd().globalToRelative(part->getPosition(), part->getAngle() + headPanOffset);
        SIM::Line2D localActualLine = SIM::Line2D(localStart, localEnd);
        SIM::Point2D t1 = SIM::Point2D();
        SIM::Point2D t2 = SIM::Point2D();
        if (leftViewCone.intersectWithLineSeg(localActualLine, t1) or rightViewCone.intersectWithLineSeg(localActualLine, t2)
                or pointWithinView(localStart, 0.7) or pointWithinView(localEnd, 0.7)) {
            if (debug) {
                cout << "expected line # " << i << endl;
                cout << "line start: " << expectedLine.getStart() << " end: " << expectedLine.getEnd() << endl;
            }
            logProb -= 100.0;
        }
    }
    resetMatchings();
    return logProb;
}

void PFLocalization::resetMatchings() {
    for (int i = 0; i < numActualLines; i++) {
        matchings[i] = 0;
    }
}

double PFLocalization::updateParticleFromLines(Particle *part, bool debug) {
    //	resetMatchings();
    // update particle from line info
    double newLogProb = 0;
    //preconstruct lines from visionobj

    for(int i = 0; i < (int) myObservedLines.size(); i++) {
        double bestProb = -1e308;
        int bestMatching = -1;
        //		if (debug)
        //			cout << "=========================================================================================================" << endl;
        for (int j=0; j< (int) actualLines.size(); j++) {
            //todo: need to actual line segments, not lines.
            double currentProb = getLineProb2(myObservedLines[i], actualLines[j], part, debug);
            //			if (debug) {
            //				cout << "       observed line: " << myObservedLines[i].getStart().relativeToGlobal(part->getPosition(), part->getAngle())
            //															 << " " << myObservedLines[i].getEnd().relativeToGlobal(part->getPosition(), part->getAngle())
            //															 << " actual line: " << actualLines[j].getStart() << " " << actualLines[j].getEnd()
            //															 << " current prob: " << currentProb << endl;
            //			}
            if (currentProb > bestProb) {
                bestProb = currentProb;
                bestMatching = j;
            }
        }
        //		if (debug) {
        //			cout << "observed line: " << myObservedLines[i].getStart().relativeToGlobal(part->getPosition(), part->getAngle())
        //										 << " " << myObservedLines[i].getEnd().relativeToGlobal(part->getPosition(), part->getAngle())
        //											 << " matched to " << actualLines[bestMatching].getStart() << " " << actualLines[bestMatching].getEnd()
        //											 << " best prob: " << bestProb << endl;
        //			cout << "=========================================================================================================" << endl;
        //			//			if (bestProb < -100) {
        //			//				for (int j=0; j< (int) actualLines.size(); j++) {
        //			//					double currentProb = getLineProb2(myObservedLines[i], actualLines[j], part, debug);
        //			//					cout << "*****observed line: " << myObservedLines[i].getStart().relativeToGlobal(part->getPosition(), part->getAngle())
        //			//										 << " " << myObservedLines[i].getEnd().relativeToGlobal(part->getPosition(), part->getAngle())
        //			//										 << " matched to " << actualLines[j].getStart() << " " << actualLines[j].getEnd()
        //			//									   << " prob: " << currentProb << endl;
        //			//}
        //			//}
        //}
        newLogProb += bestProb;
        //		matchings[bestMatching] += 1;
    }
    if ((int) myObservedLines.size() > 0 and partParams.normalizeLineProb)
        return newLogProb/((int) myObservedLines.size());
    return newLogProb;
}

double PFLocalization::getLineProb2(SIM::Line2D observedLine, SIM::Line2D actualLine, Particle *part, bool debug) {
    //observed line is longer than actual line, cant happen so return lowest prob.
    if (observedLine.getStart().getDistanceTo(observedLine.getEnd())/actualLine.getStart().getDistanceTo(actualLine.getEnd()) > 1.1) {
        return -1e308;
    }
    double angle_std = 0.75*partParams.SIGMA_SCALE;
    double dist_std = 0.15*partParams.SIGMA_SCALE;
    double ratio_std = 0.75*partParams.SIGMA_SCALE;

    SIM::Point2D localStart = actualLine.getStart().globalToRelative(part->getPosition(), part->getAngle());
    SIM::Point2D localEnd = actualLine.getEnd().globalToRelative(part->getPosition(), part->getAngle());
    SIM::Line2D localActualLine = SIM::Line2D(localStart, localEnd);
    double angle = localActualLine.getMyAngleWith(observedLine);
    SIM::Point2D p1 = localActualLine.getPointOnLineSegClosestTo(observedLine.getStart());
    SIM::Point2D p2 = localActualLine.getPointOnLineSegClosestTo(observedLine.getEnd());
    double length1 = max(observedLine.getStart().getDistanceTo(observedLine.getEnd()), 1e-12);
    double length2 = max(p1.getDistanceTo(p2), 1e-12);
    double absRatio = max(length1/length2, length2/length1) - 1;
    double dist1 = p1.getDistanceTo(observedLine.getStart());
    double dist2 = p2.getDistanceTo(observedLine.getEnd());

    return getSim(dist1, dist_std) + getSim(dist2, dist_std) + getSim(absRatio, ratio_std) + getSim(angle, angle_std);
}

double PFLocalization::getLineProb(SIM::Line2D observedLine, SIM::Line2D actualLine, Particle *part, bool debug) {
    if (observedLine.getStart().getDistanceTo(observedLine.getEnd()) > actualLine.getStart().getDistanceTo(actualLine.getEnd()) + 0.1) {
        return -1e308;
    }

    double dist_std = 0.3; //meters
    double theta_std = 10; //degrees
    double PI = 3.14159265359;

    SIM::Point2D pt1 = observedLine.getPointOnLineSegClosestTo(SIM::Point2D());
    double dist1 = pt1.getMagnitude();
    double theta1 = (pt1.getDirection()*180)/PI;
    SIM::Point2D localStart = actualLine.getStart().globalToRelative(part->getPosition(), part->getAngle());
    SIM::Point2D localEnd = actualLine.getEnd().globalToRelative(part->getPosition(), part->getAngle());


    SIM::Point2D pt2 = SIM::Line2D(localStart, localEnd).getPointOnLineSegClosestTo(SIM::Point2D());
    double dist2 = pt2.getMagnitude();
    double theta2 = (pt2.getDirection()*180)/PI;

    double s_d = getDistanceSim(dist2, dist1, dist_std);
    //	double r_d = dist1 - dist2;
    //	double s_d = exp(-(r_d*r_d)/(dist_std*dist_std));

    //	double r_theta = theta1 - theta2;
    //	if (r_theta > 180)
    //		r_theta -= 360;
    //	if (r_theta < -180)
    //		r_theta += 360;
    //	double s_theta = exp(-(r_theta*r_theta)/(theta_std*theta_std));
    double s_theta = getBearingSim(theta1, theta2, theta_std);

    double logProb = s_theta + s_d;
    //	double product = s_d*s_theta;
    //	double logProb = max(log(product), -100.0);
    if (debug) {
        cout << "========================" << endl;
        cout << "particle position: " << part->getPosition() << endl;
        cout << "particle angle: " << (part->getAngle()*180)/PI << endl;
        cout << "actual Line: " << actualLine << endl;
        cout << "observed Line: " << observedLine << endl;
        cout << "pt1: " << pt1 << endl;
        //		cout << "_pt2" << _pt2 << endl;
        cout << "pt2" << pt2 << endl;
        //		cout << "r_d: " << r_d << endl;
        cout << "s_d: " << s_d << endl;
        //	  cout << "r_theta: " << r_theta << endl;
        cout << "s_theta: " << s_theta << endl;
        //		cout << "s_d*s_theta: " << product << endl;
        //		cout << "log(s_d*s_theta): " << log(product) << endl;
        cout << "========================" << endl;
    }
    return logProb;
}

/** Resample particles */
void PFLocalization::resampleParticles() {
    // prepare the array of probabilities
    double particleProbabilities[partParams.NUM_PARTICLES];
    double probSum = 0;
    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {
        Particle *part = &particles[i];
        double p = part->getProbability();
        particleProbabilities[i] = p;
        probSum += p;
    }
    for (int i = 0; i < partParams.NUM_PARTICLES; ++i) {
        particleProbabilities[i] /= probSum;
    }
    // build cumulative probabilities array
    double cumPartProb[partParams.NUM_PARTICLES];
    double prev = 0;
    for (int i = 0; i < partParams.NUM_PARTICLES; ++i) {
        cumPartProb[i] = prev + particleProbabilities[i];
        prev = cumPartProb[i];
    }
    // resample
    for (int i = 0; i < partParams.NUM_PARTICLES; ++i) {
        int index = sampleIndexFromProbArray( cumPartProb );
        tmpParticles[i] = particles[index];
    }
    // Random particles (like simple reseed)
    for (int i = 0; i < partParams.RANDOM_PROB * partParams.NUM_PARTICLES; ++i) {
        int index = sampleIndexFromProbArray( cumPartProb );
        tmpParticles[index].placeRandomly(m_boundary);
    }
    // Particles at the sides for possibly getting beamed out
    for (int i = 0; i < partParams.SIDE_BEAM_PROB * partParams.NUM_PARTICLES; ++i) {
        int index = sampleIndexFromProbArray( cumPartProb );
        SIM::Point2D pos = SIM::Point2D(range_random(-1.0, 1.0), 10.0+range_random(-1.0, 1.0));
        if (index%2 == 0) {
            pos.setY(-pos.getY());
        }
        tmpParticles[index].setPosition(pos, m_boundary);
        tmpParticles[index].setAngle(range_random(-M_PI, M_PI));
    }

    // copy new array to original array
    memcpy(particles, tmpParticles, partParams.NUM_PARTICLES * sizeof(Particle));

    setParticleProbabilities(0.99);

}

int PFLocalization::sampleIndexFromProbArray( double cumProbArray[] ) {
    double sample = range_random(0.0, 1.0);
    for( int i = 0; i < partParams.NUM_PARTICLES; ++i ) {
        if (cumProbArray[i] > sample)
            return i;
    }
    //cerr << "-W- FAILED TO SAMPLE CORRECTLY " << cumProbArray[partParams.NUM_PARTICLES-1]<<  endl;
    // probably the last index - numerical error? return random index
    return int( sample * partParams.NUM_PARTICLES );
}





/** Take weighted average of particles (with SD)
  and set as global location to the sent params */
void PFLocalization::estimateRobotPose(SIM::Point2D& pos, SIM::AngRad& ang, bool& confidence) {

    // init vars
    SIM::Point2D w_possum;
    w_possum.setPoint(0, 0);
    SIM::Vector2D w_ang;
    SIM::Point2D sum;
    SIM::Point2D sum_sqr;
    sum.setPoint(0, 0);
    sum_sqr.setPoint(0, 0);

    SIM::AngRad angsum = 0.0;
    SIM::AngRad angdiff_sqr = 0.0;


    // calculate average and sd's
    double probval = 0.0;
    for( int i = 0; i < partParams.NUM_PARTICLES; i++ ) {
        Particle *part = &particles[i];
        w_possum += part->getPosition() * part->getProbability();
        probval += part->getProbability();
        w_ang += SIM::Vector2D( part->getProbability(),
                                part->getAngle(), POLAR );

        sum += part->getPosition();
        sum_sqr += part->getPosition() * part->getPosition();

        angsum += part->getAngle();
        // dont this this will work right with angles wrapping around
        //angsum_sqr += square(part->getAngle());

    }

    SIM::Point2D mean = sum / partParams.NUM_PARTICLES;
    SIM::Point2D var  = (sum_sqr - sum*mean) / partParams.NUM_PARTICLES;
    SIM::Point2D sd ( sqrt(var.x), sqrt(var.y));
    if (0.5*(sd.getX() + sd.getY()) > 1.0) {
        confidence = false;
    } else {
        confidence = true;
    }

    SIM::AngRad angmean = angsum / partParams.NUM_PARTICLES;

    // Second loop to get the squared differences of the angles
    for( int i = 0; i < partParams.NUM_PARTICLES; i++ ) {
        Particle *part = &particles[i];

        angdiff_sqr += SIM::square(SIM::normalizeAngle(part->getAngle() - angmean));
    }

    SIM::AngRad angvar = angdiff_sqr / partParams.NUM_PARTICLES;

    pos = w_possum / probval;
    ang = w_ang.getDirection();
}




/** For each world object, calculate relative location and
  update its world object. */
void PFLocalization::estimateRelativeObjectLocations(SIM::Point2D& pos, SIM::AngRad& ang) {

    //  Point2D& myloc = pos;
    //  AngRad& myorient = ang;
    //
    //  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    //    // calculate distance and bearing to each object
    //    WorldObject* obj = worldModel->getWorldObject( i );
    //    // TODO: currently this is 2D, like NAO. Change->3D?
    //    Point2D objLoc ( obj->pos.getX(), obj->pos.getY() );
    //    obj->distance =
    //      myloc.getDistanceTo(objLoc);
    //    obj->bearing =
    //      myloc.getBearingTo(objLoc, myorient);
    //
    //  }

}


double PFLocalization::getProbabilitySum(Particle* part, int numPart) {
    double probSum = 0.0;
    for (int i = 0; i < numPart; i++) {
        probSum += part[i].getProbability();
    }
    return probSum;
}


void PFLocalization::printParticles() {

    for (int i = 0; i < partParams.NUM_PARTICLES; i++) {
        Particle *part = &particles[i];
        printf ("-I- Particle %d: ((%5.2f, %5.2f), %5.0f, %5.2f)\n", i, part->getPosition().x,
                part->getPosition().y, Rad2Deg(part->getAngle()), part->getProbability());
    }
}

void PFLocalization::writeResults() {
    //  fprintf(resultFile,"%i,%i\n",current->frameNum,partParams.NUM_PARTICLES);
    //  WorldObject* wo = &current->commonMem->worldObjects[current->commonMem->WO_SELF];
    //  fprintf(resultFile,"%lf,%lf,%lf\n",wo->loc.x,wo->loc.y,wo->orientation);
    //  fprintf(resultFile,"%lf,%lf,%lf\n",wo->sd.x,wo->sd.y,wo->sdOrientation);
    //  for (int i = 0; i < partParams.NUM_PARTICLES; i++){
    //    Particle *part = &(current->localizationMem->particles[i]);
    //    fprintf(resultFile,"%i,%lf,%lf,%lf,%lf\n",i,part->M_pos.x,part->M_pos.y,part->M_ang,part->M_prob);
    //  }
}


/** Update the kf for the ball */
void PFLocalization::filterBall() {

    // ball - time update
    //cout << "Time passed: " << timePassed << endl;
    //  ball.timeUpdate(0, timePassed);
    //
    //  // update the pose in the kf using pf estimate
    //  ball.poseMeasurementUpdate(0);
    //
    //  // ball - measurement update
    //  ball.ballMeasurementUpdate(0);
    //
    //  // set ball kf estimate into ball world object
    //  ball.updateBallFromKF(0);

}


/**
 * Comparison function for quick sort
 */
int compareProbabilities( const void *one, const void *two ) {
    Particle *a, *b;
    a = (Particle*) one;
    b = (Particle*) two;
    if( a->getProbability() >= b->getProbability() ) {
        return -1;
    }
    else {
        return 1;
    }
}


/** Perform a random walk on the particles */
void PFLocalization::randomWalkParticles( bool useProbForRandWalk ) {
    for ( int i = 0; i < partParams.NUM_PARTICLES; i++ ) {
        Particle *part = &particles[i];
        part->randomWalk( partParams.DELTA_DIST, partParams.DELTA_ANG, m_boundary, useProbForRandWalk );
    }
}

void PFLocalization::updateWorldModelMat(SIM::Point2D& pos, SIM::AngRad& ang) {
    // TODO: currently we assume that z is constant
    const double myX = pos.getX();
    const double myY = pos.getY();
    const double myZ = 0.5;
    const double myOrient = Rad2Deg( ang );

    VecPosition globalPlusPlus, globalPlusMinus, globalMinusPlus, globalMinusMinus;
    if (worldModel->getSide() == SIDE_LEFT ) {
        globalPlusPlus = worldModel->getWorldObject( FLAG_1_R )->pos;
        globalPlusMinus = worldModel->getWorldObject(FLAG_2_R)->pos;
        globalMinusPlus = worldModel->getWorldObject(FLAG_1_L)->pos;
        globalMinusMinus = worldModel->getWorldObject(FLAG_2_L)->pos;
    } else { // SIDE_RIGHT
        globalPlusPlus = worldModel->getWorldObject(FLAG_2_L)->pos;
        globalPlusMinus = worldModel->getWorldObject(FLAG_1_L)->pos;
        globalMinusPlus = worldModel->getWorldObject(FLAG_2_R)->pos;
        globalMinusMinus = worldModel->getWorldObject(FLAG_1_R)->pos;
    }


    // Compute local coordinates of corners
    VecPosition fieldXPlusYPlus( globalPlusPlus.getX() - myX,
                                 globalPlusPlus.getY() - myY,
                                 0 - myZ );
    fieldXPlusYPlus = fieldXPlusYPlus.rotateAboutZ( myOrient );

    VecPosition fieldXPlusYMinus(  globalPlusMinus.getX() - myX,
                                   globalPlusMinus.getY() - myY,
                                   0 - myZ );
    fieldXPlusYMinus = fieldXPlusYMinus.rotateAboutZ( myOrient );

    VecPosition fieldXMinusYPlus(  globalMinusPlus.getX() - myX,
                                   globalMinusPlus.getY() - myY,
                                   0 - myZ );
    fieldXMinusYPlus = fieldXMinusYPlus.rotateAboutZ( myOrient );

    VecPosition fieldXMinusYMinus( globalMinusMinus.getX() - myX,
                                   globalMinusMinus.getY() - myY,
                                   0 - myZ );
    fieldXMinusYMinus = fieldXMinusYMinus.rotateAboutZ( myOrient );

    worldModel->updateMatricesAndMovingObjs( fieldXPlusYPlus, fieldXPlusYMinus,
            fieldXMinusYPlus, fieldXMinusYMinus );
    return ;

}

bool PFLocalization::atleastOneObjSeen() {
    bool atleastOneObjSeen = false;
    int numSeen = 0;
    vector<int>::const_iterator begin( landmarkIndices.begin() ),
           end(   landmarkIndices.end()   );
    for( ; begin != end; ++begin ) {
        int i = *begin;
        if( worldModel->getWorldObject( i )->currentlySeen ) {
            //			cout << "seen: " << WorldObjType2Str[i] << endl;
            atleastOneObjSeen = true;
            ++numSeen;
            //      break;
        }
    }

    return atleastOneObjSeen;
}

void PFLocalization::setForBeam(double x, double y, double angle) {
    // Particles for when we requested a beam

    int index = int_random(partParams.NUM_PARTICLES);
    for (int i = 0; i < partParams.REQUEST_BEAM_RESEED_PERC * partParams.NUM_PARTICLES; ++i) {
        if (index >= partParams.NUM_PARTICLES) {
            index = 0;
        }
        SIM::Point2D pos = SIM::Point2D(x, y);
        particles[index].setPosition(pos, m_boundary);
        particles[index].setAngle(Deg2Rad(angle));
        index++;
    }
}

void PFLocalization::setForTeammateSighting(double x, double y, double angle) {
    //cout << "Updating " << worldModel->getUNum() << " to " << x << "," << y << " at angle " << angle << endl;
    int index = int_random(partParams.NUM_PARTICLES);
    // Particles for when a teammate tells us where we are
    for (int i = 0; i < partParams.TEAMMATE_SIGHTING_RESEED_PERC * partParams.NUM_PARTICLES; ++i) {
        if (index >= partParams.NUM_PARTICLES) {
            index = 0;
        }
        SIM::Point2D pos = SIM::Point2D(x, y);
        particles[index].setPosition(pos, m_boundary);
        particles[index].setAngle(Deg2Rad(angle));
        index++;
    }
}

