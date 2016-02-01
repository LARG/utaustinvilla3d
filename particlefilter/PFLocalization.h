#ifndef _PF_Localization_H_
#define _PF_Localization_H_

#include "Particle.h"
#include "../worldmodel/worldmodel.h"
#include "../bodymodel/bodymodel.h"
#include "../headers/Field.h"

#include <MotionCore.h>

int compareProbabilities(const void *one, const void *two);

#define ALLOW_LOC_DEBUG

// Parameters
struct ParticleParams {
    int NUM_PARTICLES;

    int RESAMPLE_FREQ;
    bool draw;
    bool normalizeLineProb;
    bool USE_ALT_LINE_LOC;
    int numLinesToKeep;
    double SIGMA_SCALE;
    double DELTA_DIST;
    SIM::AngRad DELTA_ANG;
    double DEGRADE_FACTOR;
    double RANDOM_PROB;
    double SIDE_BEAM_PROB;
    double REQUEST_BEAM_RESEED_PERC;
    double TEAMMATE_SIGHTING_RESEED_PERC;
    int RANDOM_FREQ;
    int NUM_RANDOM;
    bool LOC_DEBUG;
    bool USE_LANDMARKS;
    bool USE_LINES;
    bool USE_INTERSECTIONS;
    bool USE_GOALPOSTS;
    bool USE_DISTANCES;
    bool USE_BEARINGS;
    bool USE_NEGATIVE_INFO;
    bool USE_RANDOM;
    bool USE_CIRCLE;
    bool RESET_PROB_EVERY;


    // history
    double MAX_HIST_BEAR;
    double MAX_HIST_DIST;
    int OBSERVATION_LENGTH;

    // reseed parameters
    int OLDEST_RESEED_FRAME;
    double MIN_RESEED_PROB;
    double MAX_RESEED_PROB;
    int MAX_RESEED_PARTICLES;
    SIM::AngRad MIN_RESEED_BEARING_DIFF;
    double RESEED_PROB_BOOST;
    bool DO_ONE_LM_RESEED;
    int RESEED_FREQ;
    int NUM_ONE_LM_PARTS;
    bool USE_INT_FOR_RESEED;

    // Sigma's for different observation types
    double BEACON_DIST_SIGMA;
    double BEACON_BEAR_SIGMA;
    double GOAL_BEAR_SIGMA;
    double GOAL_DIST_SIGMA;
    double CLOSEGOAL_DIST_SIGMA;
    double LINE_DIST_SIGMA;
    double LINE_BEAR_SIGMA;
    double INT_DIST_SIGMA;
    double INT_BEAR_SIGMA;
    double CROPPED_SIGMA_FACTOR;
    bool USE_GOALIE_HACKS;

    bool USE_BALL;
    double BALL_DIST_SIGMA;
    double BALL_BEAR_SIGMA;

    // max dist/bearing to even consider a line/int match
    double MAX_SIM_DISTANCE;
    SIM::AngRad MAX_SIM_BEARING;
    double MAX_NEGINFO_DISTANCE;
    SIM::AngRad MAX_NEGINFO_BEARING;
    double NO_INT_MATCH_PROB;
    double NO_LINE_MATCH_PROB;
    double MIN_LINE_PROB;
    double MIN_INT_PROB;
    double MAX_INT_DIST;
    double MAX_LINE_DIST;

    double MAX_LM_DIST;

    // ignore ints near center circle
    bool IGNORE_CENTER_LINES;
    double IGNORE_CENTER_DIST;
    double IGNORE_CENTER_BEAR;


    // negative info parameters
    int NUM_MISSED_FRAMES;
    double MISS_PROB;
    int IMAGE_BUFFER;

    // ball kf
    double RESET_DISTANCE;

    bool WRITE_RESULT_FILE;
};





class PFLocalization {
public:
    PFLocalization(WorldModel* worldModel_, BodyModel* bodyModel_, MotionCore* core_);
    ~PFLocalization();

    void processFrame();

//  Memory* memory;
//  Snapshot* current;

    // These should be replacing the memory and current in the Nao PFLocalization
    WorldModel* worldModel;
    BodyModel* bodyModel;
    MotionCore *core;

    // Indices of landmarks from which we update our location
    vector<int> landmarkIndices;

    // Tables for odometry
    map<SkillType, VecPosition> skillVectorDisplacement;
    map<SkillType, double> skillAngularDisplacementDegrees;


    // our ballkf
//  BallKF ball;

    double timePassed;
    double timeLast;

    // actual particles
    Particle* particles;

    // Temp particles for swapping
    // (so we're not allocating memory for this all the time)
    Particle* tmpParticles;


    ParticleParams partParams;

    //void setParams(ParticleParams* params){ partParams = params; };

    // Boundary for particles to stay in
    SIM::Rectangle m_boundary;

    // File for writing results to if running experiments
    FILE* resultFile;

    //field line coordinates
    vector<SIM::Line2D> actualLines;
    vector<SIM::Line2D> myObservedLines;
    double fieldofview;
    SIM::Line2D leftViewCone;
    SIM::Line2D rightViewCone;
//	double kRatio;
    int** expected;
    int** notseen;
    int numActualLines;
    int* matchings;
    double headAnglePan;
    double headPanOffset;

    void fillMyObservedLines();
    void resetMatchings();

    // PFLocalization functions on particles


    // Particle updates
    void updateParticlesFromObservations();
    void updateParticleFromObservations(Particle *part, int counter);
    void updateParticlesFromNegaInfoOnly();

    double updateParticleFromLandmarks(Particle *part);
    double updateParticleFromUnknownGoalposts(Particle *part);
    double updateParticleFromIntersections(Particle *part);
    double updateParticleFromNegativeInfo(Particle *part, int counter, bool debug);
    double updateParticleFromNegativeInfo2(Particle *part, int counter, bool debug);
    double updateParticleFromLines(Particle *part, bool debug);
    bool pointWithinView(SIM::Point2D pt, double kRatio);
    double getLineProb(SIM::Line2D observedLine, SIM::Line2D actualLine, Particle *part, bool debug);
    double getLineProb2(SIM::Line2D observedLine, SIM::Line2D actualLine, Particle *part, bool debug);
    void findIntersectionPoints(SIM::Point2D start, SIM::Point2D end, SIM::Point2D &trueStart, SIM::Point2D &trueEnd, bool& failure);

    VecPosition getOdometryDisplacementEstimateXY();
    void updateParticlesFromOdometry();

    double getProbKnownObjects( const SIM::Point2D& partLoc, const SIM::AngRad& partOrient,
                                double bearSigma, double distSigma );
    double getProbUnknownObjects( int UNKNOWN_START_INDEX,
                                  int UNKNOWN_END_INDEX,
                                  int KNOWN_START_INDEX,
                                  int KNOWN_END_INDEX,
                                  const SIM::Point2D& partLoc,
                                  const SIM::AngRad& partOrient,
                                  double bearSigma, double distSigma );

    // reseeding and resampling
    void resampleParticles();

    // triangulation for reseeding
    void estimateRobotPose(SIM::Point2D& pos, SIM::AngRad& ang, bool& confidence);
    void estimateRelativeObjectLocations(SIM::Point2D& pos, SIM::AngRad& ang);
    void filterBall();

    double getProbabilitySum(Particle *part, int numPart);

    void setParticleProbabilities(double newProb);
    void resetParticles();
    void resetKickoffParticles();

    void printParticles();
    void writeResults();

    void randomWalkParticles( bool useProbForRandWalk );

    bool atleastOneObjSeen();
    bool atLeastOneLineSeen();


    void updateWorldModelMat(SIM::Point2D&, SIM::AngRad&);

    void setForBeam(double x, double y, double angle);
    void setForTeammateSighting(double x, double y, double angle);

    // PFLocalization helper functions
    double getDistanceSim( double d1, double d2, double sigma);
    double getSim( double d1, double sigma);
    double getBearingSim( SIM::AngRad a1, SIM::AngRad a2, double sigma);
    int sampleIndexFromProbArray( double[] );

};

#endif
