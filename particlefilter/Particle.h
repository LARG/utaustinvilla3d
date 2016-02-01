#ifndef _PARTICLE_H
#define _PARTICLE_H

#include "../math/Geometry.h"
#include "../worldmodel/WorldObject.h"


class Particle
{
protected:
    double M_prob;
    SIM::Point2D M_pos;
    SIM::AngRad M_ang;

    // data to keep track of expected obs/ neg info
    int M_expectedObs [NUM_WORLD_OBJS];
    int M_seenObs [NUM_WORLD_OBJS];

    static double MIN_PROB;
    static double MAX_PROB;

public:
    Particle();

    Particle( double prob,
              SIM::Point2D pos,
              SIM::AngRad ang,
              SIM::Rectangle boundary );


    double getProbability();
    SIM::Point2D getPosition();
    SIM::AngRad getAngle();

    void setProbability( double prob );
    void degradeProbability( double factor );
    void degradeByLogProbability( double logProb );
    double getLogProbability();
    void setLogProbability( double logProb );

    bool setPosition( SIM::Point2D pos, SIM::Rectangle boundary );
    void setAngle( SIM::AngRad ang );

    void setParticle( double prob, SIM::Point2D pos, SIM::AngRad ang,
                      SIM::Rectangle boundary );


    void moveRelative( SIM::Vector2D relativeDisplacement,
                       SIM::AngRad rotation, SIM::Rectangle boundary );

    void moveGlobal( SIM::Vector2D globalDisplacement,
                     SIM::AngRad rotation, SIM::Rectangle boundary );

    void randomWalk( double maxDistance, SIM::AngRad maxRotation,
                     SIM::Rectangle boundary, bool useProbForRandWalk );

    void placeRandomly( SIM::Rectangle boundary );

    // for lines
    SIM::Point2D getNearestPointOnLine( SIM::Line2D l );
    SIM::Point2D getNearestPointOnLineSeg( SIM::Line2D l );

    double getDistanceToPoint( SIM::Point2D p );
    SIM::AngRad getBearingToPoint( SIM::Point2D p );

    friend std::ostream& operator << ( std::ostream &os,
                                       Particle p );

    // functions for expected observations / neg info
    void resetObservations();
    int getNumFramesExpected(int i);
    int getNumFramesNotSeen(int i);
    void incrNumFramesExpected(int i);
    void incrNumFramesNotSeen(int i);
    void incrNumFramesNotSeenForAll();
    void resetNumFramesExpected(int i);
    void resetNumFramesNotSeen(int i);


};


#endif
