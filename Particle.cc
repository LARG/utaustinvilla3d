#include<stdlib.h>
#include "Particle.h"

using namespace std;

// init static variables
double Particle::MIN_PROB = 0.000000000001; // so it won't be 0 + causes problems in normalizing probabilities
double Particle::MAX_PROB = 1;

Particle::Particle() {

    M_prob = 1000;
    resetObservations();
}

Particle::Particle( double prob,
                    SIM::Point2D pos,
                    SIM::AngRad ang,
                    SIM::Rectangle boundary ) {

    setParticle( prob, pos, ang, boundary );
    resetObservations();
}

double Particle::getProbability()
{
    return exp( M_prob );
}

void Particle::setProbability( double prob )
{
//  const double MIN_PROB = 1e-4;

    double fixedProb = ( prob < MIN_PROB ) ? MIN_PROB : ( prob > MAX_PROB ) ? MAX_PROB : prob;
    M_prob = log( fixedProb );
}

void Particle::degradeProbability( double factor )
{
    setLogProbability( getLogProbability() + log( factor ) );
//  setProbability( M_prob * factor );
}

void Particle::degradeByLogProbability( double logProb ) {
    setLogProbability( getLogProbability() + logProb );
}

double Particle::getLogProbability() {
    return M_prob;
}

void Particle::setLogProbability( double logProb ) {
//  const double MIN_PROB = 1e-4; // TODO: set it in one location

    double min = log( MIN_PROB );
    double max = log( MAX_PROB );
    double fixedLogProb = ( logProb < min ) ? min : ( logProb > max ) ? max : logProb;
    M_prob = fixedLogProb;
}

bool Particle::setPosition( SIM::Point2D pos, SIM::Rectangle boundary )
{
    if ( boundary.isInside( pos ) ) {
        M_pos = pos;
        return true;
    }
    return false;
}

SIM::Point2D Particle::getPosition()
{
    return M_pos;
}


void Particle::setAngle( SIM::AngRad ang )
{
    M_ang = SIM::normalizeAngle( ang );
}

SIM::AngRad Particle::getAngle()
{
    return M_ang;
}


void Particle::setParticle( double prob, SIM::Point2D pos, SIM::AngRad ang,
                            SIM::Rectangle boundary )
{
    setProbability( prob );
    if ( !setPosition( pos, boundary ) ) {
        cout << "setParticle position error, tried to set " << pos << endl;
        M_pos = boundary.getPosOutside();
    }
    setAngle( ang );
}



void Particle::moveRelative( SIM::Vector2D relativeDisplacement,
                             SIM::AngRad rotation, SIM::Rectangle boundary )
{
    SIM::Point2D newPos =
        relativeDisplacement.relativeToGlobal( M_pos, M_ang );
    setPosition( newPos, boundary );
//  cerr << "updating new angle " << M_ang <<"+"<< rotation << "=" <<  M_ang + rotation << endl;
    setAngle( M_ang + rotation );
}

void Particle::moveGlobal( SIM::Vector2D globalDisplacement,
                           SIM::AngRad rotation, SIM::Rectangle boundary )
{
    setPosition( M_pos + globalDisplacement, boundary );
    setAngle( M_ang + rotation );
}


void Particle::randomWalk( double maxDistance, SIM::AngRad maxRotation,
                           SIM::Rectangle boundary, bool useProbForRandWalk )
{
//  cerr << "maxDistance " << maxDistance << " maxRotation " << maxRotation << " getProbability " << getProbability() << endl;
    double moveX = maxDistance * (2 * drand48() - 1);
    double moveY = maxDistance * (2 * drand48() - 1);
    double moveAng = maxRotation * (2 * drand48() - 1);
    if( useProbForRandWalk ) {
        moveX *= ( 1 - getProbability() );
        moveY *= ( 1 - getProbability() );
        moveAng *= ( 1 - getProbability() );
    }

    SIM::Vector2D dPos( moveX, moveY );
    SIM::AngRad dAng =  moveAng;
// cerr << "Paricle is randomly moving: " << dPos << " rad: " << dAng << " deg: " <<  Rad2Deg( dAng ) << endl;
    moveGlobal( dPos, dAng, boundary );
}

void Particle::placeRandomly( SIM::Rectangle boundary )
{
    SIM::Vector2D dPos( boundary.getWidth() * drand48(),
                        boundary.getLength() * drand48() );
    setPosition( boundary.getBottomLeft() + dPos,
                 boundary );
    setAngle( TWOPI * drand48() - M_PI );
}



double Particle::getDistanceToPoint( SIM::Point2D p )
{
    return M_pos.getDistanceTo( p );
}

SIM::AngRad Particle::getBearingToPoint( SIM::Point2D p )
{
    return M_pos.getBearingTo(p, M_ang);
}


SIM::Point2D Particle::getNearestPointOnLine( SIM::Line2D l )
{
    // find point on line closest to particle position (M_pos)
    return l.getPointOnLineClosestTo( M_pos );
}

SIM::Point2D Particle::getNearestPointOnLineSeg( SIM::Line2D l )
{
    // find point on line closest to particle position (M_pos)
    return l.getPointOnLineSegClosestTo( M_pos );
}



/*! Overloaded version of the C++ output operator for output.
  \param os output stream to which information should be written
  \param p a Particle which must be printed
  \return output stream containing (x,y) */
ostream& operator << ( ostream &os, Particle p )
{
    return ( os << "( " << p.M_pos << ", " << Rad2Deg(p.M_ang) << ", " << p.getProbability() << " )" );
}




void Particle::resetObservations() {
    for (int i = 0; i < NUM_WORLD_OBJS; i++) {
        M_expectedObs[i] = 0;
        M_seenObs[i] = 0;
    }
}

int Particle::getNumFramesExpected(int i) {
    return M_expectedObs[i];
}

int Particle::getNumFramesNotSeen(int i) {
    return M_seenObs[i];
}

void Particle::incrNumFramesExpected(int i) {
    M_expectedObs[i]++;
}

void Particle::incrNumFramesNotSeen(int i) {
    M_seenObs[i]++;
}

void Particle::incrNumFramesNotSeenForAll() {
    for (int i = 0; i < NUM_WORLD_OBJS; i++)
        M_seenObs[i]++;
}

void Particle::resetNumFramesExpected(int i) {
    M_expectedObs[i] = 0;
}

void Particle::resetNumFramesNotSeen(int i) {
    M_seenObs[i] = 0;
}
