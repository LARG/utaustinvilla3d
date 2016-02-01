#include "vecposition.h"

using namespace std;
/////////////////////////////////////////////////////////////////////

/*! Constructor for the VecPosition class. When the supplied
    Coordinate System type equals CARTESIAN, the arguments x and y
    denote the x- and y-coordinates of the new position. When it
    equals POLAR however, the arguments x and y denote the polar
    coordinates of the new position; in this case x is thus equal to
    the distance r from the origin and y is equal to the angle phi
    that the polar vector makes with the x-axis.
    \param x the x-coordinate of the new position when cs == CARTESIAN; the
    distance of the new position from the origin when cs = POLAR
    \param y the y-coordinate of the new position when cs = CARTESIAN; the
    angle that the polar vector makes with the x-axis when cs = POLAR
    \param cs a CoordSystemT indicating whether x and y denote cartesian
    coordinates or polar coordinates
    \return the VecPosition corresponding to the given arguments */
VecPosition::VecPosition(double x, double y, double z, CoordSystemT cs) {

    setVecPosition(x, y, z, cs );
}

ostream& operator <<( ostream &os, const VecPosition &v ) {

    return ( os << "( " << v.m_x << ", " << v.m_y << ", " << v.m_z << " )" );
}

/*! This method writes the current VecPosition to standard output. It
    can also print a polar representation of the current VecPosition.

    \param cs a CoordSystemtT indicating whether a POLAR or CARTESIAN
    representation of the current VecPosition should be printed */
void VecPosition::show( CoordSystemT cs ) {

    if( cs == CARTESIAN )
        cout << *this << endl;
    else
        cout << "( r: " << getMagnitude() << ", theta: " << getTheta( ) << ", phi: " << getPhi() << "  )";
}


/*! This method (re)sets the coordinates of the current
    VecPosition. The given coordinates can either be polar or
    Cartesian coordinates. This is indicated by the value of the third
    argument.

    \param dX a double value indicating either a new Cartesian
    x-coordinate when cs=CARTESIAN or a new polar r-coordinate
    (distance) when cs=POLAR

    \param dY a double value indicating either a new Cartesian
    y-coordinate when cs=CARTESIAN or a new polar phi-coordinate
    (angle) when cs=POLAR

    \param cs a CoordSystemT indicating whether x and y denote
    cartesian coordinates or polar coordinates */
void VecPosition::setVecPosition( double dX, double dY, double dZ, CoordSystemT cs) {

    if( cs == CARTESIAN ) {

        m_x = dX;
        m_y = dY;
        m_z = dZ;
    }
    else
        *this = getVecPositionFromPolar( dX, dY, dZ );
}

/*! This method adjusts the coordinates of the current VecPosition in
    such a way that the magnitude of the corresponding vector equals
    the double value which is supplied as an argument. It thus scales
    the vector to a given length by multiplying both the x- and
    y-coordinates by the quotient of the argument and the current
    magnitude. This changes the VecPosition itself.

    \param d a double value representing a new magnitude

    \return the result of scaling the vector corresponding with the
    current VecPosition to the given magnitude thus yielding a
    different VecPosition */
VecPosition VecPosition::setMagnitude( double d )
{
    if( getMagnitude( ) > EPSILON )
        ( *this ) *= ( d / getMagnitude( ) );

    return ( *this );
}

/*! This method determines the magnitude (length) of the vector
    corresponding with the current VecPosition using the formula of
    Pythagoras.

    \return the length of the vector corresponding with the current
    VecPosition */
double VecPosition::getMagnitude( ) const {

    return ( sqrt( m_x * m_x + m_y * m_y + m_z * m_z) );
}

/*! This method determines the direction of the vector corresponding
    with the current VecPosition (the phi-coordinate in polar
    representation) using the arc tangent function. Note that the
    signs of x and y have to be taken into account in order to
    determine the correct quadrant.

    \return the direction in degrees of the vector corresponding with
    the current VecPosition */
double VecPosition::getTheta( ) const {

    return ( atan2Deg( m_y, m_x ) );
}

/*! This method determines the direction of the vector corresponding
    with the current VecPosition (the phi-coordinate in polar
    representation) using the arc tangent function. Note that the
    signs of x and y have to be taken into account in order to
    determine the correct quadrant.

    \return the direction in degrees of the vector corresponding with
    the current VecPosition */
double VecPosition::getPhi( ) const {

    return ( atan2Deg( m_z, sqrt(m_x * m_x + m_y * m_y)) );
}

VecPosition VecPosition::normalize( ) {

    return ( setMagnitude( 1.0 ) );
}

VecPosition VecPosition::getVecPositionFromPolar(double r, double theta, double phi) {

    // cos(phi) = x/r <=> x = r*cos(phi); sin(phi) = y/r <=> y = r*sin(phi)

    double x = r * cosDeg(phi) * cosDeg(theta);
    double y = r * cosDeg(phi) * sinDeg(theta);
    double z = r * sinDeg(phi);

    return ( VecPosition(x, y, z) );
}

VecPosition VecPosition::getCartesianFromPolar() {

    VecPosition cartesian = getVecPositionFromPolar(m_x, m_y, m_z);

    return cartesian;
}


VecPosition VecPosition::getPolarFromCartesian() {
    VecPosition polar( getMagnitude(), getTheta(), getPhi() );
    return polar;
}



/*! This method normalizes an angle. This means that the resulting
    angle lies between -180 and 180 degrees.

    \param angle the angle which must be normalized

    \return the result of normalizing the given angle */
double VecPosition::normalizeAngle( double angle ) {

    while( angle > 180.0  ) angle -= 360.0;
    while( angle < -180.0 ) angle += 360.0;

    return ( angle );
}


VecPosition VecPosition::translate(VecPosition t) {

    VecPosition res = (*this) + t;
    return res;
}

VecPosition VecPosition::rotateAboutX(double angle) {

    double cos = cosDeg(angle);
    double sin = sinDeg(angle);

    double newX, newY, newZ;

    newX = m_x;
    newY = m_y * cos + m_z * sin;
    newZ = -m_y * sin + m_z * cos;

    return VecPosition(newX, newY, newZ);
}

VecPosition VecPosition::rotateAboutY(double angle) {

    double cos = cosDeg(angle);
    double sin = sinDeg(angle);

    double newX, newY, newZ;

    newX = m_x * cos - m_z * sin;
    newY = m_y;
    newZ = m_x * sin + m_z * cos;

    return VecPosition(newX, newY, newZ);
}

VecPosition VecPosition::rotateAboutZ(double angle) {

    double cos = cosDeg(angle);
    double sin = sinDeg(angle);

    double newX, newY, newZ;

    newX = m_x * cos + m_y * sin;
    newY = -m_x * sin + m_y * cos;
    newZ = m_z;

    return VecPosition(newX, newY, newZ);
}

double VecPosition::getAngleWithVector(VecPosition v) {

    double d = dotProduct(v);
    double m1 = getMagnitude();
    double m2 = v.getMagnitude();

    if(m1 < EPSILON || m2 < EPSILON) {
        return 0;
    }

    return acosDeg(d / (m1 * m2));
}


VecPosition VecPosition::crossProduct(VecPosition v) {

    double newX, newY, newZ;

    newX = (m_y * v.m_z) - (m_z * v.m_y);
    newY = (m_z * v.m_x) - (m_x * v.m_z);
    newZ = (m_x * v.m_y) - (m_y * v.m_x);

    return VecPosition(newX, newY, newZ);
}
