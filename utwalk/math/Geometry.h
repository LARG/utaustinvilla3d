#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <cmath>
#include <iostream>

using namespace std;

class Point2D;

typedef Point2D Vector2D; /*!< Type definition for two-dimensional vectors. */
typedef float AngRad;    /*!< Type definition for angles in radians. */
typedef float AngDeg;    /*!< Type definition for angles in degrees. */
typedef float Distance;  /*!< Type definition for distance (probably in millimeters) */

//#define EPSILON 0.0001    /*!< Value used for floating point equality tests. */
const double EPSILON = 0.0001;
#define MILLI 1E-3
#define MICRO 1E-6
#define CENTI 1E-2
#define TWOPI M_PI * 2
#define PIONTWO M_PI / 2

#define RAD_T_DEG  180.0/ M_PI
#define DEG_T_RAD  M_PI/180.0

// auxiliary numeric functions for determining the
// maximum and minimum of two given float values and the sign of a value
float max ( float d1, float d2 );
float max ( float d1, float d2, float d3 );
float min ( float d1, float d2 );
float square ( float d1 );
float crop ( float val, float minval, float maxval);

/**
 * Function that finds the minmax of three values, x, min limit and
 * max limit...
 */
float minmax( float minx, float x, float maxx );
Point2D minmax( Point2D minP, Point2D P, Point2D maxP );

bool notANumber( float num );

int Sign( float d );
int Round( float d );
int Floor( float d );

// auxiliary goniometric functions which enable you to
// specify angles in degrees rather than in radians
AngDeg Rad2Deg ( AngRad ang );
AngRad Deg2Rad ( AngDeg ang );

/* Todd: lets not use these so we're not switching back from deg2rad to
   do math and then back from rad2deg for output all the time.
AngDeg Rad2Deg ( AngRad ang );
AngRad Deg2Rad ( AngDeg ang );
float cosDeg  ( AngDeg ang );
float sinDeg  ( AngDeg ang );
float tanDeg  ( AngDeg ang );
AngDeg atanDeg ( float d );
float atan2Deg( float y, float x );
AngDeg acosDeg ( float d );
AngDeg asinDeg ( float d );
*/

AngRad normalizeAngle( AngRad ang );

// various goniometric functions
bool   isAngInInterval     ( AngRad ang,    AngRad angMin, AngRad angMax );
AngRad getBisectorTwoAngles( AngRad angMin, AngRad angMax );

/*! CoordSystem is an enumeration of the different specified coordinate systems.*/
enum CoordSystemT
{
    CARTESIAN,
    POLAR
} ;


/*****************/
/* Class Point2D */
/*****************/

/*! This class contains an x- and y-coordinate of a point (x,y) as member
  data and methods which operate on this position. */
class Point2D
{
public:
    float x; /*!< x-coordinate of this position */
    float y; /*!< y-coordinate of this position */

public:
    // constructor for Point2D class
    Point2D( float       xx = 0,
             float       yx = 0,
             CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    Point2D operator -  () const;
    Point2D operator +  ( const float  &d ) const;
    Point2D operator +  ( const Point2D &p ) const;
    Point2D operator -  ( const float  &d ) const;
    Point2D operator -  ( const Point2D &p ) const;
    Point2D operator *  ( const float  &d ) const;
    Point2D operator *  ( const Point2D &p ) const;
    Point2D operator /  ( const float  &d ) const;
    Point2D operator /  ( const Point2D &p ) const;
    void    operator =  ( const float  &d );
    void    operator += ( const Point2D &p );
    void    operator += ( const float  &d );
    void    operator -= ( const Point2D &p );
    void    operator -= ( const float  &d );
    void    operator *= ( const Point2D &p );
    void    operator *= ( const float  &d );
    void    operator /= ( const Point2D &p );
    void    operator /= ( const float  &d );
    bool    operator != ( const Point2D &p ) const;
    bool    operator != ( const float  &d ) const;
    bool    operator == ( const Point2D &p ) const;
    bool    operator == ( const float  &d ) const;

    // output operator
    friend ostream& operator << ( ostream &os,
                                  Point2D p );

    // string conversion for Lua
    const char* __str__();

    // set- and get methods for private member variables
    void   setX( float xx );
    float getX() const;
    void   setY( float yy );
    float getY() const;

    // set- and get methods for derived position information
    void setPoint( float       xx = 0,
                   float       yy = 0,
                   CoordSystemT cs = CARTESIAN );

    float  getDistanceTo( const Point2D p ) const;
    AngRad  getBearingTo ( const Point2D p, const AngRad orient ) const;
    AngRad  getAngleTo   ( const Point2D p ) const;
    Point2D setMagnitude ( float d );
    float  getMagnitude () const;
    AngRad  getDirection () const;

    // comparison methods for positions
    bool isInFrontOf( const Point2D &p ) const;
    bool isInFrontOf( const float  &yy ) const;
    bool isBehind   ( const Point2D &p ) const;
    bool isBehind   ( const float  &yy ) const;
    bool isLeftOf   ( const Point2D &p ) const;
    bool isLeftOf   ( const float  &xx ) const;
    bool isRightOf  ( const Point2D &p ) const;
    bool isRightOf  ( const float  &xx ) const;
    bool isBetweenX ( const Point2D &p1,
                      const Point2D &p2 ) const;
    bool isBetweenX ( const float  &x1,
                      const float  &x2 ) const;

    bool pointIsBetweenX( const Point2D &p1,
                          const Point2D &p2 ) const;

    bool pointIsBetweenY( const Point2D &p1,
                          const Point2D &p2 ) const;

    bool isBetweenY ( const Point2D &p1,
                      const Point2D &p2 ) const;
    bool isBetweenY ( const float  &y1,
                      const float  &y2 ) const;
    bool isBetweenTwoPoints( const Point2D &p1,
                             const Point2D &p2 ) const;

    // conversion methods for positions
    Point2D normalize       ();
    Point2D rotate          ( AngRad ang );
    Point2D globalToRelative( Point2D origin,
                              AngRad  ang );
    Point2D relativeToGlobal( Point2D origin,
                              AngRad  ang );

    Point2D getPointOnLineFraction( Point2D &p,
                                    float  frac );
    AngRad  getAngleBetweenPoints ( const Point2D &p1,
                                    const Point2D &p2 );

    // static class methods
    static Point2D getPointFromPolar( float mag,
                                      AngRad ang );
} ;




/*****************/
/* Class Point3D */
/*****************/

/*! This class contains an x-.y- and z-coordinate of a point (x,y,z) as member
  data and methods which operate on this position. */
class Point3D
{
public:
    float x; /*!< x-coordinate of this position */
    float y; /*!< y-coordinate of this position */
    float z; /*!< z-coordinate of this position */

public:
    // constructor for Point3D class
    Point3D( float       xx = 0,
             float       yx = 0,
             float       zx = 0,
             CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    Point3D operator -  () const;
    Point3D operator +  ( const float  &d ) const;
    Point3D operator +  ( const Point3D &p ) const;
    Point3D operator -  ( const float  &d ) const;
    Point3D operator -  ( const Point3D &p ) const;
    Point3D operator *  ( const float  &d ) const;
    Point3D operator *  ( const Point3D &p ) const;
    Point3D operator /  ( const float  &d ) const;
    Point3D operator /  ( const Point3D &p ) const;
    void    operator =  ( const float  &d );
    void    operator += ( const Point3D &p );
    void    operator += ( const float  &d );
    void    operator -= ( const Point3D &p );
    void    operator -= ( const float  &d );
    void    operator *= ( const Point3D &p );
    void    operator *= ( const float  &d );
    void    operator /= ( const Point3D &p );
    void    operator /= ( const float  &d );
    bool    operator != ( const Point3D &p ) const;
    bool    operator != ( const float  &d ) const;
    bool    operator == ( const Point3D &p ) const;
    bool    operator == ( const float  &d ) const;

    // output operator
    friend ostream& operator << ( ostream &os,
                                  Point3D p );

    // string conversion for Lua
    const char* __str__();

    // set- and get methods for private member variables
    void   setX( float xx );
    float getX() const;
    void   setY( float yy );
    float getY() const;
    void   setZ( float zz );
    float getZ() const;

    // set- and get methods for derived position information
    void setPoint( float       xx = 0,
                   float       yy = 0,
                   float       zz = 0,
                   CoordSystemT cs = CARTESIAN );
};


/*******************/
/* Class Rectangle */
/*******************/

/*!This class represents a rectangle. A rectangle is defined by two Point2Ds
   the one at the upper left corner and the one at the right bottom. */
class Rectangle
{
private:
    Point2D m_pTopLeft;     /*!< top left position of the rectangle       */
    Point2D m_pBottomRight; /*!< bottom right position of the rectangle   */

public:
    // constructor for Rectangle class
    Rectangle() {};
    Rectangle( Point2D p1, Point2D p2 );

    // checks whether point lies inside the rectangle
    bool isInside( Point2D p ) const;

    // output operator
    friend ostream& operator << ( ostream &os,
                                  Rectangle r );

    // flip operator
    Rectangle operator -    () const;

    // standard get and set methosd
    void    setRectanglePoints( Point2D p1, Point2D p2 );
    void    setTopLeft      ( Point2D p );
    Point2D getTopLeft      () const;
    void    setBottomRight  ( Point2D p );
    Point2D getBottomRight  () const;

    Point2D getBottomLeft   () const;
    Point2D getTopRight     () const;
    Point2D getCenter       () const;

    Point2D getPosOutside   () const;

    float getWidth         () const;
    float getLength        () const;
    float getDiagonalLength() const;
    float getRight         () const;
    float getLeft          () const;
    float getTop           () const;
    float getBottom        () const;
} ;


// holds start/end points for a line
struct LineSegment {
    Point2D start;
    Point2D end;
};


/****************/
/* Class Line2D */
/****************/

/*!This class contains the representation of a line. A line is defined
  by the formula ay + bx + c = 0. */
class Line2D
{
public:
    // a line is defined by the formula: ay + bx + c = 0
    float m_a; /*!< This is the a coefficient in the line ay + bx + c = 0 */
    float m_b; /*!< This is the b coefficient in the line ay + bx + c = 0 */
    float m_c; /*!< This is the c coefficient in the line ay + bx + c = 0 */

    Point2D center;

    Line2D();
    Line2D( float a, float b, float c );
    Line2D( const Point2D& p1, const Point2D& p2 );
    Line2D( const Point2D& p, const AngRad& ang );

    Line2D operator -  () const;

    // output operator
    friend ostream& operator << ( ostream &os,
                                  Line2D l );

    // get intersection points with this line
    Point2D getIntersection            ( Line2D line ) const;
    AngRad getMyAngleWith              ( Line2D line ) const;
    AngRad getAngleToLine              ( Line2D line ) const;
    Point2D getMyIntersection          ( Line2D line ) const;
    //int     getCircleIntersectionPoints( Circle  circle,
    //				       Point2D *pSolution1,
    //			       Point2D *pSolution2 ) const;
    Line2D  getOrthogonalLine          ( Point2D p ) const;
    Point2D getPointOnLineClosestTo    ( Point2D p ) const;
    float  getDistanceToPoint         ( Point2D p ) const;
    bool    isInBetween                ( Point2D p,
                                         Point2D p1,
                                         Point2D p2 ) const;

    // calculate associated variables in the line
    float getYGivenX     ( float x ) const;
    float getXGivenY     ( float y ) const;
    float getACoefficient() const;
    float getBCoefficient() const;
    float getCCoefficient() const;
    float getSlope() const;
    float getYIntercept() const;
    float getXIntercept() const;

    Point2D getCenter() const;

    // static methods to make a line using an easier representation.
    static Line2D makeLineFromTwoPoints       ( Point2D p1,
            Point2D p2 );
    static Line2D makeLineFromPositionAndAngle( Point2D p,
            AngRad  ang );
} ;



/****************/
/* Class Circle */
/****************/

/*!This class represents a circle. A circle is defined by one Point2D
  (which denotes the center) and its radius. */
class Circle
{
private:
    Point2D m_pCenter;            /*!< Center of the circle  */
    float  m_radius;            /*!< Radius of the circle  */

public:
    // constructors for Circle class
    Circle();
    Circle( const Point2D& pCenter,
            float  radius );

    // output operator
    friend ostream& operator << ( ostream &os,
                                  Circle c );

    // string conversion for Lua
    const char* __str__();

    // get and set methods
    bool setCircle( const Point2D& pCenter,
                    float  radius );

    bool    setRadius       ( float radius );
    float  getRadius       () const;
    void    setCenter       ( const Point2D& pCenter );
    Point2D getCenter       () const;
    float  getCircumference() const;
    float  getArea         () const;

    // calculate intersection points and area with other circle
    bool   isInside             ( const Point2D& p ) const;
    int    getIntersectionPoints( Circle  c,
                                  Point2D *p1,
                                  Point2D *p2 ) const;
    float getIntersectionArea  ( Circle c ) const;

} ;


#endif
