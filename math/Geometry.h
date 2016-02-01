#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <iostream>
#include "vecposition.h"


namespace SIM {

class Point2D;

typedef Point2D Vector2D; /*!< Type definition for two-dimensional vectors. */
typedef double AngRad;    /*!< Type definition for angles in radians. */
typedef double AngDeg;    /*!< Type definition for angles in degrees. */
typedef double Distance;  /*!< Type definition for distance (probably in millimeters) */

//#define EPSILON 0.0001    /*!< Value used for floating point equality tests. */
const double EPSILON = 0.0001;
#define MILLI 1E-3
#define MICRO 1E-6
#define CENTI 1E-2
#define TWOPI (M_PI * 2)
#define PIONTWO (M_PI / 2)

#define RAD_T_DEG  (180.0/ M_PI)
#define DEG_T_RAD  (M_PI/180.0)

// auxiliary numeric functions for determining the
// maximum and minimum of two given double values and the sign of a value
double max ( double d1, double d2 );
double min ( double d1, double d2 );
double square ( double d1 );
double crop ( double val, double minval, double maxval);

/**
 * Function that finds the minmax of three values, x, min limit and
 * max limit...
 */
double minmax( double minx, double x, double maxx );
Point2D minmax( Point2D minP, Point2D P, Point2D maxP );

bool notANumber( double num );

int Sign( double d );
int Round( double d );
int Floor( double d );


AngRad normalizeAngle( AngRad ang );

// various goniometric functions
bool   isAngInInterval     ( AngRad ang,    AngRad angMin, AngRad angMax );
AngRad getBisectorTwoAngles( AngRad angMin, AngRad angMax );


/*****************/
/* Class Point2D */
/*****************/

/*! This class contains an x- and y-coordinate of a point (x,y) as member
  data and methods which operate on this position. */
class Point2D
{
public:
    double x; /*!< x-coordinate of this position */
    double y; /*!< y-coordinate of this position */

public:
    // constructor for Point2D class
    Point2D( double       xx = 0,
             double       yx = 0,
             CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    Point2D operator -  () const;
    Point2D operator +  ( const double  &d ) const;
    Point2D operator +  ( const Point2D &p ) const;
    Point2D operator -  ( const double  &d ) const;
    Point2D operator -  ( const Point2D &p ) const;
    Point2D operator *  ( const double  &d ) const;
    Point2D operator *  ( const Point2D &p ) const;
    Point2D operator /  ( const double  &d ) const;
    Point2D operator /  ( const Point2D &p ) const;
    void    operator =  ( const double  &d );
    void    operator += ( const Point2D &p );
    void    operator += ( const double  &d );
    void    operator -= ( const Point2D &p );
    void    operator -= ( const double  &d );
    void    operator *= ( const Point2D &p );
    void    operator *= ( const double  &d );
    void    operator /= ( const Point2D &p );
    void    operator /= ( const double  &d );
    bool    operator != ( const Point2D &p ) const;
    bool    operator != ( const double  &d ) const;
    bool    operator == ( const Point2D &p ) const;
    bool    operator == ( const double  &d ) const;

    // output operator
    friend std::ostream& operator << ( std::ostream &os,
                                       Point2D p );

    // string conversion for Lua
    const char* __str__();

    // set- and get methods for private member variables
    void   setX( double xx );
    double getX() const;
    void   setY( double yy );
    double getY() const;

    // set- and get methods for derived position information
    void setPoint( double       xx = 0,
                   double       yy = 0,
                   CoordSystemT cs = CARTESIAN );

    double  getDistanceTo( const Point2D p ) const;
    AngRad  getBearingTo ( const Point2D p, const AngRad orient ) const;
    AngRad  getAngleTo   ( const Point2D p ) const;
    Point2D setMagnitude ( double d );
    double  getMagnitude () const;
    AngRad  getDirection () const;

    // comparison methods for positions
    bool isInFrontOf( const Point2D &p ) const;
    bool isInFrontOf( const double  &yy ) const;
    bool isBehind   ( const Point2D &p ) const;
    bool isBehind   ( const double  &yy ) const;
    bool isLeftOf   ( const Point2D &p ) const;
    bool isLeftOf   ( const double  &xx ) const;
    bool isRightOf  ( const Point2D &p ) const;
    bool isRightOf  ( const double  &xx ) const;
    bool isBetweenX ( const Point2D &p1,
                      const Point2D &p2 ) const;
    bool isBetweenX ( const double  &x1,
                      const double  &x2 ) const;

    bool pointIsBetweenX( const Point2D &p1,
                          const Point2D &p2 ) const;

    bool pointIsBetweenY( const Point2D &p1,
                          const Point2D &p2 ) const;

    bool isBetweenY ( const Point2D &p1,
                      const Point2D &p2 ) const;
    bool isBetweenY ( const double  &y1,
                      const double  &y2 ) const;
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
                                    double  frac );
    AngRad  getAngleBetweenPoints ( const Point2D &p1,
                                    const Point2D &p2 );

    // static class methods
    static Point2D getPointFromPolar( double mag,
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
    double x; /*!< x-coordinate of this position */
    double y; /*!< y-coordinate of this position */
    double z; /*!< z-coordinate of this position */

public:
    // constructor for Point3D class
    Point3D( double       xx = 0,
             double       yx = 0,
             double       zx = 0,
             CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    Point3D operator -  () const;
    Point3D operator +  ( const double  &d ) const;
    Point3D operator +  ( const Point3D &p ) const;
    Point3D operator -  ( const double  &d ) const;
    Point3D operator -  ( const Point3D &p ) const;
    Point3D operator *  ( const double  &d ) const;
    Point3D operator *  ( const Point3D &p ) const;
    Point3D operator /  ( const double  &d ) const;
    Point3D operator /  ( const Point3D &p ) const;
    void    operator =  ( const double  &d );
    void    operator += ( const Point3D &p );
    void    operator += ( const double  &d );
    void    operator -= ( const Point3D &p );
    void    operator -= ( const double  &d );
    void    operator *= ( const Point3D &p );
    void    operator *= ( const double  &d );
    void    operator /= ( const Point3D &p );
    void    operator /= ( const double  &d );
    bool    operator != ( const Point3D &p ) const;
    bool    operator != ( const double  &d ) const;
    bool    operator == ( const Point3D &p ) const;
    bool    operator == ( const double  &d ) const;

    // output operator
    friend std::ostream& operator << ( std::ostream &os,
                                       Point3D p );

    // string conversion for Lua
    const char* __str__();

    // set- and get methods for private member variables
    void   setX( double xx );
    double getX() const;
    void   setY( double yy );
    double getY() const;
    void   setZ( double zz );
    double getZ() const;

    // set- and get methods for derived position information
    void setPoint( double       xx = 0,
                   double       yy = 0,
                   double       zz = 0,
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
    friend std::ostream& operator << ( std::ostream &os,
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

    double getWidth         () const;
    double getLength        () const;
    double getDiagonalLength() const;
    double getRight         () const;
    double getLeft          () const;
    double getTop           () const;
    double getBottom        () const;
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
    double m_a; /*!< This is the a coefficient in the line ay + bx + c = 0 */
    double m_b; /*!< This is the b coefficient in the line ay + bx + c = 0 */
    double m_c; /*!< This is the c coefficient in the line ay + bx + c = 0 */

    Point2D start;
    Point2D end;
    bool isLineSegment;

    Point2D center;

    Line2D();
    Line2D( double a, double b, double c );
    Line2D( const Point2D& p1, const Point2D& p2 );
    Line2D( const Point2D& p, const AngRad& ang );

    Line2D operator -  () const;

    // output operator
    friend std::ostream& operator << ( std::ostream &os,
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
    Point2D getPointOnLineSegClosestTo ( Point2D p ) const;
    Point2D getPointOnLineSegClosestTo2( Point2D p ) const;
    bool    intersectWithLineSeg       ( Line2D l, Point2D &pt ) const;


    double  getDistanceToPoint         ( Point2D p ) const;
    bool    isInBetween                ( Point2D p,
                                         Point2D p1,
                                         Point2D p2 ) const;

    // calculate associated variables in the line
    double getYGivenX     ( double x ) const;
    double getXGivenY     ( double y ) const;
    double getACoefficient() const;
    double getBCoefficient() const;
    double getCCoefficient() const;
    double getSlope() const;
    double getYIntercept() const;
    double getXIntercept() const;
    Point2D getStart() const;
    Point2D getEnd() const;


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
    double  m_radius;            /*!< Radius of the circle  */

public:
    // constructors for Circle class
    Circle();
    Circle( const Point2D& pCenter,
            double  radius );

    // output operator
    friend std::ostream& operator << ( std::ostream &os,
                                       Circle c );

    // string conversion for Lua
    const char* __str__();

    // get and set methods
    bool setCircle( const Point2D& pCenter,
                    double  radius );

    bool    setRadius       ( double radius );
    double  getRadius       () const;
    void    setCenter       ( const Point2D& pCenter );
    Point2D getCenter       () const;
    double  getCircumference() const;
    double  getArea         () const;

    // calculate intersection points and area with other circle
    bool   isInside             ( const Point2D& p ) const;
    int    getIntersectionPoints( Circle  c,
                                  Point2D *p1,
                                  Point2D *p2 ) const;
    double getIntersectionArea  ( Circle c ) const;

} ;

} // namespace SIM

#endif
