#ifndef VECPOSITION_H
#define VECPOSITION_H

#include "math.h"       // needed for M_PI constant
#include <string.h>       // needed for string
#include <iostream>

#include "../headers/headers.h"

enum CoordSystemT {
    CARTESIAN = 0,
    POLAR = 1
};

inline double Rad2Deg( double x )
{
    return ( x * 180 / M_PI );
}

inline double Deg2Rad( double x )
{
    return ( x * M_PI / 180 );
}

inline double cosDeg( double x )
{
    return ( cos( Deg2Rad( x ) ) );
}

inline double sinDeg( double x )
{
    return ( sin( Deg2Rad( x ) ) );
}

inline double tanDeg( double x )
{
    return ( tan( Deg2Rad( x ) ) );
}

inline double atanDeg( double x )
{
    return ( Rad2Deg( atan( x ) ) );
}

inline double atan2Deg( double x, double y )
{
    if( fabs( x ) < EPSILON && fabs( y ) < EPSILON )
        return ( 0.0 );

    return ( Rad2Deg( atan2( x, y ) ) );
}

inline double acosDeg( double x )
{
    if( x >= 1 )
        return ( 0.0 );
    else if( x <= -1 )
        return ( 180.0 );

    return ( Rad2Deg( acos( x ) ) );
}

inline double asinDeg( double x )
{
    if( x >= 1 )
        return ( 90.0 );
    else if ( x <= -1 )
        return ( -90.0 );

    return ( Rad2Deg( asin( x ) ) );
}


class VecPosition {

    // private member data
private:

    double m_x;   /*!< x-coordinate of this position */
    double m_y;   /*!< y-coordinate of this position */
    double m_z;   /*!< z-coordinate of this position */

    // public methods
public:
    // constructor for VecPosition class
    VecPosition(double vx = 0, double vy = 0, double vz = 0, CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    /*
       VecPosition        operator -             (                                );
       VecPosition        operator +             ( const double      &d           );
       VecPosition        operator +             ( const VecPosition &p           );
       VecPosition        operator -             ( const double      &d           );
       VecPosition        operator -             ( const VecPosition &p           );
       VecPosition        operator *             ( const double      &d           );
       VecPosition        operator *             ( const VecPosition &p           );
       VecPosition        operator /             ( const double      &d           );
       VecPosition        operator /             ( const VecPosition &p           );
       void               operator =             ( const double      &d           );
       void               operator +=            ( const VecPosition &p           );
       void               operator +=            ( const double      &d           );
       void               operator -=            ( const VecPosition &p           );
       void               operator -=            ( const double      &d           );
       void               operator *=            ( const VecPosition &p           );
       void               operator *=            ( const double      &d           );
       void               operator /=            ( const VecPosition &p           );
       void               operator /=            ( const double      &d           );
       bool               operator !=            ( const VecPosition &p           );
       bool               operator !=            ( const double      &d           );
       bool               operator ==            ( const VecPosition &p           );
       bool               operator ==            ( const double      &d           );
       */
    inline VecPosition operator - ( ) const {

        return ( VecPosition( -m_x, -m_y, -m_z ) );
    }

    inline VecPosition operator + ( const double &d ) const {

        return ( VecPosition( m_x + d, m_y + d, m_z + d ) );
    }

    inline VecPosition operator + ( const VecPosition &p ) const {

        return ( VecPosition( m_x + p.m_x, m_y + p.m_y, m_z + p.m_z ) );
    }

    inline VecPosition operator - ( const double &d ) const {

        return ( VecPosition( m_x - d, m_y - d, m_z - d ) );
    }

    inline VecPosition operator - ( const VecPosition &p ) const {

        return ( VecPosition( m_x - p.m_x, m_y - p.m_y, m_z - p.m_z ) );
    }

    inline VecPosition operator * ( const double &d  ) const {

        return ( VecPosition( m_x * d, m_y * d, m_z * d  ) );
    }

    inline VecPosition operator * ( const VecPosition &p ) const {

        return ( VecPosition( m_x * p.m_x, m_y * p.m_y, m_z * p.m_z ) );
    }

    inline VecPosition operator / ( const double &d ) const {

        return ( VecPosition( m_x / d, m_y / d, m_z / d  ) );
    }

    inline VecPosition operator / ( const VecPosition &p ) const {

        return ( VecPosition( m_x / p.m_x, m_y / p.m_y, m_z / p.m_z ) );
    }

    inline void operator = ( const double &d ) {

        m_x = d;
        m_y = d;
        m_z = d;
    }

    inline void operator +=( const VecPosition &p ) {

        m_x += p.m_x;
        m_y += p.m_y;
        m_z += p.m_z;
    }

    inline void operator += ( const double &d ) {

        m_x += d;
        m_y += d;
        m_z += d;
    }

    inline void operator -=( const VecPosition &p ) {

        m_x -= p.m_x;
        m_y -= p.m_y;
        m_z -= p.m_z;
    }

    inline void operator -=( const double &d ) {

        m_x -= d;
        m_y -= d;
        m_z -= d;
    }

    inline void operator *=( const VecPosition &p ) {

        m_x *= p.m_x;
        m_y *= p.m_y;
        m_z *= p.m_z;
    }

    inline void operator *=( const double &d ) {

        m_x *= d;
        m_y *= d;
        m_z *= d;
    }

    inline void operator /=( const VecPosition &p ) {

        m_x /= p.m_x;
        m_y /= p.m_y;
        m_z /= p.m_z;
    }

    inline void operator /=( const double &d ) {

        m_x /= d;
        m_y /= d;
        m_z /= d;
    }

    inline bool operator !=( const VecPosition &p ) {

        return ( ( m_x != p.m_x ) || ( m_y != p.m_y ) || (m_z != p.m_z) );
    }

    inline bool operator !=( const double &d ) {

        return ( ( m_x != d ) || ( m_y != d ) || (m_z != d) );
    }

    inline bool operator ==( const VecPosition &p ) {

        return ( ( m_x == p.m_x ) && ( m_y == p.m_y ) && (m_z == p.m_z) );
    }

    inline bool operator ==( const double &d ) {

        return ( ( m_x == d ) && ( m_y == d ) && (m_z == d));
    }




    // methods for producing output
    friend std::ostream&    operator <<            ( std::ostream           &os,
            const VecPosition       &p            );

    void               show                   ( CoordSystemT      cs =CARTESIAN);

    // set- and get methods for private member variables
    /*
       bool               setX                   ( double            dX           );
       double             getX                   (                          ) const;
       bool               setY                   ( double            dY           );
       double             getY                   (                          ) const;
       bool               setZ                   ( double            dZ           );
       double             getZ                   (                          ) const;
       */


    inline bool setX( double dX ) {
        m_x = dX;
        return ( true );
    }

    inline double getX( ) const {

        return ( m_x );
    }

    inline bool setY( double dY ) {

        m_y = dY;
        return ( true );
    }

    inline double getY( ) const {

        return ( m_y );
    }

    inline bool setZ( double dZ ) {

        m_z = dZ;
        return ( true );
    }

    inline double getZ( ) const {

        return ( m_z );
    }



    // set- and get methods for derived position information
    void               setVecPosition         ( double            dX = 0,
            double            dY = 0,
            double            dZ = 0,
            CoordSystemT      cs =CARTESIAN);

    //  double             getDistanceTo          ( const VecPosition p            );
    inline double getDistanceTo( const VecPosition p ) {

        return ( ( *this - p ).getMagnitude( ) );
    }


    VecPosition        setMagnitude           ( double            d            );
    double             getMagnitude           (                          ) const;


    double             getTheta           (                          ) const;
    double             getPhi           (                          ) const;

    VecPosition        normalize              (                                );

    //    double             getAngleBetweenPoints  ( const VecPosition &p1,
    //				              const VecPosition &p2          );

    inline double getAngleBetweenPoints( const VecPosition &p1,
                                         const VecPosition &p2 ) {

        VecPosition v1 = *this - p1;
        VecPosition v2 = *this - p2;
        VecPosition vec = v1 * v2;
        double dotProduct = vec.getX() + vec.getY() + vec.getZ();
        return fabs( normalizeAngle( acosDeg( dotProduct / v1.getMagnitude() / v2.getMagnitude() ) ) );
    }

    //static class methods
    static VecPosition getVecPositionFromPolar(double r, double theta, double phi);

    VecPosition getCartesianFromPolar();
    VecPosition getPolarFromCartesian();

    static double     normalizeAngle         ( double            angle        );

    VecPosition translate(VecPosition t);
    VecPosition rotateAboutX(double angle);
    VecPosition rotateAboutY(double angle);
    VecPosition rotateAboutZ(double angle);
    //    double dotProduct(VecPosition v);
    inline double dotProduct(const VecPosition& v) {

        double result = (m_x * v.m_x) + (m_y * v.m_y) + (m_z * v.m_z);

        return result;
    }

    double getAngleWithVector(VecPosition v);
    VecPosition crossProduct(VecPosition v);

    // Project this vector onto the target vector.
    inline VecPosition project(VecPosition t) {
        t.normalize();
        return t * dotProduct(t);
    }


};

inline VecPosition getDirectionVector( double angleRad ) {
    return VecPosition( cos(angleRad), sin(angleRad), 0 );
}


#endif // VECPOSITION_H

