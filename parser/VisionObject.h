#ifndef _VISION_OBJECT_H
#define _VISION_OBJECT_H


#include "../math/vecposition.h"

struct VisionObject {

    VisionObject( double r, double theta, double phi, int id_ ) {
        polar.setX( r );
        polar.setY( theta );
        polar.setZ( phi );
        id = id_;
    }

    VisionObject( double r1, double theta1, double phi1, double r2, double theta2, double phi2, int id_ ) {
        polar.setX( r1 );
        polar.setY( theta1 );
        polar.setZ( phi1 );

        polar2.setX( r2 );
        polar2.setY( theta2 );
        polar2.setZ( phi2 );
        id = id_;
    }

    VisionObject() {
        polar.setVecPosition( 0, 0, 0 );
        id = -1;
    }

    VecPosition polar;
    VecPosition polar2;
    int id; // unique id for all objects that can ever be seen

};

#endif
