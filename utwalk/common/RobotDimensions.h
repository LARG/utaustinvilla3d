/**
 * @file RobotDimensions.h
 *
 * Description of the Dimensions of the Kondo Robot
 *
 * @author Cord Niehaus
 * Edited by Sam Barrett
 */

#ifndef __RobotDimensions_H__
#define __RobotDimensions_H__

#include "math/Vector3.h"

class RobotDimensions
{
public:

    float lengthBetweenLegs;         //!<length between leg joints LL1 and LR1
    float upperLegLength;            //!<length between leg joints LL2 and LL3 in z-direction
    float lowerLegLength;            //!<length between leg joints LL3 and LL4 in z-direction
    float heightLeg5Joint;           //!<height of leg joints LL4 and LR4 of the ground


    float zLegJoint1ToHeadPan;       //!<height offset between LL1 and head pan joint

    Vector3<float> armOffset;             //! The offset of the first left arm joint relative to the middle between the hip joints
    float upperArmLength;            //! The length between the shoulder and the elbow in y-direction
    float lowerArmLength;            //!< height off lower arm starting at arm2/arm3

    float footHeight;

    float motionCycleTime;

    // these values come from Configs/Robot/Nao/robotDimensions.cfg
    RobotDimensions():
        lengthBetweenLegs(100),
        upperLegLength(100),
        lowerLegLength(102.75),
        heightLeg5Joint(45.11),
        zLegJoint1ToHeadPan(211.5),
        armOffset(0,98,185),
        upperArmLength(90),
        lowerArmLength(120.0), // estimated
        footHeight(45.11),
        motionCycleTime(0.010)
    {
    }

//# forward offset head tilt to camera
//48.8

//# height offset head tilt to camera
//23.81

//# head tilt to camera tilt (in radians)
//0.698132

//# forward offset head tilt to upper camera
//53.9

//# height offset head tilt to upper camera
//67.9

//# head tilt to upper camera tilt (in radians)
//0.0

//# image recording time (in s, for simulation)
//0

//# image recording delay (in s, for simulation)
//0
};

#endif
