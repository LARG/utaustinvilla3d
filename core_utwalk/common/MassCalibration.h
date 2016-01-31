/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#ifndef __MassCalibration_H__
#define __MassCalibration_H__

#include "math/Vector3.h"
#include "RobotInfo.h"

class MassCalibration
{
public:

    /**
    * Information on the mass distribution of a limb of the robot.
    */
    class MassInfo
    {
    public:
        float mass; /**< The mass of this limb. */
        Vector3<float> offset; /**< The offset of the center of mass of this limb relative to its hinge. */

        /**
        * Default constructor.
        */
        MassInfo() : mass(0), offset() {}

        void set(float m, float x, float y, float z)
        {
            mass = m;
            offset = Vector3<float>(x,y,z);
        }
    };

    MassInfo masses[BodyPart::NUM_PARTS]; /**< Information on the mass distribution of all joints. */

    MassCalibration()
    {
        masses[BodyPart::neck].set(59.59,-0.03,0.18,-25.73);
        masses[BodyPart::head].set(476.71,3.83,-0.93,51.56);
        masses[BodyPart::left_shoulder].set(69.84,-1.78,-25.07,0.19);
        masses[BodyPart::left_bicep].set(121.66,20.67,3.88,3.62);
        masses[BodyPart::left_elbow].set(59.59,-25.73,-0.01,-0.2);
        masses[BodyPart::left_forearm].set(112.82,69.92,0.96,-1.14);
        masses[BodyPart::right_shoulder].set(69.84,-1.78,25.07,0.19);
        masses[BodyPart::right_bicep].set(121.66,20.67,-3.88,3.62);
        masses[BodyPart::right_elbow].set(59.59,-25.73,0.01,-0.2);
        masses[BodyPart::right_forearm].set(112.82,69.92,-0.96,-1.14);
        masses[BodyPart::left_pelvis].set(72.44,-7.17,-11.87,27.05);
        masses[BodyPart::left_hip].set(135.3,-16.49,0.29,-4.75);
        masses[BodyPart::left_thigh].set(397.98,1.31,2.01,-53.86);
        masses[BodyPart::left_tibia].set(297.06,4.71,2.1,-48.91);
        masses[BodyPart::left_ankle].set(138.92,1.42,0.28,6.38);
        masses[BodyPart::left_foot].set(163.04,24.89,3.3,-32.08);
        masses[BodyPart::left_bottom_foot].set(0,0,0,0);
        masses[BodyPart::right_pelvis].set(72.44,-7.17,11.87,27.05);
        masses[BodyPart::right_hip].set(135.3,-16.49,-0.29,-4.75);
        masses[BodyPart::right_thigh].set(397.98,1.31,-2.01,-53.86);
        masses[BodyPart::right_tibia].set(297.06,4.71,-2.1,-48.91);
        masses[BodyPart::right_ankle].set(138.92,1.42,-0.28,6.38);
        masses[BodyPart::right_foot].set(163.04,24.89,-3.3,-32.08);
        masses[BodyPart::right_bottom_foot].set(0,0,0,0);
        masses[BodyPart::torso].set(1026.28,-4.80,0.06,127.27); //z=85+42.27

    }
};
#endif
