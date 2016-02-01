#ifndef FORWARDKINEMATICS_GJA13VPQ
#define FORWARDKINEMATICS_GJA13VPQ

#include <math/Pose3D.h>
#include <math/Vector3.h>
#include <common/MassCalibration.h>
#include <common/RobotDimensions.h>

namespace ForwardKinematics {
void calculateRelativePose(float *joint_angles, float angleX, float angleY, Pose3D *rel_parts,const RobotDimensions &dimensions);
void calculateAbsolutePose(Pose3D *rel_parts, Pose3D *abs_parts);
void calculateCoM(Pose3D *abs_parts, Vector3<float> &center_of_mass, const MassCalibration &mass_calibration); /**< Calculates the position of the center of mass relative to the robot's origin. */
}

#endif /* end of include guard: FORWARDKINEMATICS_GJA13VPQ */
