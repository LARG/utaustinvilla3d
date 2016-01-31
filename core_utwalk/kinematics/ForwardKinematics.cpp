#include "ForwardKinematics.h"

void ForwardKinematics::calculateRelativePose(float *joint_angles, float angleX, float angleY, Pose3D *rel_parts, const RobotDimensions &dimensions) {
    const Vector3<float> axis(angleX, angleY, 0);
    // Rotate torso by filtered tilt and roll
    rel_parts[BodyPart::torso] = Pose3D(0,0,0).rotate(axis);

    // Head
    rel_parts[BodyPart::neck] = Pose3D(rel_parts[BodyPart::torso])
                                .translate(0, 0, dimensions.zLegJoint1ToHeadPan)
                                .rotateZ(joint_angles[HeadYaw]);
    rel_parts[BodyPart::head] = Pose3D(rel_parts[BodyPart::neck])
                                .rotateY(-joint_angles[HeadPitch]);


    // Body
    for(int side = 0; side < 2; side++)
    {
        bool left = side == 0;
        int sign = left ? -1 : 1;
        // Decide on left or right arm/leg
        BodyPart::Part pelvis = left ? BodyPart::left_pelvis : BodyPart::right_pelvis;
        Joint leg0 = left ? LHipYawPitch : RHipYawPitch;
        BodyPart::Part shoulder = left ? BodyPart::left_shoulder : BodyPart::right_shoulder;
        Joint arm0 = left ? LShoulderPitch : RShoulderPitch;

        //Arms
        rel_parts[shoulder + 0] = Pose3D(rel_parts[BodyPart::torso])
                                  .translate(dimensions.armOffset.x, dimensions.armOffset.y * -sign, dimensions.armOffset.z)
                                  .rotateY(-joint_angles[arm0 + 0]);
        rel_parts[shoulder + 1] = Pose3D(rel_parts[shoulder + 0])
                                  .rotateZ(joint_angles[arm0 + 1] * -sign);
        rel_parts[shoulder + 2] = Pose3D(rel_parts[shoulder + 1])
                                  .translate(dimensions.upperArmLength, 0, 0)
                                  .rotateX(joint_angles[arm0 + 2] * -sign);
        rel_parts[shoulder + 3] = Pose3D(rel_parts[shoulder + 2])
                                  .rotateZ(joint_angles[arm0 + 3] * -sign);
        // Legs
        rel_parts[pelvis + 0] = Pose3D(rel_parts[BodyPart::torso])
                                .translate(0, dimensions.lengthBetweenLegs / 2.0f * -sign, 0)
                                .rotateX(-M_PI_4 * sign)
                                .rotateZ(joint_angles[leg0 + 0] * sign);
        rel_parts[pelvis + 1] = Pose3D(rel_parts[pelvis + 0])
                                .rotateX((joint_angles[leg0 + 1] + M_PI_4) * sign);
        rel_parts[pelvis + 2] = Pose3D(rel_parts[pelvis + 1])
                                .rotateY(joint_angles[leg0 + 2]);
        rel_parts[pelvis + 3] = Pose3D(rel_parts[pelvis + 2])
                                .translate(0, 0, -dimensions.upperLegLength)
                                .rotateY(joint_angles[leg0 + 3]);
        rel_parts[pelvis + 4] = Pose3D(rel_parts[pelvis + 3])
                                .translate(0, 0, -dimensions.lowerLegLength)
                                .rotateY(joint_angles[leg0 + 4]);
        rel_parts[pelvis + 5] = Pose3D(rel_parts[pelvis + 4])
                                .rotateX(joint_angles[leg0 + 5] * sign);
        rel_parts[pelvis + 6] = Pose3D(rel_parts[pelvis + 5])
                                .translate(0,0, -dimensions.footHeight);
    }
}


void ForwardKinematics::calculateAbsolutePose(Pose3D *rel_parts, Pose3D *abs_parts) {
    // Find lowest z
    float lowestz = 10000;
    for (int i=0; i < BodyPart::NUM_PARTS; i++) {
        if (rel_parts[i].translation.z < lowestz) {
            lowestz = rel_parts[i].translation.z;
        }
    }

    // Create absolute body parts (relative raised up);
    float zOffset = 0.0 - lowestz;
    for (int i=0; i < BodyPart::NUM_PARTS; i++) {
        abs_parts[i] = rel_parts[i];
        abs_parts[i].translation.z += zOffset;
    }

}

void ForwardKinematics::calculateCoM(Pose3D *abs_parts, Vector3<float> &center_of_mass, const MassCalibration &mass_calibration) {
    float total_mass = 0;
    center_of_mass = Vector3<float> (0,0,0);

    for(int i = 0; i < BodyPart::NUM_PARTS; i++)
    {
        const MassCalibration::MassInfo& limb(mass_calibration.masses[i]);
        total_mass += limb.mass;
        center_of_mass += (abs_parts[i] * limb.offset) * limb.mass;
    }
    center_of_mass /= total_mass;
}
