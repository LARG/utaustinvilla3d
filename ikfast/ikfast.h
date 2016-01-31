#ifndef IKFAST_H
#define IKFAST_H
#include <vector>

typedef std::vector<double> joints_t;

/**
 * Computes inverse kinomatics for the left foot with respect to the torso.
 *
 * \param[in] eetrans array of desired translation coordinates. Eg, double trans[3] = {x, y, z};
 * \param[in] eerot   3x3 rotation matrix, in row-major order, representing the desired rotation. Eg,
 *                    double rot[9] = { r00, r01, r02,
 *                                      r10, r11, r12,
 *                                      r20, r21, r22 };
 * \param[out] solutions  Vector of proposed solutions, some of which may be out of bounds.
 *                        Returned angles are in radians.
 * \return  true iff solution(s) were found, false otherwise.
 */
bool ikfast_left_foot_ik(const double *eetrans, const double *eerot, std::vector<joints_t> &solutions);

/**
 * Computes inverse kinomatics for the right foot with respect to the torso.
 *
 * \param[in] eetrans array of desired translation coordinates. Eg, double trans[3] = {x, y, z};
 * \param[in] eerot   3x3 rotation matrix, in row-major order, representing the desired rotation. Eg,
 *                    double rot[9] = { r00, r01, r02,
 *                                      r10, r11, r12,
 *                                      r20, r21, r22 };
 * \param[out] solutions  Vector of proposed solutions, some of which may be out of bounds.
 *                        Returned angles are in radians.
 * \return  true iff solution(s) were found, false otherwise.
 */
bool ikfast_right_foot_ik(const double *eetrans, const double *eerot, std::vector<joints_t> &solutions);

/**
 * Computes the forward kinematics for the left foot with respect to the torso;
 *
 * \param[in]  j        array of the joint angles in radians. Eg, double j[6] = {j1, j2, j3, j4, j5, j6};
 * \param[out] eetrans  array of computed translation coordinates. Eg, double trans[3] = {x, y, z};
 * \param[out] eerot    3x3 rotation matrix, in row-major order, representing the rotation. Eg,
 *                      double rot[9] = { r00, r01, r02,
 *                                        r10, r11, r12,
 *                                        r20, r21, r22 };
 */
void ikfast_left_foot_fk(const double* j, double* eetrans, double* eerot);

/**
 * Computes the forward kinematics for the right foot with respect to the torso;
 *
 * \param[in]  j        array of the joint angles in radians. Eg, double j[6] = {j1, j2, j3, j4, j5, j6};
 * \param[out] eetrans  array of computed translation coordinates. Eg, double trans[3] = {x, y, z};
 * \param[out] eerot    3x3 rotation matrix, in row-major order, representing the rotation. Eg,
 *                      double rot[9] = { r00, r01, r02,
 *                                        r10, r11, r12,
 *                                        r20, r21, r22 };
 */
void ikfast_right_foot_fk(const double* j, double* eetrans, double* eerot);


#endif // IKFAST_H
