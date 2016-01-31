/**
* \file Matrix.h
* Template classes for various 3x3 matrices.
* \author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* \author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a>
* \author Max Risler
*/

#ifndef RotationMatrix_H
#define RotationMatrix_H

#include "Matrix3x3.h"

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3x3<float>
{
public:
    /**
     * Default constructor.
     */
    RotationMatrix() {}

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
     */
    RotationMatrix(const Vector3<float>& c0, const Vector3<float>& c1,
                   const Vector3<float>& c2);

    /**
     * Assignment operator.
     *
     * \param  other  The other matrix that is assigned to this one
     * \return        A reference to this object after the assignment.
     */
    RotationMatrix& operator=(const Matrix3x3<float>& other)
    {
        c[0] = other.c[0];
        c[1] = other.c[1];
        c[2] = other.c[2];
        return *this;
    }

    /**
     * Copy constructor.
     *
     * \param  other  The other matrix that is copied to this one
     */
    RotationMatrix(const Matrix3x3<float>& other)
    {
        *this = other;
    }

    /**
     * RotationMatrix from RPY-angles.
     *   Roll  rotates along z axis,
     *   Pitch rotates along y axis,
     *   Yaw   rotates along x axis
     *
     *   R(roll,pitch,yaw) = R(z,roll)*R(y,pitch)*R(x,yaw)
     *
     * \see  "Robotik 1 Ausgabe Sommersemester 2001" by Prof. Dr. O. von Stryk
     * \attention  RPY-angles are not clearly defined!
     */
    RotationMatrix(float yaw, float pitch, float roll);

    /**
     * RotationMatrix from rotation around any axis.
     * \param axis The axis.
     * \param angle The angle to rotate around the axis.
     */
    RotationMatrix(const Vector3<float>& axis, float angle);

    /**
     * RotationMatrix from rotation around any axis with an angle given as the length of the axis.
     * \param axis The axis.
     */
    RotationMatrix(const Vector3<float>& axis);

    /**
     * Invert the matrix.
     *
     * \note: Inverted rotation matrix is transposed matrix.
     */
    RotationMatrix invert() const
    {
        return transpose();
    }

    /**
     * Rotation around the x-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateX(const float angle);

    /**
     * Rotation around the y-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateY(const float angle);

    /**
     * Rotation around the z-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix& rotateZ(const float angle);

    /**
     * Get the x-angle of a RotationMatrix.
     *
     * \return  The angle around the x-axis between the original
     *          and the rotated z-axis projected on the y-z-plane
     */
    float getXAngle() const;

    /**
     * Get the y-angle of a RotationMatrix.
     *
     * \return  The angle around the y-axis between the original
     *          and the rotated x-axis projected on the x-z-plane
     */
    float getYAngle() const;

    /**
     * Get the z-angle of a RotationMatrix.
     *
     * \return  The angle around the z-axis between the original
     *          and the rotated x-axis projected on the x-y-plane
     */
    float getZAngle() const;

    /**
     * Create and return a RotationMatrix, rotated around x-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationX(const float angle)
    {
        return RotationMatrix().rotateX(angle);
    }

    /**
     * Create and return a RotationMatrix, rotated around y-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationY(const float angle)
    {
        return RotationMatrix().rotateY(angle);
    }

    /**
     * Create and return a RotationMatrix, rotated around z-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix fromRotationZ(const float angle)
    {
        return RotationMatrix().rotateZ(angle);
    }

    void getAngleAxis(Vector3<float>& axis, float& angle) const;

    /**
    * Converts the rotation matrix into the single vector format.
    * @return The rotation matrix as angleAxis.
    */
    Vector3<float> getAngleAxis() const;
};

#endif // RotationMatrix_H
