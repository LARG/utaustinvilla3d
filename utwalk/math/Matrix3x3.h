/**
* @file Matrix3x3.h
* Contains template class Matrix5x5 of type V
* @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
* @author Colin Graf
*/

#ifndef Matrix3x3_H
#define Matrix3x3_H

#include "Vector3.h"
#include "Matrix2x2.h"

template <class V> class Matrix3x3;
template <class V> class Matrix2x3;
template <class V> class Matrix3x2;

/**
 * This class represents a 3x3-matrix
 *
 */
template <class V> class Matrix3x3
{
public:
    /**
     * The columns of the matrix
     */
    Vector3<V> c[3];

    /**
     * Default constructor.
     */
    Matrix3x3<V>()
    {
        c[0]=Vector3<V>(1,0,0);
        c[1]=Vector3<V>(0,1,0);
        c[2]=Vector3<V>(0,0,1);
    }

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
    */
    Matrix3x3<V>(
        const Vector3<V>& c0,
        const Vector3<V>& c1,
        const Vector3<V>& c2
    )
    {
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }

    /**
     * Anti-lateral thinking constructor.
     */
    Matrix3x3<V>(
        const V& a11, const V& a12, const V& a13,
        const V& a21, const V& a22, const V& a23,
        const V& a31, const V& a32, const V& a33)
    {
        c[0].x = a11;
        c[1].x = a12;
        c[2].x = a13;
        c[0].y = a21;
        c[1].y = a22;
        c[2].y = a23;
        c[0].z = a31;
        c[1].z = a32;
        c[2].z = a33;
    }

    /**
     * Assignment operator.
     *
     * \param  other   The other matrix that is assigned to this one
     * \return         A reference to this object after the assignment.
    */
    Matrix3x3<V>& operator=(const Matrix3x3<V>& other)
    {
        c[0] = other.c[0];
        c[1] = other.c[1];
        c[2] = other.c[2];
        return *this;
    }

    /**
     * Copy constructor.
     *
     * \param other The other matrix that is copied to this one
     */
    Matrix3x3<V>(const Matrix3x3<V>& other)
    {
        *this = other;
    }

    /**
     * Adds this matrix with another matrix.
     *
     * \param  other  The matrix this one is added to
     * \return         A new matrix containing the result
     *                 of the calculation.
    */
    Matrix3x3<V> operator+(const Matrix3x3<V>& other) const
    {
        return Matrix3x3<V>(
                   (*this).c[0]+other.c[0],
                   (*this).c[1]+other.c[1],
                   (*this).c[2]+other.c[2]
               );
    }
    /**
    * Adds another matrix to this matrix.
    *
    * \param  other  The other matrix that is added to this one
    * \return        A reference this object after the calculation.
    */
    Matrix3x3<V>& operator+=(const Matrix3x3<V>& other)
    {
        (*this).c[0]+=other.c[0];
        (*this).c[1]+=other.c[1];
        (*this).c[2]+=other.c[2];
        return *this;
    }

    /**
     * Compute difference of this matrix and another one
     *
     * \param  other  The matrix which is substracted from this one
     * \return         A new matrix containing the result
     *                 of the calculation.
    */
    Matrix3x3<V> operator-(const Matrix3x3<V>& other) const
    {
        return Matrix3x3<V>(
                   (*this).c[0]-other.c[0],
                   (*this).c[1]-other.c[1],
                   (*this).c[2]-other.c[2]
               );
    }
    /**
    * Substracts another matrix from this one
    *
    * \param  other  The other matrix that is substracted from this one
    * \return        A reference this object after the calculation.
    */
    Matrix3x3<V>& operator-=(const Matrix3x3<V>& other)
    {
        (*this).c[0]-=other.c[0];
        (*this).c[1]-=other.c[1];
        (*this).c[2]-=other.c[2];
        return *this;
    }

    /**
     * Multiplication of this matrix by vector.
     *
     * \param  vector  The vector this one is multiplied by
     * \return         A new vector containing the result
     *                 of the calculation.
    */
    Vector3<V> operator*(const Vector3<V>& vector) const
    {
        return (c[0]*vector.x + c[1]*vector.y + c[2]*vector.z);
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A new matrix containing the result
     *                of the calculation.
    */
    Matrix3x3<V> operator*(const Matrix3x3<V>& other) const
    {
        return Matrix3x3<V>(
                   (*this)*other.c[0],
                   (*this)*other.c[1],
                   (*this)*other.c[2]
               );
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A reference this object after the calculation.
    */
    Matrix3x3<V>& operator*=(const Matrix3x3<V>& other)
    {
        return *this = *this * other;
    }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A reference to this object after the calculation.
    */
    Matrix3x3<V>& operator*=(const V& factor)
    {
        c[0] *= factor;
        c[1] *= factor;
        c[2] *= factor;
        return *this;
    }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A reference to this object after the calculation.
     */
    Matrix3x3<V>& operator/=(const V& factor)
    {
        *this *= 1 / factor;
        return *this;
    }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator*(const V& factor) const
    {
        return Matrix3x3<V>(*this) *= factor;
    }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator/(const V& factor) const
    {
        return Matrix3x3<V>(*this) /= factor;
    }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrices are equal.
     */
    bool operator==(const Matrix3x3<V>& other) const
    {
        return (
                   c[0] == other.c[0] &&
                   c[1] == other.c[1] &&
                   c[2] == other.c[2]
               );
    }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix3x3<V>& other) const
    {
        return !(*this == other);
    }

    /**
     * Array-like member access.
     * \param  i index
     * \return reference to column
     */
    Vector3<V>& operator[](int i)
    {
        return c[i];
    }

    /**
     * Transpose the matrix
     *
     * \return  A new object containing transposed matrix
     */
    Matrix3x3<V> transpose() const
    {
        return Matrix3x3<V>(
                   Vector3<V>(c[0].x, c[1].x, c[2].x),
                   Vector3<V>(c[0].y, c[1].y, c[2].y),
                   Vector3<V>(c[0].z, c[1].z, c[2].z)
               );
    }

    /**
     * Calculation of the determinant of this matrix.
     *
     * \return The determinant.
     */
    V det() const
    {
        return
            c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) +
            c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
            c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
    }

    /**
     * Calculate determinant of 2x2 Submatrix
     * | a b |
     * | c d |
     *
     * \return  determinant.
     */
    static V det2(V a, V b, V c, V d)
    {
        return a*d - b*c;
    }

    /**
     * Calculate the adjoint of this matrix.
     *
     * \return the adjoint matrix.
     */
    Matrix3x3<V> adjoint() const
    {
        return Matrix3x3<V>(
                   Vector3<V>(
                       det2(c[1].y, c[2].y, c[1].z, c[2].z),
                       det2(c[2].x, c[1].x, c[2].z, c[1].z),
                       det2(c[1].x, c[2].x, c[1].y, c[2].y)
                   ),
                   Vector3<V>(
                       det2(c[2].y, c[0].y, c[2].z, c[0].z),
                       det2(c[0].x, c[2].x, c[0].z, c[2].z),
                       det2(c[2].x, c[0].x, c[2].y, c[0].y)
                   ),
                   Vector3<V>(
                       det2(c[0].y, c[1].y, c[0].z, c[1].z),
                       det2(c[1].x, c[0].x, c[1].z, c[0].z),
                       det2(c[0].x, c[1].x, c[0].y, c[1].y)
                   )
               );

    }

    /**
     * Calculate the inverse of this matrix.
     *
     * \return The inverse matrix
     */
    Matrix3x3<V> invert() const
    {
        return adjoint().transpose() / det();
    }
};

/**
* @class Matrix2x3
* An incomplete implementation of Matrix2x3.
*/
template <class V> class Matrix2x3
{
public:
    Vector2<V> c0; /**< The first column of the matrix. */
    Vector2<V> c1; /**< The second column of the matrix. */
    Vector2<V> c2; /**< The third column of the matrix. */

    /** Default constructor. */
    Matrix2x3<V>() {}

    /**
    * Constructor; initializes each column of the matrix.
    * @param c0 The first column of the matrix.
    * @param c1 The second column of the matrix.
    * @param c2 The third column of the matrix.
    */
    Matrix2x3<V>(const Vector2<V>& c0, const Vector2<V>& c1, const Vector2<V>& c2) :
        c0(c0), c1(c1), c2(c2) {}

    /**
    * Multiplication of this matrix by vector.
    * @param vector The vector this one is multiplied by
    * @return A new vector containing the result of the calculation.
    */
    Vector2<V> operator*(const Vector3<V>& vector) const
    {
        return c0 * vector.x + c1 * vector.y + c2 * vector.z;
    }

    /**
    * Multiplication of this matrix by a 3x3 matrix.
    * @param matrix The other matrix this one is multiplied by .
    * @return A new matrix containing the result of the calculation.
    */
    Matrix2x3<V> operator*(const Matrix3x3<V>& matrix) const
    {
        return Matrix2x3<V>(*this * matrix.c[0], *this * matrix.c[1], *this * matrix.c[2]);
    }

    /**
    * Multiplication of this matrix by a 3x2 matrix.
    * @param matrix The other matrix this one is multiplied by .
    * @return A new matrix containing the result of the calculation.
    */
    Matrix2x2<V> operator*(const Matrix3x2<V>& matrix) const
    {
        return Matrix2x2<V>(*this * matrix.c0, *this * matrix.c1);
    }

    /**
    * Transposes the matrix.
    * @return A new object containing transposed matrix
    */
    Matrix3x2<V> transpose()
    {
        return Matrix2x3<V>(Vector2<V>(c0.x, c1.x, c2.x), Vector2<V>(c0.y, c1.y, c2.y));
    }
};

/**
* @class Matrix3x2
* An incomplete implementation of Matrix3x2.
*/
template <class V> class Matrix3x2
{
public:
    Vector3<V> c0; /**< The first column of the matrix. */
    Vector3<V> c1; /**< The second column of the matrix. */

    /** Default constructor. */
    Matrix3x2<V>() {}

    /**
    * Constructor; initializes each column of the matrix.
    * @param c0 The first column of the matrix.
    * @param c1 The second column of the matrix.
    */
    Matrix3x2<V>(const Vector3<V>& c0, const Vector3<V>& c1) :
        c0(c0), c1(c1) {}

    /**
    * Adds another matrix to this one (component by component).
    * @param other The matrix to add.
    * @return A reference this object after the calculation.
    */
    Matrix3x2<V>& operator+=(const Matrix3x2<V>& other)
    {
        c0 += other.c0;
        c1 += other.c1;
        return *this;
    }

    /**
    * Computes the sum of two matrices
    * @param other Another matrix
    * @return The sum
    */
    Matrix3x2<V> operator+(const Matrix3x2<V>& other) const
    {
        return Matrix3x2<V>(*this) += other;
    }

    /**
    * Division of this matrix by a factor.
    * @param factor The factor this matrix is divided by
    * @return A reference to this object after the calculation.
    */
    Matrix3x2<V>& operator/=(const V& factor)
    {
        c0 /= factor;
        c1 /= factor;
        return *this;
    }

    /**
    * Multiplication of this matrix by a factor.
    * @param factor The factor this matrix is multiplied by
    * @return A reference to this object after the calculation.
    */
    Matrix3x2<V>& operator*=(const V& factor)
    {
        c0 *= factor;
        c1 *= factor;
        return *this;
    }

    /**
    * Transposes the matrix.
    * @return A new object containing transposed matrix
    */
    Matrix2x3<V> transpose()
    {
        return Matrix2x3<V>(Vector2<V>(c0.x, c1.x), Vector2<V>(c0.y, c1.y), Vector2<V>(c0.z, c1.z));
    }
};

#endif // Matrix3x3_H
