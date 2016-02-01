/**
* @file Vector.h
* Contains template class Vector of type V and size n
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
* @author Colin Graf
*/

#ifndef Vector_H
#define Vector_H

#include <cassert>

template <int n, class V> struct VectorData
{
    V v[n];
};

template <class V> struct VectorData<2, V>
{
    V x;
    V y;
};

template <class V> struct VectorData<3, V>
{
    V x;
    V y;
    V z;
};

/** This class represents a n-dimensional vector */
template <int n = 2, class V = float> class Vector : public VectorData<n, V>
{
public:
    /** Default constructor. */
    Vector<n, V>()
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] = V();
    }

    /** constructor for a 2-dimensional vector. */
    Vector<n, V>(V x, V y)
    {
        assert(n == 2);
        (*this)[0] = x;
        (*this)[1] = y;
    }

    /** constructor for a 3-dimensional vector. */
    Vector<n, V>(V x, V y, V z)
    {
        assert(n == 3);
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
    }

    /** constructor for a 4-dimensional vector. */
    Vector<n, V>(V x, V y, V z, V w)
    {
        assert(n == 4);
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
        (*this)[3] = w;
    }
    /** constructor for a n-dimensional vector. */
    /*
    Vector<n, V>(V x, V y, V z, V w, ...)
    {
      (*this)[0] = x;
      (*this)[1] = y;
      (*this)[2] = z;
      (*this)[3] = w;
      va_list vl;
      va_start(vl, w);
      for(int i = 4; i < n; ++i)
        (*this)[i] = va_arg(vl, V);
      va_end(vl);
    }
    */

    /**
    * Copy constructor
    * @param other The other vector that is copied to this one
    */
    Vector<n, V>(const Vector<n, V>& other)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] = other[i];
    }

    /**
    * Assignment operator
    * @param other The other vector that is assigned to this one
    * @return A reference to this object after the assignment.
    */
    Vector<n, V>& operator=(const Vector<n, V>& other)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] = other[i];
        return *this;
    }

    /**
    * Addition of another vector to this one.
    * @param other The other vector that will be added to this one
    * @return A reference to this object after the calculation.
    */
    Vector<n, V>& operator+=(const Vector<n, V>& other)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] += other[i];
        return *this;
    }

    /**
    * Substraction of this vector from another one.
    * @param other The other vector this one will be substracted from
    * @return A reference to this object after the calculation.
    */
    Vector<n, V>& operator-=(const Vector<n, V>& other)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] -= other[i];
        return *this;
    }

    /**
    * Multiplication of this vector by a factor.
    * @param factor The factor this vector is multiplied by
    * @return A reference to this object after the calculation.
    */
    Vector<n, V>& operator*=(const V& factor)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] *= factor;
        return *this;
    }

    /**
    * Division of this vector by a factor.
    * @param factor The factor this vector is divided by
    * @return A reference to this object after the calculation.
    */
    Vector<n, V>& operator/=(const V& factor)
    {
        for(int i = 0; i < n; ++i)
            (*this)[i] /= factor;
        return *this;
    }

    /**
    * Addition of another vector to this one.
    * @param other The other vector that will be added to this one
    * @return A new object that contains the result of the calculation.
    */
    Vector<n, V> operator+(const Vector<n, V>& other) const
    {
        return Vector<n, V>(*this) += other;
    }

    /**
    * Subtraction of another vector to this one.
    * @param other The other vector that will be added to this one
    * @return A new object that contains the result of the calculation.
    */
    Vector<n, V> operator-(const Vector<n, V>& other) const
    {
        return Vector<n, V>(*this) -= other;
    }

    /**
    * Inner product of this vector and another one.
    * @param other The other vector this one will be multiplied by
    * @return The inner product.
    */
    V operator*(const Vector<n, V>& other) const
    {
        V result = (*this)[0] * other[0];
        for(int i = 1; i < n; ++i)
            result += (*this)[i] * other[i];
        return result;
    }

    /**
    * Multiplication of this vector by a factor.
    * @param factor The factor this vector is multiplied by
    * @return A new object that contains the result of the calculation.
    */
    Vector<n, V> operator*(const V& factor) const
    {
        return Vector<n, V>(*this) *= factor;
    }

    /**
    * Division of this vector by a factor.
    * @param factor The factor this vector is divided by
    * @return A new object that contains the result of the calculation.
    */
    Vector<n, V> operator/(const V& factor) const
    {
        return Vector<n, V>(*this) /= factor;
    }

    /**
    * Negation of this vector.
    * @return A new object that contains the result of the calculation.
    */
    Vector<n, V> operator-() const
    {
        return Vector<n, V>() -= *this;
    }

    /**
    * Comparison of another vector with this one.
    * @param other The other vector that will be compared to this one
    * @return Whether the two vectors are equal.
    */
    bool operator==(const Vector<n, V>& other) const
    {
        for(int i = 0; i < n; ++i)
            if((*this)[i] != other[i])
                return false;
        return true;
    }

    /**
    * Comparison of another vector with this one.
    * @param other The other vector that will be compared to this one
    * @return Whether the two vectors are unequal.
    */
    bool operator!=(const Vector<n, V>& other) const
    {
        for(int i = 0; i < n; ++i)
            if((*this)[i] != other[i])
                return true;
        return false;
    }

    /**
    * array-like member access.
    * @param i index of coordinate
    * @return reference to the coordinate
    */
    inline V& operator[](int i)
    {
        return ((V*)this)[i];
    }

    /**
    * array-like member access.
    * @param i index of coordinate
    * @return reference to the coordinate
    */
    inline const V& operator[](int i) const
    {
        return ((const V*)this)[i];
    }

    /**
    * Calculation of the square length of this vector.
    * @return length*length.
    */
    V sqr() const
    {
        V result = (*this)[0] * (*this)[0];
        for(int i = 1; i < n; ++i)
            result += (*this)[i] * (*this)[i];
        return result;
    }

    /**
    * Calculation of the length of this vector.
    * @return length*length.
    */
    V abs() const
    {
        return sqrt(sqr());
    }

};



typedef Vector<2, float> Vector2f;
typedef Vector<3, float> Vector3f;
typedef Vector<4, float> Vector4f;

typedef Vector<2, int> Vector2i;
typedef Vector<3, int> Vector3i;
typedef Vector<4, int> Vector4i;

#endif // Vector_H

