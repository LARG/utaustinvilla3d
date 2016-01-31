/**
 * @file Pose2D.h
 * Contains class Pose2D
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#ifndef __Pose2D_h__
#define __Pose2D_h__

#include <math/Vector2.h>
#include <math/Range.h>
#include <math/Common.h>


/** representation for 2D Transformation and Position (Location + Orientation)*/
class Pose2D
{
public:

    /** Rotation as an angle*/
    float rotation;

    /** translation as an vector2*/
    Vector2<float> translation;

    /** noargs-constructor*/
    Pose2D():rotation(0),translation(0,0) {}

    /** constructor from rotation and translation
     * \param rotation rotation (float)
     * \param translation translation (Vector2)
     */
    Pose2D(const float rotation, const Vector2<float>& translation):rotation(rotation),translation(translation) {}

    /** constructor from rotation and translation
     * \param rot rotation (float)
     * \param x translation.x (float)
     * \param y translation.y (float)
     */
    Pose2D(const float rot, const float x, const float y):rotation(rot),translation(x,y)
    {}

    /** constructor from rotation
     * \param rotation rotation (float)
     */
    Pose2D(const float rotation):rotation(rotation),translation(0,0) {}

    /** constructor from translation
     * \param translation translation (Vector2)
     */
    Pose2D(const Vector2<float>& translation):rotation(0),translation(translation) {}

    /** constructor from translation
     * \param translation translation (Vector2)
     */
    Pose2D(const Vector2<int>& translation):rotation(0),translation((float) translation.x,(float) translation.y) {}

    /** constructor from two translation values
     * \param x translation x component
     * \param y translation y component
     */
    Pose2D(const float x, const float y):rotation(0),translation(x,y) {}

    /** get the Angle
    * @return Angle the Angle which defines the rotation
    */
    inline float getAngle() const {
        return rotation;
    }


    /** set rotation from Angle
    * @return the new Pose2D
    */
    inline Pose2D fromAngle(const float a) {
        rotation=a;
        return *this;
    }

    /** get the cos of the angle
    * @return the cos of the angle
    */
    inline float getCos() const {
        return cosf(rotation);
    }

    /** get the sin of the angle
     * @return the sin of the angle
     */
    inline float getSin() const {
        return sinf(rotation);
    }

    /** Assignment operator
    *\param other The other Pose2D that is assigned to this one
    *\return A reference to this object after the assignment.
    */
    Pose2D& operator=(const Pose2D& other)
    {
        rotation=other.rotation;
        translation=other.translation;
        return *this;
    }

    /** Copy constructor
    *\param other The other vector that is copied to this one
    */
    Pose2D(const Pose2D& other) {
        *this = other;
    }

    /** Multiplication of a Vector2 with this Pose2D
    *\param point The Vector2 that will be multiplicated with this Pose2D
    *\return The resulting Vector2
    */

    Vector2<float> operator*(const Vector2<float>& point) const
    {
        float s=sinf(rotation);
        float c=cosf(rotation);
        return (Vector2<float>(point.x*c-point.y*s , point.x*s+point.y*c) + translation);
    }

    /** Comparison of another pose with this one.
    *\param other The other pose that will be compared to this one
    *\return Whether the two poses are equal.
    */
    bool operator==(const Pose2D& other) const
    {
        return ((translation==other.translation)&&(rotation==other.rotation));
    }

    /** Comparison of another pose with this one.
    *\param other The other pose that will be compared to this one
    *\return Whether the two poses are unequal.
    */
    bool operator!=(const Pose2D& other) const
    {
        return !(*this == other);
    }

    /**Concatenation of this pose with another pose.
    *\param other The other pose that will be concatenated to this one.
    *\return A reference to this pose after concatenation.
    */
    Pose2D& operator+=(const Pose2D& other)
    {
        translation = *this * other.translation;
        rotation += other.rotation;
        rotation = normalize(rotation);
        return *this;
    }

    /**A concatenation of this pose and another pose.
    *\param other The other pose that will be concatenated to this one.
    *\return The resulting pose.
    */
    Pose2D operator+(const Pose2D& other) const
    {
        return Pose2D(*this) += other;
    }

    /**Subtracts a difference pose from this one to get the source pose. So if A+B=C is the addition/concatenation, this calculates C-B=A.
    *\param diff The difference Pose2D that shall be subtracted.
    *\return The resulting source pose. Adding diff to it again would give the this pose back.
    */
    Pose2D minusDiff(const Pose2D& diff) const
    {
        float rot=rotation-diff.rotation;
        float s=sinf(rot);
        float c=cosf(rot);
        return Pose2D(
                   rot,
                   translation.x - c*diff.translation.x + s*diff.translation.y,
                   translation.y - c*diff.translation.y - s*diff.translation.x);
    }

    /**Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates C-A=B.
    *\param other The other pose that will be used as origin for the new pose.
    *\return A reference to this pose after calculating the difference.
    */
    Pose2D& operator-=(const Pose2D& other)
    {
        translation -= other.translation;
        Pose2D p(-other.rotation);
        return *this = p + *this;
    }

    /**Difference of this pose relative to another pose.
    *\param other The other pose that will be used as origin for the new pose.
    *\return The resulting pose.
    */
    Pose2D operator-(const Pose2D& other) const
    {
        return Pose2D(*this) -= other;
    }

    /**Concatenation of this pose with another pose
    *\param other The other pose that will be concatenated to this one.
    *\return A reference to this pose after concatenation
    */
    Pose2D& conc(const Pose2D& other)
    {
        return *this += other;
    }

    /**Translate this pose by a translation vector
    *\param trans Vector to translate with
    *\return A reference to this pose after translation
    */
    Pose2D& translate(const Vector2<float>& trans)
    {
        translation = *this * trans;
        return *this;
    }

    /**Translate this pose by a translation vector
    *\param x x component of vector to translate with
    *\param y y component of vector to translate with
    *\return A reference to this pose after translation
    */
    Pose2D& translate(const float x, const float y)
    {
        translation = *this * Vector2<float>(x,y);
        return *this;
    }


    /**Rotate this pose by a rotation
    *\param angle Angle to rotate.
    *\return A reference to this pose after rotation
    */
    Pose2D& rotate(const float angle)
    {
        rotation += angle;
        return *this;
    }


    /** Calculates the inverse transformation from the current pose
    * @return The inverse transformation pose.
    */
    Pose2D invert() const
    {
        const float& invRotation = -rotation;
        return Pose2D(invRotation, (Vector2<float>() - translation).rotate(invRotation));
    }

    /**
    * The function creates a random pose.
    * @param x The range for x-values of the pose.
    * @param y The range for y-values of the pose.
    * @param angle The range for the rotation of the pose.
    */
    static Pose2D random(const Range<float>& x,
                         const Range<float>& y,
                         const Range<float>& angle)
    {   // angle should even work in wrap around case!
        return Pose2D(float(::randomFloat() * (angle.max - angle.min) + angle.min),
                      Vector2<float>(float(::randomFloat() * (x.max - x.min) + x.min),
                                     float(::randomFloat() * (y.max - y.min) + y.min)));
    }
};

#endif // __Pose2D_h__
