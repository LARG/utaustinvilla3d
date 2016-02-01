/**
* @file Matrix.cpp
* Implements RotationMatrix
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
* Edited by Sam Barrett
*/

#include "RotationMatrix.h"
#include "Range.h"
#include "MVTools.h"
//#include "Common.h"

RotationMatrix::RotationMatrix(
    const Vector3<float>& c0,
    const Vector3<float>& c1,
    const Vector3<float>& c2) : Matrix3x3<float>(c0,c1,c2) {}

RotationMatrix::RotationMatrix(float yaw, float pitch, float roll)
{
    const float cy=cosf(yaw);
    const float sy=sinf(yaw);
    const float cp=cosf(pitch);
    const float sp=sinf(pitch);
    const float cr=cosf(roll);
    const float sr=sinf(roll);

    c[0].x=cr*cp ;
    c[0].y=-sr*cy+cr*sp*sy ;
    c[0].z=sr*sy+cr*sp*cy ;
    c[1].x=sr*cp ;
    c[1].y=cr*cy+sr*sp*sy ;
    c[1].z=-cr*sy+sr*sp*cy ;
    c[2].x=-sp ;
    c[2].y=cp*sy ;
    c[2].z=cp*cy ;
}

RotationMatrix::RotationMatrix(const Vector3<float>& a, float angle)
{
    Vector3<float> axis(a);
    //rotation is only possible with unit vectors
    axis.normalize();
    const float &x = axis.x, &y = axis.y, &z = axis.z;
    //compute sine and cosine of angle because it is needed quite often for complete matrix
    const float si = sinf(angle), co = cosf(angle);
    //compute all components needed more than once for complete matrix
    const float v = 1 - co;
    const float xyv = x * y * v;
    const float xzv = x * z * v;
    const float yzv = y * z * v;
    const float xs = x * si;
    const float ys = y * si;
    const float zs = z * si;
    //compute matrix
    c[0].x = x * x * v + co;
    c[1].x = xyv - zs;
    c[2].x = xzv + ys;
    c[0].y = xyv + zs;
    c[1].y = y * y * v + co;
    c[2].y = yzv - xs;
    c[0].z = xzv - ys;
    c[1].z = yzv + xs;
    c[2].z = z * z * v + co;
}

RotationMatrix::RotationMatrix(const Vector3<float>& a)
{
    const float angle = a.abs();
    const Vector3<float> axis(a / angle); // normalize
    //rotation is only possible with unit vectors
    const float &x = axis.x, &y = axis.y, &z = axis.z;
    //compute sine and cosine of angle because it is needed quite often for complete matrix
    const float si = sinf(angle), co = cosf(angle);
    //compute all components needed more than once for complete matrix
    const float v = 1 - co;
    const float xyv = x * y * v;
    const float xzv = x * z * v;
    const float yzv = y * z * v;
    const float xs = x * si;
    const float ys = y * si;
    const float zs = z * si;
    //compute matrix
    c[0].x = x * x * v + co;
    c[1].x = xyv - zs;
    c[2].x = xzv + ys;
    c[0].y = xyv + zs;
    c[1].y = y * y * v + co;
    c[2].y = yzv - xs;
    c[0].z = xzv - ys;
    c[1].z = yzv + xs;
    c[2].z = z * z * v + co;
}

RotationMatrix& RotationMatrix::rotateX(const float angle)
{
    const float c = cosf(angle),
                s = sinf(angle);
    *this = RotationMatrix(this->c[0], this->c[1] * c + this->c[2] * s, this->c[2] * c - this->c[1] * s);
    return *this;
}

RotationMatrix& RotationMatrix::rotateY(const float angle)
{
    const float c = cosf(angle),
                s = sinf(angle);
    *this = RotationMatrix(this->c[0] * c - this->c[2] * s, this->c[1], this->c[2] * c + this->c[0] * s);
    return *this;
}

RotationMatrix& RotationMatrix::rotateZ(const float angle)
{
    const float c = cosf(angle),
                s = sinf(angle);
    *this = RotationMatrix(this->c[0] * c + this->c[1] * s, this->c[1] * c - this->c[0] * s, this->c[2]);
    return *this;
}

float RotationMatrix::getXAngle() const
{
    const float h = sqrtf(c[2].y * c[2].y + c[2].z * c[2].z);
    return h ? acosf(c[2].z / h) * (c[2].y > 0 ? -1 : 1) : 0;
}

float RotationMatrix::getYAngle() const
{
    const float h = sqrtf(c[0].x * c[0].x + c[0].z * c[0].z);
    return h ? acosf(c[0].x / h) * (c[0].z > 0 ? -1 : 1) : 0;
}

float RotationMatrix::getZAngle() const
{
    const float h = sqrtf(c[0].x * c[0].x + c[0].y * c[0].y);
    return h ? acosf(c[0].x / h) * (c[0].y < 0 ? -1 : 1) : 0;
}

void RotationMatrix::getAngleAxis(Vector3<float>& axis, float& angle) const
{
    const float cosAngle = ((c[0].x + c[1].y + c[2].z) - 1.0f) / 2.0f;
    static const Range<float> clipping(-1.0f, 1.0f);
    angle = acosf(clipping.limit(cosAngle));
    if(MVTools::isNearZero(angle))
    {
        axis = Vector3<float>(1, 0, 0);
    }
    else
    {
        axis.x = c[1].z - c[2].y;
        axis.y = c[2].x - c[0].z;
        axis.z = c[0].y - c[1].x;
        axis.normalize();
    }
}

Vector3<float> RotationMatrix::getAngleAxis() const
{
    float co = (this->c[0].x + this->c[1].y + this->c[2].z - 1.f) * 0.5f;
    if(co > 1.f)
        co = 1.f;
    else if(co < -1.f)
        co = 1.f;
    const float angle = acos(co);
    if(angle == 0.f)
        return Vector3<float>();
    Vector3<float> result(
        this->c[1].z - this->c[2].y,
        this->c[2].x - this->c[0].z,
        this->c[0].y - this->c[1].x);
    result *= angle / (2.f * sin(angle));
    return result;
}
