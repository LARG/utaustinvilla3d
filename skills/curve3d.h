#ifndef CURVE3D_H
#define CURVE3D_H

#include <vector>
#include "../math/vecposition.h"

class Curve3D {
public:
    Curve3D(const std::vector<VecPosition> &controlPoints);
    virtual ~Curve3D();

    virtual VecPosition getPoint(float t) const = 0;

    const std::vector<VecPosition> &getControlPoints() const {
        return _controlPoints;
    }

protected:
    const std::vector<VecPosition> _controlPoints;
};

class Bezier3D : public Curve3D {
public:
    Bezier3D(const std::vector<VecPosition> &controlPoints);
    virtual ~Bezier3D();

    virtual VecPosition getPoint(float t) const;

private:
    std::vector<long> _coeff;
};

class HermiteSpline3D : public Curve3D {
public:
    HermiteSpline3D(const std::vector<VecPosition> &controlPoints, float tangentScale);
    virtual ~HermiteSpline3D();
    virtual VecPosition getPoint(float t) const;
private:
    float scale;
    std::vector<VecPosition> tangents;
};

class UniformBSpline3D : public Curve3D {
public:
    UniformBSpline3D(const std::vector<VecPosition> &controlPoints);
    virtual ~UniformBSpline3D();

    virtual VecPosition getPoint(float t) const;

protected:
    const size_t _n;              ///< degree
    const size_t _m;              ///< m knots
    std::vector<float> _T;      ///< knot vector

    float b(int, int, float) const;

};

#endif
