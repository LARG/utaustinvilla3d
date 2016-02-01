#include "curve3d.h"
#include <cmath>
#include <cassert>
#include <stdexcept>
using namespace std;

/**
 * Computes binomial coefficients using n-choose-k.
 *
 * \param n The number of items to choose from.
 * \param k The number of items to choose.
 * \return The number of ways you can choose k items out of a set of n items.
 */
int nChooseK(const int n, const int k) {
    if ((n < k) // can't choose more items than there are to choose from
            || (n < 0) // can't have negative number of items to choose from
            || (k < 0)) { // can't choose negative number of items
        return 0;
    }

    if ((k == n) || (k == 0)) {
        // recursion base case
        // ways to pick all or none of the items
        return 1;
    } else {
        int ret = nChooseK(n - 1, k - 1) + nChooseK(n - 1, k);
        if (ret < 0) {
            throw std::overflow_error("nChooseK overflowed");
        }
        return ret;
    }
}

/**
 * Constructor.
 *
 * \param controlPoints The control points defining the curve.
 */
Curve3D::Curve3D(const std::vector<VecPosition> &controlPoints) :
    _controlPoints(controlPoints) {
}

/**
 * Destructor.
 */
Curve3D::~Curve3D() {
}

/**
 * Constructor.
 *
 * \param controlPoints The control points defining the curve.
 */
Bezier3D::Bezier3D(const std::vector<VecPosition> &controlPoints) :
    Curve3D(controlPoints), _coeff(controlPoints.size()) {
    int n = controlPoints.size() - 1;
    for (int i = 0; i <= n; ++i) {
        _coeff[i] = nChooseK(n, i);
    }
}

/**
 * Destructor.
 */
Bezier3D::~Bezier3D() {
}

/**
 * Gets a point along the curve.
 *
 * \param t  Specifies where along the curve to sample.
 *           t=0.0 is the start of the curve
 *           t=0.5 is the middle of the curve
 *           t=1.0 is the end of the curve
 *           etc.
 *           Value of t < 0.0 or > 1.0 are erroneous.
 */
VecPosition Bezier3D::getPoint(float t) const {
    if (t < 0.0) {
        throw std::domain_error("t must be >= 0.0");
    } else if (t > 1.0) {
        throw std::domain_error("t must be <= 1.0");
    }

    VecPosition sum;
    int n = _controlPoints.size() - 1;

    for (int i = 0; i <= n; ++i) {
        const VecPosition &p = _controlPoints[i];
        float mult = _coeff[i] * (float) (pow(1 - t, n - i) * pow(t, i));
        sum = sum + p * mult;
    }

    return sum;
}

/**
 * Constructor.
 */
UniformBSpline3D::UniformBSpline3D(const std::vector<VecPosition> &controlPoints) :
    Curve3D(controlPoints),
    _n(1),        // Linear
    _m(controlPoints.size() + _n + 1),
    _T(_m) {
    //LOG(_n);
    //LOG(_m);
    float dt = 1.0f / (_m-1);
    //LOG(dt);
    for(size_t i=0; i<_m; ++i) {
        _T[i] = dt * i;     // uniform
    }
    assert(0.0f == _T[0]);
    assert(1.0f == _T[_m-1]);
    //LOG(_T);
}

/**
 * Destroctor.
 */
UniformBSpline3D::~UniformBSpline3D() {
}

HermiteSpline3D::HermiteSpline3D(const std::vector<VecPosition> &controlPoints, float tangentScale)
    : Curve3D(controlPoints), scale(tangentScale) {
    // set up tangent points
    tangents.resize(_controlPoints.size());
    size_t len = tangents.size();
    tangents[0] = VecPosition();//(controlPoints[1] - controlPoints[0]) * scale;
    for(size_t i = 1; i < len - 1; ++i) {
        tangents[i] = (controlPoints[i+1] - controlPoints[i-1]) * scale;
    }
    tangents[len - 1] = VecPosition();//(controlPoints[len - 1] - controlPoints[len - 2]) * scale;
}

VecPosition HermiteSpline3D::getPoint(float u) const {
    assert( u >= 0 && u <= 1 );

    // get associated index for curve
    int idx = floor( (_controlPoints.size() - 1) * u);
    // get u between these corresponding points only
    u = u * (_controlPoints.size()-1) - idx;

    // compute cubic hermite parameters
    float h1 = 2 * pow(u, 3) - 3 * pow(u, 2) + 1;
    float h2 = -2 * pow(u, 3) + 3 * pow(u, 2);
    float h3 = pow(u, 3) - 2 * pow(u, 2) + u;
    float h4 = pow(u, 3) -  pow(u, 2);

    VecPosition p1, p2;
    p1 = _controlPoints[ idx ];
    p2 = _controlPoints[ idx + 1 ];

    return p1 * h1 + p2 * h2 + tangents[idx] * h3 + tangents[idx + 1] * h4;
}

/**
 * Destructor.
 */
HermiteSpline3D::~HermiteSpline3D() {}

/**
 * Gets a point along the curve.
 *
 * \param t  Specifies where along the curve to sample.
 *           t=0.0 is the start of the curve
 *           t=0.5 is the middle of the curve
 *           t=1.0 is the end of the curve
 *           etc.
 *           Value of t < 0.0 or > 1.0 are erroneous.
 */
VecPosition UniformBSpline3D::getPoint(float t) const {
    if (t < 0.0) {
        throw std::domain_error("t must be >= 0.0");
    } else if (t > 1.0) {
        throw std::domain_error("t must be <= 1.0");
    }

    assert(_controlPoints.size() == _m -_n - 1);

    VecPosition sum = VecPosition(0, 0, 0);

    // t really needs to be in the range [_T[_n], _T[_m-_n-1]]
    float trange = _T[_m-_n-1] - _T[_n];
    float newT = t*trange + _T[_n];

    for (size_t i = 0; i < _controlPoints.size(); ++i) {
        sum += _controlPoints[i] * b(i, _n, newT);
//    LOG(sum);
    }

    return sum;
}

/**
 * Compute the basis functions N_i,j(t).
 *
 * \param j
 * \param n 0 .. m-2
 * \param t [0, 1]
 * \return the value of the basis function at t
 */
float UniformBSpline3D::b(const int j, const int n, const float t) const {
    // bounds checking
//  if (j < 0) {
//    throw std::domain_error("BSpline3D::b(): j must be >= 0");
//  }
//  if (j > _m-_n - 2) {
//    throw std::domain_error("BSpline3D::b(): j must be <= m-n-2");
//  }
//  if (n < 0) {
//    throw std::domain_error("BSpline3D::b(): n must be >= 0");
//  }
//  if (n > _m-2) {
//    throw std::domain_error("BSpline3D::b(): n must be <= m-2");
//  }
    if (t < 0.0) {
        throw std::domain_error("BSpline3D::b(): t must be >= 0.0");
    }
    if (t > 1.0) {
        throw std::domain_error("BSpline3D::b(): t must be <= 1.0");
    }

    try {
        // base case
        if (0 == n) {
            if ((_T.at(j) <= t) && (t < _T.at(j + 1))) {
                return 1.0;
            } else {
                return 0.0;
            }

        }

        return (t - _T.at(j))         / (_T.at(j + n)     - _T.at(j))     * b(j    , n - 1, t)
               + (_T.at(j + n + 1) - t)  / (_T.at(j + n + 1) - _T.at(j + 1)) * b(j + 1, n - 1, t);
    } catch (std::out_of_range &e) {
        throw std::domain_error(e.what());
    }
}
