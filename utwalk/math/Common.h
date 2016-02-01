#ifndef COMMON_15V925VT
#define COMMON_15V925VT

#include <cmath>
#include <cstdlib>

//const float M_2PI = 2.0f * M_PI;
#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

/**
* Returns the sign of a value.
* \param a The value.
* \return The sign of \c a.
*/
template <class V> inline int sgn(const V& a) {
    return a < 0 ? -1 : ((a == 0) ? 0 : 1);
}

/**
* Calculates the square of a value.
* \param a The value.
* \return The square of \c a.
*/
template <class V> inline V sqr(const V& a) {
    return a * a;
}

/**
* Calculates the sec of a value.
* \param a The value.
* \return The sec of \c a.
*/
template <class V> inline V sec(const V& a) {
    return 1/cosf(a);
}

/**
* Calculates the cosec of a value.
* \param a The value.
* \return The cosec of \c a.
*/
template <class V> inline V cosec(const V& a) {
    return 1/sinf(a);
}

/**
* reduce angle to [-pi..+pi]
* \param data angle coded in rad
* \return normalized angle coded in rad
*/
template <class V> inline V normalize(const V& data)
{
    if(data < V(M_PI) && data >= -V(M_PI)) return data;
    V ndata = data - ((int )(data / V(M_2PI))) * V(M_2PI);
    if(ndata >= V(M_PI))
    {
        ndata -= V(M_2PI);
    }
    else if(ndata < -V(M_PI))
    {
        ndata += V(M_2PI);
    }
    return ndata;
}

/** constant, cast before execution*/
const float RAND_MAX_FLOAT = static_cast<float>(RAND_MAX);

/**
* The function returns a random number in the range of [0..1].
* @return The random number.
*/
inline float randomFloat() {
    return float(rand()) / RAND_MAX_FLOAT;
}

/**
* The function returns a random integer number in the range of [0..n-1].
* @param n the number of possible return values (0 ... n-1)
* @return The random number.
*/
inline int random(int n) {
    return (int)(randomFloat()*n*0.999999);
}

/**
* The function returns a random integer number in the range of [0..n].
* @param n the number of possible return values (0 ... n)
* @return The random number.
*/
inline int randomFast(int n)
{
    return static_cast<int>((rand()*n) / RAND_MAX_FLOAT);
}

#endif /* end of include guard: COMMON_15V925VT */
