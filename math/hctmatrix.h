#ifndef HCTMATRIX_H
#define HCTMATRIX_H

#include "../headers/headers.h"
#include "vecposition.h"

#include <iostream>
using namespace std;


class HCTMatrix {

private:

    double cell[4][4];

    inline void copy(const HCTMatrix& m) {

        setCell(0, 0, m.getCell(0, 0));
        setCell(0, 1, m.getCell(0, 1));
        setCell(0, 2, m.getCell(0, 2));
        setCell(0, 3, m.getCell(0, 3));

        setCell(1, 0, m.getCell(1, 0));
        setCell(1, 1, m.getCell(1, 1));
        setCell(1, 2, m.getCell(1, 2));
        setCell(1, 3, m.getCell(1, 3));

        setCell(2, 0, m.getCell(2, 0));
        setCell(2, 1, m.getCell(2, 1));
        setCell(2, 2, m.getCell(2, 2));
        setCell(2, 3, m.getCell(2, 3));

        setCell(3, 0, m.getCell(3, 0));
        setCell(3, 1, m.getCell(3, 1));
        setCell(3, 2, m.getCell(3, 2));
        setCell(3, 3, m.getCell(3, 3));
    }

public:

    HCTMatrix();
    HCTMatrix(const HCTMatrix& m);
    HCTMatrix(const int &mode);
    HCTMatrix(const int &mode, const double &rotateAngle);
    HCTMatrix(const int &mode, const VecPosition &translateVector);
    HCTMatrix(const int &mode, const VecPosition &axis, const double &rotateAngle);

    inline void setCell(const int &i, const int &j, const double &v) {
        cell[i][j] = v;
    }
    inline double getCell(const int &i, const int &j) const {
        return cell[i][j];
    }

    inline void createIdentity() {

        setCell(0, 0, 1.0);
        setCell(0, 1, 0);
        setCell(0, 2, 0);
        setCell(0, 3, 0); //First row
        setCell(1, 0, 0);
        setCell(1, 1, 1.0);
        setCell(1, 2, 0);
        setCell(1, 3, 0); //Second row
        setCell(2, 0, 0);
        setCell(2, 1, 0);
        setCell(2, 2, 1.0);
        setCell(2, 3, 0); //Third row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0); //Fourth row
    }

    inline void createRotateX(const double &theta) {

        double cos = cosDeg(-theta);
        double sin = sinDeg(-theta);

        setCell(0, 0, 1.0);
        setCell(0, 1, 0);
        setCell(0, 2, 0);
        setCell(0, 3, 0); //First row
        setCell(1, 0, 0);
        setCell(1, 1, cos);
        setCell(1, 2, -sin);
        setCell(1, 3, 0); //Second row
        setCell(2, 0, 0);
        setCell(2, 1, sin);
        setCell(2, 2, cos);
        setCell(2, 3, 0); //Third row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0); //Fourth row
    }

    inline void createRotateY(const double &theta) {

        double cos = cosDeg(-theta);
        double sin = sinDeg(-theta);

        setCell(0, 0, cos);
        setCell(0, 1, 0);
        setCell(0, 2, sin);
        setCell(0, 3, 0); //First row
        setCell(1, 0, 0);
        setCell(1, 1, 1.0);
        setCell(1, 2, 0);
        setCell(1, 3, 0); //Second row
        setCell(2, 0, -sin);
        setCell(2, 1, 0);
        setCell(2, 2, cos);
        setCell(2, 3, 0); //Third row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0); //Fourth row
    }

    inline void createRotateZ(const double &theta) {

        double cos = cosDeg(-theta);
        double sin = sinDeg(-theta);

        setCell(0, 0, cos);
        setCell(0, 1, -sin);
        setCell(0, 2, 0);
        setCell(0, 3, 0); //First row
        setCell(1, 0, sin);
        setCell(1, 1, cos);
        setCell(1, 2, 0);
        setCell(1, 3, 0); //Second row
        setCell(2, 0, 0);
        setCell(2, 1, 0);
        setCell(2, 2, 1.0);
        setCell(2, 3, 0); //Third row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0); //Fourth row
    }

    inline void createRotateGeneral(const VecPosition &axis, const double &theta) {

        double X = axis.getX();
        double Y = axis.getY();
        double Z = axis.getZ();

        double c = cosDeg(theta);
        double t = 1 - c;
        double s = sinDeg(theta);

        //First row
        setCell(0, 0, (t * X * X) + c);
        setCell(0, 1, (t * X * Y) - (s * Z));
        setCell(0, 2, (t * X * Z) + (s * Y));
        setCell(0, 3, 0);

        //Second row
        setCell(1, 0, (t * X * Y) + (s * Z));
        setCell(1, 1, (t * Y * Y) + c);
        setCell(1, 2, (t * Y * Z) - (s * X) );
        setCell(1, 3, 0);

        //Third row
        setCell(2, 0, (t * X * Z) - (s * Y));
        setCell(2, 1, (t * Y * Z) + (s * X));
        setCell(2, 2, (t * Z * Z) + c);
        setCell(2, 3, 0);

        //Fourth row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0);

    }


    inline void createTranslate(const VecPosition& v) {

        setCell(0, 0, 1.0);
        setCell(0, 1, 0);
        setCell(0, 2, 0);
        setCell(0, 3, v.getX()); //First row
        setCell(1, 0, 0);
        setCell(1, 1, 1.0);
        setCell(1, 2, 0);
        setCell(1, 3, v.getY()); //Second row
        setCell(2, 0, 0);
        setCell(2, 1, 0);
        setCell(2, 2, 1.0);
        setCell(2, 3, v.getZ()); //Third row
        setCell(3, 0, 0);
        setCell(3, 1, 0);
        setCell(3, 2, 0);
        setCell(3, 3, 1.0); //Fourth row
    }

    inline void multiply(const HCTMatrix& m) {

        double s[4][4];

        s[0][0] = getCell(0, 0) * m.getCell(0, 0);
        s[0][0] += getCell(0, 1) * m.getCell(1, 0);
        s[0][0] += getCell(0, 2) * m.getCell(2, 0);
        s[0][0] += getCell(0, 3) * m.getCell(3, 0);

        s[0][1] = getCell(0, 0) * m.getCell(0, 1);
        s[0][1] += getCell(0, 1) * m.getCell(1, 1);
        s[0][1] += getCell(0, 2) * m.getCell(2, 1);
        s[0][1] += getCell(0, 3) * m.getCell(3, 1);

        s[0][2] = getCell(0, 0) * m.getCell(0, 2);
        s[0][2] += getCell(0, 1) * m.getCell(1, 2);
        s[0][2] += getCell(0, 2) * m.getCell(2, 2);
        s[0][2] += getCell(0, 3) * m.getCell(3, 2);

        s[0][3] = getCell(0, 0) * m.getCell(0, 3);
        s[0][3] += getCell(0, 1) * m.getCell(1, 3);
        s[0][3] += getCell(0, 2) * m.getCell(2, 3);
        s[0][3] += getCell(0, 3) * m.getCell(3, 3);

        s[1][0] = getCell(1, 0) * m.getCell(0, 0);
        s[1][0] += getCell(1, 1) * m.getCell(1, 0);
        s[1][0] += getCell(1, 2) * m.getCell(2, 0);
        s[1][0] += getCell(1, 3) * m.getCell(3, 0);

        s[1][1] = getCell(1, 0) * m.getCell(0, 1);
        s[1][1] += getCell(1, 1) * m.getCell(1, 1);
        s[1][1] += getCell(1, 2) * m.getCell(2, 1);
        s[1][1] += getCell(1, 3) * m.getCell(3, 1);

        s[1][2] = getCell(1, 0) * m.getCell(0, 2);
        s[1][2] += getCell(1, 1) * m.getCell(1, 2);
        s[1][2] += getCell(1, 2) * m.getCell(2, 2);
        s[1][2] += getCell(1, 3) * m.getCell(3, 2);

        s[1][3] = getCell(1, 0) * m.getCell(0, 3);
        s[1][3] += getCell(1, 1) * m.getCell(1, 3);
        s[1][3] += getCell(1, 2) * m.getCell(2, 3);
        s[1][3] += getCell(1, 3) * m.getCell(3, 3);

        s[2][0] = getCell(2, 0) * m.getCell(0, 0);
        s[2][0] += getCell(2, 1) * m.getCell(1, 0);
        s[2][0] += getCell(2, 2) * m.getCell(2, 0);
        s[2][0] += getCell(2, 3) * m.getCell(3, 0);

        s[2][1] = getCell(2, 0) * m.getCell(0, 1);
        s[2][1] += getCell(2, 1) * m.getCell(1, 1);
        s[2][1] += getCell(2, 2) * m.getCell(2, 1);
        s[2][1] += getCell(2, 3) * m.getCell(3, 1);

        s[2][2] = getCell(2, 0) * m.getCell(0, 2);
        s[2][2] += getCell(2, 1) * m.getCell(1, 2);
        s[2][2] += getCell(2, 2) * m.getCell(2, 2);
        s[2][2] += getCell(2, 3) * m.getCell(3, 2);

        s[2][3] = getCell(2, 0) * m.getCell(0, 3);
        s[2][3] += getCell(2, 1) * m.getCell(1, 3);
        s[2][3] += getCell(2, 2) * m.getCell(2, 3);
        s[2][3] += getCell(2, 3) * m.getCell(3, 3);

        s[3][0] = getCell(3, 0) * m.getCell(0, 0);
        s[3][0] += getCell(3, 1) * m.getCell(1, 0);
        s[3][0] += getCell(3, 2) * m.getCell(2, 0);
        s[3][0] += getCell(3, 3) * m.getCell(3, 0);

        s[3][1] = getCell(3, 0) * m.getCell(0, 1);
        s[3][1] += getCell(3, 1) * m.getCell(1, 1);
        s[3][1] += getCell(3, 2) * m.getCell(2, 1);
        s[3][1] += getCell(3, 3) * m.getCell(3, 1);

        s[3][2] = getCell(3, 0) * m.getCell(0, 2);
        s[3][2] += getCell(3, 1) * m.getCell(1, 2);
        s[3][2] += getCell(3, 2) * m.getCell(2, 2);
        s[3][2] += getCell(3, 3) * m.getCell(3, 2);

        s[3][3] = getCell(3, 0) * m.getCell(0, 3);
        s[3][3] += getCell(3, 1) * m.getCell(1, 3);
        s[3][3] += getCell(3, 2) * m.getCell(2, 3);
        s[3][3] += getCell(3, 3) * m.getCell(3, 3);


        setCell(0, 0, s[0][0]);
        setCell(0, 1, s[0][1]);
        setCell(0, 2, s[0][2]);
        setCell(0, 3, s[0][3]);

        setCell(1, 0, s[1][0]);
        setCell(1, 1, s[1][1]);
        setCell(1, 2, s[1][2]);
        setCell(1, 3, s[1][3]);

        setCell(2, 0, s[2][0]);
        setCell(2, 1, s[2][1]);
        setCell(2, 2, s[2][2]);
        setCell(2, 3, s[2][3]);

        setCell(3, 0, s[3][0]);
        setCell(3, 1, s[3][1]);
        setCell(3, 2, s[3][2]);
        setCell(3, 3, s[3][3]);

    }

    inline VecPosition transform(const VecPosition& p) const {

        double x, y, z;

        x = getCell(0, 0) * p.getX() + getCell(0, 1) * p.getY() + getCell(0, 2) * p.getZ() + getCell(0, 3);
        y = getCell(1, 0) * p.getX() + getCell(1, 1) * p.getY() + getCell(1, 2) * p.getZ() + getCell(1, 3);
        z = getCell(2, 0) * p.getX() + getCell(2, 1) * p.getY() + getCell(2, 2) * p.getZ() + getCell(2, 3);

        VecPosition result = VecPosition(x, y, z);
        return result;
    }

    inline bool isIdentity() {
        bool result = true;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4 && result; j++) {
                result &= getCell(i, j) == (i == j ? 1.0 : 0.0);
            }
        }
        return result;
    }

    inline HCTMatrix getInverse() const {
        double m[16], inv[16], det;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m[i * 4 + j] = getCell(i, j);
            }
        }

        inv[0] = m[5]  * m[10] * m[15] -
                 m[5]  * m[11] * m[14] -
                 m[9]  * m[6]  * m[15] +
                 m[9]  * m[7]  * m[14] +
                 m[13] * m[6]  * m[11] -
                 m[13] * m[7]  * m[10];

        inv[4] = -m[4]  * m[10] * m[15] +
                 m[4]  * m[11] * m[14] +
                 m[8]  * m[6]  * m[15] -
                 m[8]  * m[7]  * m[14] -
                 m[12] * m[6]  * m[11] +
                 m[12] * m[7]  * m[10];

        inv[8] = m[4]  * m[9] * m[15] -
                 m[4]  * m[11] * m[13] -
                 m[8]  * m[5] * m[15] +
                 m[8]  * m[7] * m[13] +
                 m[12] * m[5] * m[11] -
                 m[12] * m[7] * m[9];

        inv[12] = -m[4]  * m[9] * m[14] +
                  m[4]  * m[10] * m[13] +
                  m[8]  * m[5] * m[14] -
                  m[8]  * m[6] * m[13] -
                  m[12] * m[5] * m[10] +
                  m[12] * m[6] * m[9];

        inv[1] = -m[1]  * m[10] * m[15] +
                 m[1]  * m[11] * m[14] +
                 m[9]  * m[2] * m[15] -
                 m[9]  * m[3] * m[14] -
                 m[13] * m[2] * m[11] +
                 m[13] * m[3] * m[10];

        inv[5] = m[0]  * m[10] * m[15] -
                 m[0]  * m[11] * m[14] -
                 m[8]  * m[2] * m[15] +
                 m[8]  * m[3] * m[14] +
                 m[12] * m[2] * m[11] -
                 m[12] * m[3] * m[10];

        inv[9] = -m[0]  * m[9] * m[15] +
                 m[0]  * m[11] * m[13] +
                 m[8]  * m[1] * m[15] -
                 m[8]  * m[3] * m[13] -
                 m[12] * m[1] * m[11] +
                 m[12] * m[3] * m[9];

        inv[13] = m[0]  * m[9] * m[14] -
                  m[0]  * m[10] * m[13] -
                  m[8]  * m[1] * m[14] +
                  m[8]  * m[2] * m[13] +
                  m[12] * m[1] * m[10] -
                  m[12] * m[2] * m[9];

        inv[2] = m[1]  * m[6] * m[15] -
                 m[1]  * m[7] * m[14] -
                 m[5]  * m[2] * m[15] +
                 m[5]  * m[3] * m[14] +
                 m[13] * m[2] * m[7] -
                 m[13] * m[3] * m[6];

        inv[6] = -m[0]  * m[6] * m[15] +
                 m[0]  * m[7] * m[14] +
                 m[4]  * m[2] * m[15] -
                 m[4]  * m[3] * m[14] -
                 m[12] * m[2] * m[7] +
                 m[12] * m[3] * m[6];

        inv[10] = m[0]  * m[5] * m[15] -
                  m[0]  * m[7] * m[13] -
                  m[4]  * m[1] * m[15] +
                  m[4]  * m[3] * m[13] +
                  m[12] * m[1] * m[7] -
                  m[12] * m[3] * m[5];

        inv[14] = -m[0]  * m[5] * m[14] +
                  m[0]  * m[6] * m[13] +
                  m[4]  * m[1] * m[14] -
                  m[4]  * m[2] * m[13] -
                  m[12] * m[1] * m[6] +
                  m[12] * m[2] * m[5];

        inv[3] = -m[1] * m[6] * m[11] +
                 m[1] * m[7] * m[10] +
                 m[5] * m[2] * m[11] -
                 m[5] * m[3] * m[10] -
                 m[9] * m[2] * m[7] +
                 m[9] * m[3] * m[6];

        inv[7] = m[0] * m[6] * m[11] -
                 m[0] * m[7] * m[10] -
                 m[4] * m[2] * m[11] +
                 m[4] * m[3] * m[10] +
                 m[8] * m[2] * m[7] -
                 m[8] * m[3] * m[6];

        inv[11] = -m[0] * m[5] * m[11] +
                  m[0] * m[7] * m[9] +
                  m[4] * m[1] * m[11] -
                  m[4] * m[3] * m[9] -
                  m[8] * m[1] * m[7] +
                  m[8] * m[3] * m[5];

        inv[15] = m[0] * m[5] * m[10] -
                  m[0] * m[6] * m[9] -
                  m[4] * m[1] * m[10] +
                  m[4] * m[2] * m[9] +
                  m[8] * m[1] * m[6] -
                  m[8] * m[2] * m[5];

        det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

        HCTMatrix result = HCTMatrix();
        if (det == 0) {
            return result;
        }

        det = 1.0 / det;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.setCell(i, j, inv[i * 4 + j] * det);
            }
        }
        return result;
    }

    void display();
};

#endif // HCTMATRIX_H

