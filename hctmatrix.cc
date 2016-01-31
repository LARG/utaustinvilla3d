#include "hctmatrix.h"

HCTMatrix::HCTMatrix() {

    createIdentity();
}

HCTMatrix::HCTMatrix(const HCTMatrix& m) {

    copy(m);
}

HCTMatrix::HCTMatrix(const int &mode) {

    if(mode != HCT_IDENTITY) {
        std::cout << "(HCTMatrix) Constructor Error; mode: " << mode << ".\n";
    }

    createIdentity();
}

HCTMatrix::HCTMatrix(const int &mode, const double &rotateAngle) {

    if(mode == HCT_ROTATE_X) {
        createRotateX(rotateAngle);
    }
    else if(mode == HCT_ROTATE_Y) {
        createRotateY(rotateAngle);
    }
    else if(mode == HCT_ROTATE_Z) {
        createRotateZ(rotateAngle);
    }
    else {
        std::cout << "(HCTMatrix) Constructor Error; mode: " << mode << "; rotateAngle: " << rotateAngle << ".\n";
    }
}

HCTMatrix::HCTMatrix(const int &mode, const VecPosition &axis, const double &rotateAngle) {

    if(mode == HCT_GENERALIZED_ROTATE) {
        createRotateGeneral(axis, rotateAngle);
    }
    else {
        std::cout << "(HCTMatrix) Constructor Error; mode: " << mode << "; axis: " << axis << "; rotateAngle: " << rotateAngle << ".\n";
    }
}

HCTMatrix::HCTMatrix(const int &mode, const VecPosition &translateVector) {

    if(mode != HCT_TRANSLATE) {
        std::cout << "(HCTMatrix) Constructor Error; mode: " << mode << "; translateVector: " << translateVector << ".\n";
    }

    createTranslate(translateVector);
}

void HCTMatrix::display() {

    std::cout << "\n";

    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {

            std::cout << getCell(i, j) << "\t";
        }

        std::cout << "\n";
    }
}

