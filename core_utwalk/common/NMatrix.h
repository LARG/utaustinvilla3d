#ifndef _NMatrix_h_DEFINED
#define _NMatrix_h_DEFINED


#include <iostream>
#include <math.h>
#include <vector>
using namespace std;

class NMatrix
{
public:
    int		M;			// number of rows
    int		N;			// number of columns
    float*	X;			// matrix pointer

    int		getm()	const;		// return number  of rows
    int		getn()	const;		// return number  of columns
    float*	getx()	const;		// return pointer to array

    // Constructors
    NMatrix();
    NMatrix(int m, int n, bool I=false);
    ~NMatrix();
    NMatrix(const NMatrix& a);

    NMatrix transp(); // NMatrix Transpose
    NMatrix getRow(int index); // Get Row
    NMatrix getCol(int index); // Get Column
    void setRow(int index, NMatrix in); // Set Row
    void setCol(int index, NMatrix in); // Set Column
    NMatrix&	operator =  (const NMatrix& a); // Overloaded Operator
    float*	operator [] (int i)	const; // Overloaded Operator

    // output operator
    friend ostream& operator << ( ostream &os, NMatrix a );


};

// Overloaded Operators
NMatrix	operator +  (const NMatrix& a, const NMatrix& b);
NMatrix	operator -  (const NMatrix& a, const NMatrix& b);
NMatrix	operator *  (const NMatrix& a, const NMatrix& b);
NMatrix	operator *  (const float& a, const NMatrix& b);
NMatrix	operator *  (const NMatrix& a, const float& b);
NMatrix	operator /  (const NMatrix& a, const float& b);


// 2x2 Matrix Inversion
NMatrix Invert22(const NMatrix& a);

// concatenation
NMatrix horzcat(NMatrix a, NMatrix b);
NMatrix vertcat(NMatrix a, NMatrix b);
NMatrix diagcat(NMatrix a, NMatrix b);
NMatrix cholesky(NMatrix P);
NMatrix HT(NMatrix A);

inline float convDble(const NMatrix& a) {
    return a[0][0];    // Convert 1x1 matrix to Double
}
inline	 int		NMatrix::getm() const {
    return M;
}
inline	 int		NMatrix::getn() const {
    return N;
}
inline	 float*	NMatrix::getx() const {
    return X;
}

#endif
