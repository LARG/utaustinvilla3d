#include "NMatrix.h"
#include <string.h>

// Constructors
NMatrix::NMatrix()
{
    M = 0;
    N = 0;
    X = 0;
}

NMatrix::NMatrix(int m, int n, bool I/*= false*/)
{
    M=m;
    N=n;
    X=new float [m*n];
    //Initialise NMatrix to zero
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            (*this)[i][j] = 0;
        }
    }
    //Identity NMatrix Initialisation
    if (I)
    {
        if (m != n)
            return;

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                (*this)[i][j] = (i == j) ? 1 : 0;
            }
        }
    }
}

// Copy Constructor
NMatrix::NMatrix(const NMatrix& a)
{
    M=a.M;
    N=a.N;
    X=new float [M*N];
    memcpy(X,a.X,sizeof(float)*M*N);
}

// Destructor
NMatrix::~NMatrix()
{
    delete [] X;
    X = 0;
}

// NMatrix Index Operator
// Returns a pointer to the ith row in the NMatrix
float*	NMatrix::operator []	(int i)	const
{
    return &X[i*N];
}

// NMatrix Addition
NMatrix operator + (const NMatrix& a, const NMatrix& b)
{
    NMatrix addAns(a.getm(),a.getn());
    int i=0,j=0;
    if ((a.getn()==b.getn())&&(a.getm()==b.getm()))
    {
        for (i=0; i<a.getm(); i++)
        {
            for (j=0; j<a.getn(); j++)
            {
                addAns[i][j]=a[i][j]+b[i][j];
            }
        }
    }
    return addAns;
    //This return calls the copy constructor which copies the NMatrix into another block of memory
    //and then returns the pointer to this new memory.
    //Otherwise the array addAns is deleted by the destructor here and the pointer returned from the addition
    //is a pointer to deleted memory. This causes problems when the function calling this tries to delete this memory again.
}

// NMatrix Subtraction
NMatrix	operator -  (const NMatrix& a, const NMatrix& b)
{
    NMatrix subAns(a.getm(),a.getn());
    int i=0,j=0;
    if ((a.getn()==b.getn())&&(a.getm()==b.getm()))
    {
        for (i=0; i<a.getm(); i++)
        {
            for (j=0; j<a.getn(); j++)
            {
                subAns[i][j]=a[i][j]-b[i][j];
            }
        }
    }
    return subAns;
}

//NMatrix Multiplication
NMatrix	operator * (const NMatrix& a, const NMatrix& b)
{
    NMatrix multAns(a.getm(),b.getn());
    int i=0,j=0,k=0;
    if (a.getn()==b.getm())
    {
        for (i=0; i<a.getm(); i++)
        {
            for (j=0; j<b.getn(); j++)
            {
                float temp=0;
                for (k=0; k<a.getn(); k++)
                {
                    temp+=a[i][k]*b[k][j];
                }
                multAns[i][j]=temp;
            }
        }
    }
    return multAns;
}

// NMatrix Multiplication by a Scalar
NMatrix	operator * (const float& a, const NMatrix& b)
{
    NMatrix multAns(b.getm(),b.getn());
    int i=0,j=0;
    for (i=0; i<b.getm(); i++)
    {
        for (j=0; j<b.getn(); j++)
        {
            multAns[i][j]=b[i][j]*a;
        }
    }
    return multAns;
}

// NMatrix Multiplication by a Scalar
NMatrix	operator * (const NMatrix& a, const float& b)
{
    NMatrix multAns(a.getm(),a.getn());
    int i=0,j=0;
    for (i=0; i<a.getm(); i++)
    {
        for (j=0; j<a.getn(); j++)
        {
            multAns[i][j]=a[i][j]*b;
        }
    }
    return multAns;
}

// NMatrix Division by a Scalar
NMatrix	operator / (const NMatrix& a, const float& b)
{
    NMatrix divAns(a.getm(),a.getn());
    int i=0,j=0;
    for (i=0; i<a.getm(); i++)
    {
        for (j=0; j<a.getn(); j++)
        {
            divAns[i][j]=a[i][j]/b;
        }
    }
    return divAns;
}

// NMatrix Equality
NMatrix& NMatrix::operator =  (const NMatrix& a)
{
    if (X!=0)
        delete [] X;
    M=a.M;
    N=a.N;
    X=new float [M*N];
    memcpy(X,a.X,sizeof(float)*M*N);
    return *this;
}

// NMatrix Transpose
NMatrix	NMatrix::transp()
{
    NMatrix transpAns(getn(),getm());
    int i=0,j=0;
    for (i=0; i<getm(); i++)
    {
        for (j=0; j<getn(); j++)
        {
            transpAns[j][i]=(*this)[i][j];
        }
    }
    return transpAns;
}

NMatrix NMatrix::getRow(int index)
{
    NMatrix Row (1,getn());
    int i=0;
    for(i=0; i<getn(); i++) {
        Row[0][i]=(*this)[index][i];
    }
    return Row;
}

NMatrix  NMatrix::getCol(int index)
{
    NMatrix Col (getm(),1);
    int i=0;
    for(i=0; i<getm(); i++) {
        Col[i][0]=(*this)[i][index];
    }
    return Col;
}

void  NMatrix::setRow(int index, NMatrix in)
{
    int i=0;
    for(i=0; i<getn(); i++) {
        (*this)[index][i]=in[0][i];
    }
}

void  NMatrix::setCol(int index, NMatrix in)
{
    int i=0;
    for(i=0; i<getm(); i++) {
        (*this)[i][index]=in[i][0];
    }
}

// 2x2 NMatrix Inversion
NMatrix Invert22(const NMatrix& a)
{
    NMatrix invertAns(a.getm(),a.getn());
    invertAns[0][0]=a[1][1];
    invertAns[0][1]=-a[0][1];
    invertAns[1][0]=-a[1][0];
    invertAns[1][1]=a[0][0];
    float divisor=a[0][0]*a[1][1]-a[0][1]*a[1][0];
    invertAns=invertAns/divisor;
    return invertAns;
}

NMatrix vertcat(NMatrix a, NMatrix b)
{
    //NMatrix concatenation
    //assume same dimension on cols
    int mTotal=a.getm()+b.getm();
    NMatrix c=NMatrix(mTotal,a.getn(),false);
    for(int i=0; i<a.getm(); i++) {
        c.setRow(i,a.getRow(i));
    }
    for(int i=0; i<b.getm(); i++) {
        c.setRow(i+a.getm(),b.getRow(i));
    }
    return c;
}

NMatrix horzcat(NMatrix a, NMatrix b)
{
    //NMatrix concatenation
    //assume same dimension on rows
    int nTotal=a.getn()+b.getn();
    NMatrix c=NMatrix(a.getm(),nTotal,false);
    for(int i=0; i<a.getn(); i++) {
        c.setCol(i,a.getCol(i));
    }
    for(int i=0; i<b.getn(); i++) {
        c.setCol(i+a.getn(),b.getCol(i));
    }
    return c;
}

NMatrix diagcat(NMatrix a, NMatrix b)
{
    //NMatrix concatenation
    //assume both NMatrix a and b are square
    int mTotal=a.getm()+b.getm();
    int nTotal=a.getn()+b.getn();
    NMatrix c=NMatrix(mTotal,nTotal,false);
    for(int i=0; i<a.getm(); i++) {
        for(int j=0; j<a.getn(); j++) {
            c[i][j]=a[i][j];
        }
    }
    for(int i=0; i<b.getm(); i++) {
        for(int j=0; j<b.getn(); j++) {
            c[i+a.getm()][j+a.getn()]=b[i][j];
        }
    }
    return c;
}
NMatrix cholesky(NMatrix P)
{
    NMatrix L = NMatrix (P.getm(),P.getn(),false);
    float a=0;
    for(int i=0; i<P.getm(); i++) {
        for(int j=0; j<i; j++) {
            a=P[i][j];
            for(int k=0; k<j; k++) {
                a=a-L[i][k]*L[j][k];
            }
            L[i][j]=a/L[j][j];
        }
        a=P[i][i];
        for(int k=0; k<i; k++) {
            a=a-powf(L[i][k],2);
        }
        L[i][i]=sqrtf(a);
    }
    return L;
}
NMatrix HT(NMatrix A)
{
    //Householder Triangularization Algorithm
    //rows = n in the algorithm, for avoid confusion.
    int rows=A.getm();
    int r=A.getn()-rows;
    float sigma;
    float a;
    float b;
    std::vector <float> v (A.getn());
    NMatrix B= NMatrix(rows,rows,false);
    for(int k=rows-1; k>=0; k--) {
        sigma=0.0;
        for(int j=0; j<=r+k; j++) {
            sigma=sigma+A[k][j]*A[k][j];
        }
        a=sqrtf(sigma);
        sigma=0.0;
        for(int j=0; j<=r+k; j++) {
            if(j==r+k) {
                v.at(j)=(A[k][j]-a);
            }
            else {
                v.at(j)=(A[k][j]);
            }
            sigma=sigma+v.at(j)*v.at(j);
        }
        a=2.0/(sigma+1e-15);
        for(int i=0; i<=k; i++) {
            sigma=0.0;
            for(int j=0; j<=r+k; j++) {
                sigma=sigma+A[i][j]*v.at(j);
            }
            b=a*sigma;
            for(int j=0; j<=r+k; j++) {
                A[i][j]=A[i][j]-b*v.at(j);
            }
        }
    }
    for(int i=0; i<rows; i++) {
        B.setCol(i,A.getCol(r+i));
    }
    return B;
}


/*! Overloaded version of the C++ output operator for output.
  \param os output stream to which information should be written
  \param a a NMatrix which must be printed
  \return output stream  */
ostream& operator << ( ostream &os, NMatrix a )
{
    for (int i = 0; i < a.getm(); i++)
    {
        os << "[ ";
        for (int j = 0; j < a.getn(); j++)
        {
            os << a[i][j];
            if (j!=a.getn()-1) os << ", ";
        }
        os << "] " << endl;
    }

    return os;
}
