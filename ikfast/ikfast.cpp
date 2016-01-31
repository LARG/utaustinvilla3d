#include <cassert>
#define IKFAST_NO_MAIN

#include "ikfast.h"

// Left leg
#define IKFAST_NAMESPACE left
#include "ikfast_left_foot.cpp"
#undef IKFAST_NAMESPACE

// Right leg
#define IKFAST_NAMESPACE right
#include "ikfast_right_foot.cpp"
#undef IKFAST_NAMESPACE

#define ikfast_foot(side)                                               \
bool ikfast_##side##_foot_ik(const double *eetrans, const double *eerot,\
        std::vector<joints_t> &solutions) {                             \
    using namespace std;                                                \
    using namespace side;                                               \
                                                                        \
    vector<IKSolution> vsolutions;                                      \
    bool bSuccess = ik(eetrans, eerot, NULL, vsolutions);               \
                                                                        \
    solutions.resize(vsolutions.size());                                \
    for(size_t i=0; i<vsolutions.size(); ++i) {                         \
        assert(getNumJoints() == 6);                                    \
        solutions[i].resize(6);                                         \
        vsolutions[i].GetSolution(&solutions[i][0], NULL);              \
    }                                                                   \
    return bSuccess;                                                    \
}                                                                       \
                                                                        \
void ikfast_##side##_foot_fk(const double* j, double* eetrans,          \
              double* eerot) {                                          \
    side::fk(j, eetrans, eerot);                                        \
}

ikfast_foot(left);
ikfast_foot(right);


#undef IKFAST_NO_MAIN
