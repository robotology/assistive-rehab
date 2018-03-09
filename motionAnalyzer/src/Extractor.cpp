#include "Extractor.h"

using namespace std;
using namespace yarp::math;

void Extractor::configure(Matrix &joint_mat_)
{
    joint_mat = joint_mat_;
}

double Extractor::computeRom()
{
    Vector j1, j2, j3;
    j1 = joint_mat.getCol(0);
    j2 = joint_mat.getCol(1);
    j3 = joint_mat.getCol(2);

    cout << j1(0) << " " << j1(1) << " " << j1(2);

    double a_norm = norm(j1-j2);
    double b_norm = norm(j1-j3);

    double dot_p = dot(j1-j2, j1-j3);

    return ( acos(dot_p/(a_norm*b_norm)) * (180/M_PI) );
}
