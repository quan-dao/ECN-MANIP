#include <robot_kr16.h>
#include <trig_solvers.h>

// Model of Kuka KR16 robot

// Any end-effector to wrist constant transform
void ecn::RobotKr16::init_wMe()
{
    double wte = 0.158;
    wMe[1][1] = -1;
    wMe[2][2] = -1;
    wMe[2][3] = -wte;
}

// Direct Geometry fixed to wrist frame
vpHomogeneousMatrix ecn::RobotKr16::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    double r1 = 0.675;
    double a2 = 0.26;
    double a3 = 0.68;
    double a4 = 0.035;
    double r4 = 0.67;
    // Generated pose code
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double c4 = cos(q[3]);
    const double c5 = cos(q[4]);
    const double c6 = cos(q[5]);
    const double c23 = cos(q[1]+q[2]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    const double s4 = sin(q[3]);
    const double s5 = sin(q[4]);
    const double s6 = sin(q[5]);
    const double s23 = sin(q[1]+q[2]);
    M[0][0] = (-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*c6 - (s1*c4 - s4*s23*c1)*s6;
    M[0][1] = -(-(s1*s4 + s23*c1*c4)*c5 - s5*c1*c23)*s6 - (s1*c4 - s4*s23*c1)*c6;
    M[0][2] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
    M[0][3] = (a2 + a3*c2 - a4*s23 + r4*c23)*c1;
    M[1][0] = ((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*c6 - (s1*s4*s23 + c1*c4)*s6;
    M[1][1] = -((s1*s23*c4 - s4*c1)*c5 + s1*s5*c23)*s6 - (s1*s4*s23 + c1*c4)*c6;
    M[1][2] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
    M[1][3] = (-a2 - a3*c2 + a4*s23 - r4*c23)*s1;
    M[2][0] = (s5*s23 - c4*c5*c23)*c6 + s4*s6*c23;
    M[2][1] = -(s5*s23 - c4*c5*c23)*s6 + s4*c6*c23;
    M[2][2] = s5*c4*c23 + s23*c5;
    M[2][3] = -a3*s2 - a4*c23 + r1 - r4*s23;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1.;
    // End of pose code

    return M;
}


// Inverse Geometry
vpColVector ecn::RobotKr16::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    // desired wrist pose
    vpHomogeneousMatrix fMw = Md * wMe.inverse();


    return bestCandidate(q0);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);
//    double r1 = 0.675;
    double a2 = 0.26;
    double a3 = 0.68;
    double a4 = 0.035;
    double r4 = 0.67;
    // Generated Jacobian code
    const double c1 = cos(q[0]);
    const double c2 = cos(q[1]);
    const double c4 = cos(q[3]);
    const double c5 = cos(q[4]);
    const double c23 = cos(q[1]+q[2]);
    const double s1 = sin(q[0]);
    const double s2 = sin(q[1]);
    const double s4 = sin(q[3]);
    const double s5 = sin(q[4]);
    const double s23 = sin(q[1]+q[2]);
    J[0][0] = (-a2 - a3*c2 + a4*s23 - r4*c23)*s1;
    J[0][1] = -(a3*s2 + a4*c23 + r4*s23)*c1;
    J[0][2] = -(a4*c23 + r4*s23)*c1;
    //J[0][3] = 0;
    //J[0][4] = 0;
    //J[0][5] = 0;
    J[1][0] = -(a2 + a3*c2 - a4*s23 + r4*c23)*c1;
    J[1][1] = (a3*s2 + a4*c23 + r4*s23)*s1;
    J[1][2] = (a4*c23 + r4*s23)*s1;
    //J[1][3] = 0;
    //J[1][4] = 0;
    //J[1][5] = 0;
    //J[2][0] = 0;
    J[2][1] = -a3*c2 + a4*s23 - r4*c23;
    J[2][2] = a4*s23 - r4*c23;
    //J[2][3] = 0;
    //J[2][4] = 0;
    //J[2][5] = 0;
    //J[3][0] = 0;
    J[3][1] = s1;
    J[3][2] = s1;
    J[3][3] = -c1*c23;
    J[3][4] = s1*c4 - s4*s23*c1;
    J[3][5] = (s1*s4 + s23*c1*c4)*s5 - c1*c5*c23;
    //J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = c1;
    J[4][3] = s1*c23;
    J[4][4] = s1*s4*s23 + c1*c4;
    J[4][5] = -(s1*s23*c4 - s4*c1)*s5 + s1*c5*c23;
    J[5][0] = -1.;
    //J[5][1] = 0;
    //J[5][2] = 0;
    J[5][3] = s23;
    J[5][4] = -s4*c23;
    J[5][5] = s5*c4*c23 + s23*c5;
    // End of Jacobian code

    return J;
}
