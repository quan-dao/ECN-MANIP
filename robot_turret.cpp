#include <robot_turret.h>
#include <trig_solvers.h>

// Model of Turret robot

// Any constant transform at base of end-effector
void ecn::RobotTurret::init_wMe()
{
    vpHomogeneousMatrix M;
    M[0][0] = 1;
    M[0][1] = 0;
    M[0][2] = 0;
    M[0][3] = 0;
    M[1][0] = 0;
    M[1][1] = 1;
    M[1][2] = 0;
    M[1][3] = 0;
    M[2][0] = 0;
    M[2][1] = 0;
    M[2][2] = 1;
    M[2][3] = 0;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    double d = 0.1;
    double b = 0.5;
    // Generated pose code
        const double c1 = cos(q[0]);
        const double c2 = cos(q[1]);
        const double s1 = sin(q[0]);
        const double s2 = sin(q[1]);
        M[0][0] = c1*c2;
        M[0][1] = -s1;
        M[0][2] = -s2*c1;
        M[0][3] = (-d - q[2])*s2*c1;
        M[1][0] = s1*c2;
        M[1][1] = c1;
        M[1][2] = -s1*s2;
        M[1][3] = (-d - q[2])*s1*s2;
        M[2][0] = s2;
        M[2][1] = 0;
        M[2][2] = c2;
        M[2][3] = b + (d + q[2])*c2;
        M[3][0] = 0;
        M[3][1] = 0;
        M[3][2] = 0;
        M[3][3] = 1.;
    // End of pose code


    return M;
}


// Inverse Geometry
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &Md, const vpColVector &q0) const
{
    vpColVector q(dofs);


    return q;
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);
    double d = 0.1;
    double b = 0.5;
    // Generated Jacobian code
        const double c1 = cos(q[0]);
        const double c2 = cos(q[1]);
        const double s1 = sin(q[0]);
        const double s2 = sin(q[1]);
        J[0][0] = (d + q[2])*s1*s2;
        J[0][1] = (-d - q[2])*c1*c2;
        J[0][2] = -s2*c1;
        J[1][0] = (-d - q[2])*s2*c1;
        J[1][1] = (-d - q[2])*s1*c2;
        J[1][2] = -s1*s2;
        //J[2][0] = 0;
        J[2][1] = -(d + q[2])*s2;
        J[2][2] = c2;
        //J[3][0] = 0;
        J[3][1] = s1;
        //J[3][2] = 0;
        //J[4][0] = 0;
        J[4][1] = -c1;
        //J[4][2] = 0;
        J[5][0] = 1.;
        //J[5][1] = 0;
        //J[5][2] = 0;
    // End of Jacobian code

    return J;
}
