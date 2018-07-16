#include <robot_turret.h>

// Model of Turret robot

// Any constant transform at base of end-effector
void ecn::RobotTurret::init_wMe()
{

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;

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



    return J;
}
