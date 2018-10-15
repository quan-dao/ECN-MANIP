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
    double d = 0.1;
    double b = 0.5;
    vpColVector q(dofs);
    vpMatrix ig_solutions(3, 2);

    double q1_pos = atan2(-Md[0][1], Md[1][1]);
    double q1_neg = q1_pos +M_PI;

    std::vector<double> q1_list = {q1_pos, q1_neg};
    for (int i = 0; i < 2; ++i) {
        auto q1 = q1_list[i];
        if (inAngleLimits(q1, q_min[0], q_max[0])) {
            double q2, q3;

            if (isNull(sin(q1))) {
                q2 = atan2(-Md[0][2]/cos(q1), Md[0][0]/cos(q1));
            } else {
                q2 = atan2(-Md[1][2]/sin(q1), Md[1][0]/sin(q1));
            }
//            q2 = atan2(Md[2][0], Md[2][2]);
//            if (inAngleLimits(q2, q_min[1], q_max[1])) {
//                //
//            }

            if (inAngleLimits(q2, q_min[1], q_max[1])) {
                if (!isNull(cos(q2)))
                    q3 = (Md[2][3] - b) / cos(q2) - d;
                else
                    if(!isNull(cos(q1)))
                        q3 = -Md[0][3]/(sin(q2) * cos(q1)) - d;
                    else
                        q3 = -Md[1][3]/(sin(q2) * sin(q1)) - d;
            }
            // update
            ig_solutions[0][i] = q1;
            ig_solutions[1][i] = q2;
            ig_solutions[2][i] = q3;
        }
    }

    vpColVector dist_vector = ig_solutions.getCol(0) - q0;
    double min_dist = dist_vector.transpose() * dist_vector;

    vpColVector dist_vector_1 = ig_solutions.getCol(1) - q0;
    if (dist_vector_1.transpose() * dist_vector_1 < min_dist) {
        return ig_solutions.getCol(1);
    }

    return ig_solutions.getCol(0);
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
