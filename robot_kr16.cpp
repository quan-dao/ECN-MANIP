#include <robot_kr16.h>
#include <trig_solvers.h>

// Model of Kuka KR16 robot
const double r1 = 0.675;
const double a2 = 0.26;
const double a3 = 0.68;
const double a4 = 0.035;
const double r4 = 0.67;

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

    /* WRIST POSITIONING */
    // get position of the wrist in fixed frame
    auto ftw = fMw.getTranslationVector();

    // q2 & q3
    double z1 = r1 - ftw[2];
    double z2 = a2 + sqrt(ftw[0] * ftw[0] + ftw[1] * ftw[1]);
    double z2_neg = a2 - sqrt(ftw[0] * ftw[0] + ftw[1] * ftw[1]);
    double q23_min = q_min[1] + q_min[2];
    double q23_max = q_max[1] + q_max[2];

    // solve for q2 & q3 with z1 & z2
    auto sol_q2_q23 = solveType7(0, -a3, z1, z2, a4, r4, q_min[1], q_max[1], q23_min, q23_max);
    // solve for q2 & q3 with z1 & z2_neg
    for (auto qs: solveType7(0, -a3, z1, z2_neg, a4, r4, q_min[1], q_max[1], q23_min, q23_max)) {
        sol_q2_q23.push_back(qs);
    }

    // initilize IG solutions
    vpMatrix ig_solutions(6, sol_q2_q23.size()*2);

    // extract q2 & q3, solve for q1 & update IG solutions
    int j = 0;
    for (auto qs : sol_q2_q23) {
        // q2 & q3
        auto q2 = qs.qi;
        auto q3 = qs.qj - q2;
        // q1
        auto _coef_q1 = a2 + a3 * cos(q2) - a4 * sin(q2 + q3) + r4 * cos(q2 + q3);
        auto q1 = atan2(ftw[1]/(-_coef_q1), ftw[0]/_coef_q1);
        // update IG solutions
        ig_solutions[0][j] = q1;
        ig_solutions[1][j] = q2;
        ig_solutions[2][j] = q3;
        ig_solutions[0][j + 1] = q1;
        ig_solutions[1][j + 1] = q2;
        ig_solutions[2][j + 1] = q3;
        // move to next pair of columns
        j = j + 2;
    }
    /* End: WRIST POSITIONING */

    /* WRIST ORIENTING */
    for (int col_offset = 0; col_offset < sol_q2_q23.size(); ++col_offset){
        // Compute fR3
        const auto q1 = ig_solutions[0][0 + 2 * col_offset];
        const auto q2 = ig_solutions[1][0 + 2 * col_offset];
        const auto q3 = ig_solutions[2][0 + 2 * col_offset];
        const double s23 = sin(q2 + q3);
        const double s1 = sin(q1);
        const double c23 = cos(q2 + q3);
        const double c1 = cos(q1);
        vpMatrix fR3(3, 3);
        fR3[0][0] = -s23 * c1;
        fR3[1][0] = s1 * s23;
        fR3[2][0] = -c23;
        fR3[0][1] = -c1 * c23;
        fR3[1][1] = s1 * c23;
        fR3[2][1] = s23;
        fR3[0][2] = s1;
        fR3[1][2] = c1;
        fR3[2][2] = 0;

        // orientation of Wrist relative to frame 3
        auto _3Rw = fR3.transpose() * fMw.getRotationMatrix();

        // q5
        double q5 = acos(_3Rw[1][2]);
        double q5_neq = -q5;
        if (inAngleLimits(q5, q_min[4], q_max[4])) {
            ig_solutions[4][0 + 2 * col_offset] = q5;
        }
        if (inAngleLimits(q5_neq, q_min[4], q_max[4])) {
            ig_solutions[4][1 + 2 * col_offset] = q5_neq;
        }

        // q4 & q6
        if (!isNull(sin(q5))) {
            double q6 = atan2(-_3Rw[1][1] / sin(q5), _3Rw[1][0] / sin(q5));
            double q4 = atan2(_3Rw[2][2] / sin(q5), -_3Rw[0][2] / sin(q5));
            // update IG solutions
            if (inAngleLimits(q6, q_min[5], q_max[5]) && inAngleLimits(q4, q_min[3], q_max[3])) {
                ig_solutions[3][0 + 2 * col_offset] = q4;
                ig_solutions[5][0 + 2 * col_offset] = q6;
                ig_solutions[3][1 + 2 * col_offset] = M_PI + q4;
                ig_solutions[5][1 + 2 * col_offset] = M_PI + q6;
            }
        } else {
            // Singularity
            double q64 = atan2(-_3Rw[0][1], _3Rw[0][0]);
            double q6 = q0[5];  // last value of q6
            double q4 = (q64 - q6) * cos(q5);  // already take into account c5 = 1 & c5 = -1
            // update IG solutions
            if (inAngleLimits(q6, q_min[5], q_max[5]) && inAngleLimits(q4, q_min[3], q_max[3])) {
                ig_solutions[3][0 + 2 * col_offset] = q4;
                ig_solutions[5][0 + 2 * col_offset] = q6;
                ig_solutions[3][1 + 2 * col_offset] = q4;
                ig_solutions[5][1 + 2 * col_offset] = q6;
            }
        }

    }
    /* End Wrist Orienting*/

    // Choose the bestCandidate
    int best_col = 0;
    vpColVector dist_vector = ig_solutions.getCol(best_col) - q0;
    double min_dist = dist_vector.transpose() * dist_vector;
    for (int j = 1; j < sol_q2_q23.size()*2; ++j) {
        dist_vector = ig_solutions.getCol(j) - q0;
        if (dist_vector.transpose() * dist_vector < min_dist) {
            best_col = j;
        }
//        if (!isNull(ig_solutions[0][j])){
//            best_col = j;
//        }
    }

//     return bestCandidate(q0);
    return ig_solutions.getCol(best_col);
}


vpMatrix ecn::RobotKr16::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);
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
