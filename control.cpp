#include <robot_init.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // robot properties
    const vpColVector vMax = robot->vMax();
    const vpColVector aMax = robot->aMax();

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);        // joint position setpoint for initial and final poses
    double t, t0, tf;

    // main control loop
    while(robot->ok())
    {
        // current time
        t = ros::Time::now().toSec();

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            t0 = t;
        }


        // get current joint positions
        q = robot->jointPosition();
        //cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ecn::Robot::MODE_POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // TODO: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->vw();

            // TODO: fill the fJw function
            // TODO: compute vCommand
            vpMatrix fSe(6,6);  // initialize fSe
            // concatenate fSe
            ecn::putAt(fSe, M.getRotationMatrix(), 0, 0);
            ecn::putAt(fSe, M.getRotationMatrix(), 3, 3);

            // write operational velocity in fixed frame
            vpColVector fVe;
            fVe = fSe * v;

            // compute joint velocity
            vCommand = robot->fJe(q).pseudoInverse() * fVe;

            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // TODO: fill the inverseGeometry function
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }


        else if(robot->mode() == ecn::Robot::MODE_INTERP_P2P)
        {
            // reach Md with interpolated joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);
                // find tf
                tf = 0;
                for (int i = 0; i < vMax.size(); i++) {
//                    double cand = vMax[i] / aMax[i] + (qf[i] - q0[i]) / vMax[i];  // trapezoidal profile
                	double dq = qf[i] - q0[i];
                    double cand = 1.5 * std::abs(dq) / vMax[i];
                    if (cand > tf)
                        {tf = cand;}
                    cand = std::sqrt(6 * std::abs(dq) / aMax[i]);
                    if (cand > tf)
                        {tf = cand;}
                }
            }
            // compute joitn command
            double ratio = (t - t0) / tf;
            if (ratio < 1) {
                for (int i = 0; i < vMax.size(); i++) {
                    // double to = vMax[i] / aMax[i];
                    // if (_t < to) {
                    //     qCommand[i] = q0[i] + 0.5 * aMax[i] *pow(_t, 2);
                    // } else if (_t < tf - to) {
                    //     qCommand[i] = q0[i] + vMax[i] * (_t - 0.5 * to);
                    // } else {
                    //     qCommand[i] = qf[i] - 0.5 * aMax[i] * pow((tf - _t), 2);
                    // }
                    double dq = qf[i] - q0[i];
                    double P = 3 * pow(ratio, 2) - 2 * pow(ratio, 3);
                    qCommand[i] = q0[i] + P * dq;
                }
            } else {
                qCommand = qf;
            }

        //   cout<<"t ="<<t-t0<<"\tq1 = "<<qCommand[0]<<"\tq2 = "<<qCommand[1]<<"\tq3 = "<<qCommand[2]<<"\tq4 = "<<qCommand[3]<<"\tq5 = "<<qCommand[4]<<"\tq6 = "<<qCommand[5]<<"\n";
            // TODO: compute qCommand from q0, qf, t, t0 and tf
            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;

            // TODO: compute qCommand from M0, Md, t, t0 and tf
            // use robot->intermediaryPose to build poses between M0 and Md
            double alpha = (t - t0) / 1;
            auto Mt = robot->intermediaryPose(M0, Md, alpha);
            qCommand = robot->inverseGeometry(Mt, q);
            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_P2P)
        {
            // go to Md using operational velocity

            // TODO: compute joint velocity command

            // compute pose error between current pose and desired pose, relative to desired pose
            vpPoseVector e;
            auto eStar_M_e = Md.inverse() * M;
            e.buildFrom(eStar_M_e);

            // construct W
            vpMatrix W(6, 6);
            for(int i = 0; i < 6; ++i)
                W[i][i] = robot->lambda();
//            // try to mess thing up
//            W[1][1] *= 2;
//            W[2][2] *= 0.5;

            // compute desired value of velocity screw of E-E in E-E frame
            auto e_v_e = eStar_M_e.getRotationMatrix().inverse() * e.getTranslationVector();  // translational velocity of frame{E} in frame{E}
            auto e_omega_e = e.getThetaUVector();  // rotational velocity of frame{E} in frame{E}
            vpColVector eVe(6);  // velocity screw
            ecn::putAt(eVe, e_v_e, 0);
            ecn::putAt(eVe, e_omega_e, 3);
            eVe = -W * eVe;

            // Transform eVe to fVe
            // construct f_Rbar_e
            vpMatrix f_Rbar_e(6, 6);
            ecn::putAt(f_Rbar_e, M.getRotationMatrix(), 0, 0);
            ecn::putAt(f_Rbar_e, M.getRotationMatrix(), 3, 3);
            auto fVe = f_Rbar_e * eVe;

            // Compute joint velocity
            vCommand = robot->fJe(q).pseudoInverse() * fVe;

            robot->setJointVelocity(vCommand);
            //break;
        }


    }
}
