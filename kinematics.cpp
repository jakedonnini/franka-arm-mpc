#define _USE_MATH_DEFINES

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

// Helper function: homogeneous transform using DH parameters
Eigen::Matrix4d Ai(double a_i, double alpha_i, double d_i, double theta_i) {
    Eigen::Matrix4d T;
    T << cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i),
         sin(theta_i), cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i),
         0,            sin(alpha_i),               cos(alpha_i),                d_i,
         0,            0,                           0,                          1;
    return T;
}


// Forward kinematics function
void forwardKinematics(Eigen::Vector<double, 7>& q,
             Eigen::Matrix<double, 8, 3>& jointPositions,
             Eigen::Matrix4d& T0e,
             std::vector<Eigen::Matrix4d>& T_list)
{
    // Link lengths
    double l1 = 0.141, l2 = 0.192, l3 = 0.195, l4 = 0.121, l5 = 0.0825;
    double l6 = 0.0825, l7 = 0.125, l8 = 0.259, l9 = 0.088, l10 = 0.051;
    double l11 = 0.159, l12 = 0.015;

    // Compute transforms
    Eigen::Matrix4d T01 = Ai(0, M_PI/2, l1+l2, q[0]);
    Eigen::Matrix4d T12 = Ai(0, -M_PI/2, 0, -q[1]);
    Eigen::Matrix4d T23 = Ai(l6, M_PI/2, l3+l4, q[2]);
    Eigen::Matrix4d T34 = Ai(-l5, -M_PI/2, 0, q[3]);
    Eigen::Matrix4d T45 = Ai(0, M_PI/2, l7+l8, q[4]);
    Eigen::Matrix4d T56 = Ai(l9, M_PI/2, 0, q[5]);
    Eigen::Matrix4d T6e = Ai(0, 0, l10+l11, q[6]-M_PI/4);

    Eigen::Matrix4d T02 = T01*T12;
    Eigen::Matrix4d T03 = T02*T23;
    Eigen::Matrix4d T04 = T03*T34;
    Eigen::Matrix4d T05 = T04*T45;
    Eigen::Matrix4d T06 = T05*T56;
    T0e = T06*T6e;

    // Store transforms
    T_list.clear();
    T_list.push_back(Eigen::Matrix4d::Identity());
    T_list.push_back(T01);
    T_list.push_back(T02);
    T_list.push_back(T03);
    T_list.push_back(T04);
    T_list.push_back(T05);
    T_list.push_back(T06);
    T_list.push_back(T0e);

    Eigen::Vector4d base(0,0,0,1);

    // Optional offsets
    Eigen::Matrix4d T0_offset = Ai(0,0,l1,0);
    Eigen::Matrix4d T02_offset = Ai(0,0,l3,0);
    Eigen::Matrix4d T04_offset = Ai(0,0,l7,0);
    Eigen::Matrix4d T05_offset = Ai(0,0,-l12,0);
    Eigen::Matrix4d T06_offset = Ai(0,0,l10,0);

    jointPositions.row(0) = (T0_offset*base).head<3>();
    jointPositions.row(1) = (T01*base).head<3>();
    jointPositions.row(2) = (T02*T02_offset*base).head<3>();
    jointPositions.row(3) = (T03*base).head<3>();
    jointPositions.row(4) = (T04*T04_offset*base).head<3>();
    jointPositions.row(5) = (T05*T05_offset*base).head<3>();
    jointPositions.row(6) = (T06*T06_offset*base).head<3>();
    jointPositions.row(7) = (T0e*base).head<3>();
}

// create the jacobian function
void computeJacobian(const Eigen::Vector<double, 7>& q,
             const std::vector<Eigen::Matrix4d>& T_list,
             Eigen::Matrix<double, 6, 7>& J)
{
    (void)q; // Suppress unused parameter warning - q not needed as jacobian computed from T_list
    Eigen::Vector3d pe = T_list.back().block<3,1>(0,3); // end-effector position

    for (int i = 0; i < 7; ++i) {
        Eigen::Vector3d zi = T_list[i].block<3,1>(0,2); // z-axis of the i-th joint
        Eigen::Vector3d pi = T_list[i].block<3,1>(0,3); // position of the i-th joint

        Eigen::Vector3d Ji_pos = zi.cross(pe - pi);
        Eigen::Vector3d Ji_ori = zi;

        J.block<3,1>(0,i) = Ji_pos;
        J.block<3,1>(3,i) = Ji_ori;
    }
}

// difference to target function
void diff_to_target(const Eigen::Matrix4d& T_current, const Eigen::Matrix4d& T_target, Eigen::VectorXd& dq) {
    Eigen::Vector3d p_current = T_current.block<3,1>(0,3);
    Eigen::Vector3d p_target = T_target.block<3,1>(0,3);
    Eigen::Vector3d dp = p_target - p_current;

    Eigen::Matrix3d R_current = T_current.block<3,3>(0,0);
    Eigen::Matrix3d R_target = T_target.block<3,3>(0,0);
    Eigen::Matrix3d R_diff = R_target * R_current.transpose();
    Eigen::AngleAxisd angleAxis(R_diff);
    Eigen::Vector3d dphi = angleAxis.angle() * angleAxis.axis();

    Eigen::VectorXd dx(6);
    dx.head<3>() = dp;
    dx.tail<3>() = dphi;

    // Simple proportional control for demonstration
    double gain = 1.0;
    
    // Ensure dq has the right size (should match the number of DOFs)
    if (dq.size() < 7) {
        dq.resize(7);
    }
    
    // For now, just use the first 7 components (or available DOFs)
    int min_size = std::min(static_cast<int>(dq.size()), 7);
    dq.head(min_size) = (gain * dx).head(min_size);
    
    // Zero out any remaining DOFs if dq is larger than 7
    if (dq.size() > 7) {
        dq.tail(dq.size() - 7).setZero();
    }
}