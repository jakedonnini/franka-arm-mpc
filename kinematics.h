#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense> 
#include <vector>

// Compute the Denavit-Hartenberg homogeneous transformation
Eigen::Matrix4d Ai(double a_i, double alpha_i, double d_i, double theta_i);

// Forward kinematics for 7-DOF Panda arm
// INPUT:
//   q - vector of 7 joint angles [q0, q1, ..., q6]
// OUTPUT:
//   jointPositions - 8x3 matrix: positions of each joint + end effector in world frame
//   T0e - 4x4 homogeneous transform of the end effector
//   T_list - vector of 4x4 transforms from base to each joint + end effector
void forwardKinematics(
    Eigen::Vector<double, 7>& q,
    Eigen::Matrix<double, 8, 3>& jointPositions,
    Eigen::Matrix4d& T0e,
    std::vector<Eigen::Matrix4d>& T_list
);

// Compute the geometric Jacobian for the 7-DOF Panda arm
// INPUT:  
//   q - vector of 7 joint angles [q0, q1, ..., q6]
//   T_list - vector of 4x4 transforms from base to each joint + end effector
// OUTPUT:
//   J - 6x7 Jacobian matrix
void computeJacobian(const Eigen::Vector<double, 7>& q,
                     const std::vector<Eigen::Matrix4d>& T_list,
                     Eigen::Matrix<double, 6, 7>& J);

// Compute the difference in joint angles to move the end-effector towards the target
// INPUT:
//   T_current - current end-effector pose (4x4 homogeneous transform) 
//   T_target - target end-effector pose (4x4 homogeneous transform)
// INOUT:
//   dq - vector of joint angle differences (size 7)
void diff_to_target(const Eigen::Matrix4d& T_current,
                    const Eigen::Matrix4d& T_target,
                    Eigen::VectorXd& dq);

#endif // KINEMATICS_H
