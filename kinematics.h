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
    const std::vector<double>& q,
    Eigen::Matrix<double, 8, 3>& jointPositions,
    Eigen::Matrix4d& T0e,
    std::vector<Eigen::Matrix4d>& T_list
);

#endif // KINEMATICS_H
