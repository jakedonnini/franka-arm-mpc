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

// Compute the Euclidean position difference from current to target end-effector pose
// INPUT:
//   T_current - current end-effector pose (4x4 homogeneous transform) 
//   T_target - target end-effector pose (4x4 homogeneous transform)
// OUTPUT:
//   Returns a 3D vector dp = p_target - p_current
Eigen::Vector3d diff_to_target(const Eigen::Matrix4d& T_current,
                               const Eigen::Matrix4d& T_target);

// Helper function for End Effector Orientation Task
// Computes the axis of rotation from the current orientation to the target orientation
// INPUT:
//   R_des - 3x3 matrix representing the desired orientation from end effector to world
//   R_curr - 3x3 matrix representing the current end effector orientation  
// OUTPUT:
//   omega - 3-element vector containing the axis of rotation from current to desired frame
//           The magnitude of this vector is sin(angle), where angle is the rotation angle
Eigen::Vector3d calcAngDiff(const Eigen::Matrix3d& R_des, const Eigen::Matrix3d& R_curr);

// Inverse Kinematics velocity solver (least-squares with minimal-norm solution)
// INPUTS:
//   q_in      - 7x1 vector of current joint configuration
//   v_in      - 3x1 desired linear velocity in world frame; NaN components are unconstrained
//   omega_in  - 3x1 desired angular velocity in world frame; NaN components are unconstrained
// OUTPUT:
//   Returns dq (7x1) joint velocities minimizing least squares error; if underdetermined,
//   returns the minimal L2-norm solution. If all targets are unconstrained, returns zeros.
Eigen::Vector<double, 7> IK_velocity(const Eigen::Vector<double, 7>& q_in,
                                     const Eigen::Vector3d& v_in,
                                     const Eigen::Vector3d& omega_in);

#endif // KINEMATICS_H
