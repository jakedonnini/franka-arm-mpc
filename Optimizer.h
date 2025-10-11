#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "kinematics.h"

// Optimizer class for MPC
class Optimizer {
private:
    int max_iters = 100;
    int predict_horizon = 10;
    int control_horizon = 5;
    double alpha = 0.3;  // step size
    double tol = 1e-4; 
    const double lower_limits[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    const double upper_limits[7] = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};
    static KinematicsCache cache; // create a static cache for IK optimization to be used across calls

public:
    // Constructor & Destructor
    Optimizer() = default;
    ~Optimizer() = default;

    // Perform one optimization step
    Eigen::Vector<double, 7> step(
        const Eigen::Vector<double, 7>& q_current,
        const Eigen::Matrix4d& T_target
    );

    // Cost function to minimize distance to target
    double cost(const Eigen::Matrix4d& T_target, const Eigen::Matrix4d& T0e);

    // Check for collisions (stub function, implement as needed)
    bool has_colided(const Eigen::Vector<double, 7>& q);
};

#endif // OPTIMIZER_H
