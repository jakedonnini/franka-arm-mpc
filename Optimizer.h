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
    double alpha = 0.1;  // step size
    double tol = 1e-4;

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
    double cost(
        const Eigen::Vector<double, 7>& q,
        const Eigen::Matrix4d& T_target
    );
};

#endif // OPTIMIZER_H
