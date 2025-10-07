#define _USE_MATH_DEFINES

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "kinematics.h"
#include "Optimizer.h"

// optimizer class for MPC

// create optimizer function for forward kinematics
Eigen::Vector<double, 7> Optimizer::step(const Eigen::Vector<double, 7>& q_current, const Eigen::Matrix4d& T_target) {
    // use the current cost so new cost is always lower
    double min_cost = cost(q_current, T_target);
    Eigen::Vector<double, 7> q_best = q_current;

    // iterate to minimize distance to target
    for (double j1 = -1; j1 <= 1; j1 += alpha) {
        std::cout << "j1: " << j1 << std::endl;
        for (double j2 = -1; j2 <= 1; j2 += alpha) {
            std::cout << "j2: " << j2 << std::endl;
            for (double j3 = -1; j3 <= 1; j3 += alpha) {
                for (double j4 = -1; j4 <= 1; j4 += alpha) {
                    for (double j5 = -1; j5 <= 1; j5 += alpha) {
                        for (double j6 = -1; j6 <= 1; j6 += alpha) {
                            for (double j7 = -1; j7 <= 1; j7 += alpha) {
                                Eigen::Vector<double, 7> q_new = q_current;
                                q_new[0] += j1 * alpha;
                                q_new[1] += j2 * alpha;
                                q_new[2] += j3 * alpha;
                                q_new[3] += j4 * alpha;
                                q_new[4] += j5 * alpha;
                                q_new[5] += j6 * alpha;
                                q_new[6] += j7 * alpha;

                                double c_new = cost(q_new, T_target);

                                if (c_new < min_cost) {
                                    min_cost = c_new;
                                    q_best = q_new; // update current to new
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return q_best; // return the best found configuration
}

// cost function to minimize distance to target
double Optimizer::cost(const Eigen::Vector<double, 7>& q, const Eigen::Matrix4d& T_target) {
    // TODO add more rules to cost function for obstacle avoidance, joint limits, etc.
    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    Eigen::Vector<double, 7> q_copy = q; // forwardKinematics requires non-const reference
    T_list = forwardKinematics(q_copy, jointPositions, T0e);
    Eigen::Vector3d dp = diff_to_target(T0e, T_target);
    return dp.squaredNorm(); // minimize squared distance
}

