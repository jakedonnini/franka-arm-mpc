#define _USE_MATH_DEFINES

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "kinematics.h"
#include "Optimizer.h"

// optimizer class for MPC
KinematicsCache Optimizer::cache; // define static cache

Eigen::Vector<double, 7> Optimizer::step(const Eigen::Vector<double, 7>& q_current, const Eigen::Matrix4d& T_target) {
    // Initialize
    cache.setConfiguration(q_current);
    Eigen::Matrix4d T0e = cache.T0e();
    double best_cost = cost(q_current, T_target, T0e);
    Eigen::Vector<double,7> q_best = q_current;

    // Simple local search in task space position (coarse heuristic)
    for (double step_x = -alpha; step_x <= alpha; step_x += alpha) {
        for (double step_y = -alpha; step_y <= alpha; step_y += alpha) {
            for (double step_z = -alpha; step_z <= alpha; step_z += alpha) {
                Eigen::Matrix4d T_waypoint = Eigen::Matrix4d::Identity();
                T_waypoint.block<3,1>(0, 3) = T0e.block<3,1>(0, 3); // copy current position
                T_waypoint.block<3,3>(0, 0) = T_target.block<3,3>(0, 0); // use target orientation
                T_waypoint(0,3) += step_x;
                T_waypoint(1,3) += step_y;
                T_waypoint(2,3) += step_z;

                // One IK step towards waypoint
                cache.setConfiguration(q_current);
                Eigen::Vector<double,7> dq = inverse_kinematics_step_optimized(cache, T_waypoint, 0.05, 0.1);
                Eigen::Vector<double,7> q_candidate = q_current + dq;

                // Evaluate candidate
                Eigen::Vector<double,7> q_copy = q_candidate;
                cache.setConfiguration(q_copy);
                cache.ensureFK();
                Eigen::Matrix4d T_candidate = cache.T0e();
                double c = cost(q_candidate, T_target, T_candidate);
                if (c < best_cost) {
                    best_cost = c;
                    q_best = q_candidate;
                }
            }
        }
    }
    return q_best;
}

// cost function to minimize distance to target
double Optimizer::cost(const Eigen::Vector<double, 7>& q, const Eigen::Matrix4d& T_target, const Eigen::Matrix4d& T0e) {
    // TODO add more rules to cost function for obstacle avoidance, joint limits, etc.
    double distance, angle;
    distance_and_angle(T0e, T_target, distance, angle);

    double angle_gain = 0.5; // weight for angle in cost function

    if (!has_colided(q)) {
        return 1e6; // invalid solution, return high cost
    }

    return distance * distance + angle_gain * angle * angle; // minimize squared distance and angle
}

bool Optimizer::has_colided(const Eigen::Vector<double, 7>& q) {
    // TODO implement collision checking

    for (int i = 0; i < 7; ++i) {
        if (q[i] < lower_limits[i] || q[i] > upper_limits[i]) {
            return true; // Out of bounds
        }
    }

    return false;
}

