#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include <GLFW/glfw3.h>

// Suppress warnings from external libraries
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4127) // conditional expression is constant (Eigen)
#endif

mjvCamera cam; 
mjvOption opt; 
mjvScene scn; 
mjrContext con; 
GLFWwindow* window;

const char* MODEL_XML = "C:/Users/jaked/Documents/Physics_Sim/mujoco_menagerie-main/mujoco_menagerie-main/franka_emika_panda/mjx_panda.xml";

int main() {
    char error_msg[1000] = "Could not load XML";
    mjModel* m = mj_loadXML(MODEL_XML, NULL, error_msg, sizeof(error_msg));
    if (!m) { std::cerr << "Failed to load model: " << error_msg << std::endl; return 1; }
    mjData* d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); std::cerr << "Failed to allocate data\n"; return 1; }

    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 1; }
    window = glfwCreateWindow(1200, 900, "MuJoCo Panda", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // camera
    mjv_defaultCamera(&cam);
    // Get site id for end-effector ("gripper")
    int gripper_site_id = mj_name2id(m, mjOBJ_SITE, "gripper");
    if (gripper_site_id < 0) {
        std::cerr << "Warning: gripper site not found!" << std::endl;
    }



    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    mj_resetData(m, d);

    std::vector<int> joint_ids, qpos_addr, dof_addr;
    for (int i = 1; i <= 7; ++i) {
        int jid = mj_name2id(m, mjOBJ_JOINT, ("joint" + std::to_string(i)).c_str());
        if (jid >= 0) {
            joint_ids.push_back(jid);
            qpos_addr.push_back(m->jnt_qposadr[jid]);
            dof_addr.push_back(m->jnt_dofadr[jid]);
        }
    }

    // Check we found all 7 joints
    const int controlled_dofs = 7;
    if (controlled_dofs != (int)dof_addr.size()) {
        std::cerr << "Warning: Found " << dof_addr.size() << " DOFs, but expected " << controlled_dofs << ".\n";
    }

    std::cout << "Found " << controlled_dofs << " Panda joints to control.\n";

    // Initialize target joint positions to zero (home position)
    Eigen::Vector<double, controlled_dofs> q_target = Eigen::Vector<double, controlled_dofs>::Zero();

    const double Kp = 0.01;
    // Removed Kd as it was unused - add back if needed for damping control

    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    // jacobian
    Eigen::Matrix<double, 6, controlled_dofs> J;
    Eigen::Matrix<double, 6, controlled_dofs> J_mujoco;
    Eigen::Matrix<double, controlled_dofs, 6> J_pseudoInv;  // Pseudoinverse is 7×6
    // target end-effector position (4x4 homogeneous transform)
    Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
    Eigen::Vector<double, controlled_dofs> u;


    while (!glfwWindowShouldClose(window)) {
        for (int i=0;i<m->nv;i++) d->qfrc_applied[i] = 0.0;

        // Debug: print current joint angles
        std::cout << "q_target: " << q_target.transpose() << std::endl;
        
        // use old custom functions to compute FK and Jacobian
        // Create a mutable copy for forwardKinematics (function may modify it)
        Eigen::Vector<double, controlled_dofs> q_current = q_target;
        forwardKinematics(q_current, /*out*/ jointPositions, /*out*/ T0e, /*out*/ T_list);
        computeJacobian(q_current, T_list, J);

        // std::cout << "Jacobian:\n" << J << std::endl;

        // compute jacobian with mujoco (using end-effector site)
        if (gripper_site_id >= 0) {
            std::vector<double> jacp(3 * m->nv);  // translational jacobian
            std::vector<double> jacr(3 * m->nv);  // rotational jacobian
            mj_jacSite(m, d, jacp.data(), jacr.data(), gripper_site_id);
            
            // Copy relevant parts to our Jacobian matrix (first 7 DOFs)
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < std::min(controlled_dofs, 7); ++j) {
                    J_mujoco(i, j) = jacp[i * m->nv + dof_addr[j]];
                    J_mujoco(i + 3, j) = jacr[i * m->nv + dof_addr[j]];
                }
            }
            // std::cout << "Mujoco Jacobian:\n" << J_mujoco << std::endl;
            // std::cout << "Difference in Jacobians:\n" << J - J_mujoco << std::endl;
        } else {
            std::cout << "Gripper site not found, skipping MuJoCo Jacobian comparison\n";
        }

        // move end-effector to target
        T_target.block<3,1>(0,3) = Eigen::Vector3d(0.5, 0.5, 0.5);
        // compute inverse kinematics
        // find the difference in joint position from current to target
        Eigen::VectorXd dq(controlled_dofs);
        diff_to_target(T0e, T_target, /*inout*/ dq);
        std::cout << "dq: " << dq.transpose() << std::endl;
        std::cout << "T0e:\n" << T0e << std::endl;
        std::cout << "T_target:\n" << T_target << std::endl;

        // Debug: Check Jacobian
        std::cout << "J matrix:\n" << J << std::endl;
        
        // Compute pseudoinverse using SVD (more robust than direct inverse)
        auto svd = J.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        // Check for singularities
        double tolerance = 1e-6;
        auto singularValues = svd.singularValues();
        std::cout << "Singular values: " << singularValues.transpose() << std::endl;
        
        // Compute pseudoinverse with regularization
        J_pseudoInv = svd.matrixV() * (singularValues.array() > tolerance).select(singularValues.array().inverse(), 0.0).matrix().asDiagonal() * svd.matrixU().transpose();

        // apply P control to reach target joint positions
        u = Kp * J_pseudoInv * dq;
        std::cout << "u: " << u.transpose() << std::endl;
        
        // update target joint positions to mujoco
        // q_target += u;

        for (int i = 0; i < controlled_dofs; ++i) {
            d->qpos[qpos_addr[i]] = q_target[i];
        }

        mj_step(m, d);

        // --- camera follows end-effector ---
        if (gripper_site_id >= 0) {
            // gripper_site_id corresponds to d->site_xpos
            cam.lookat[0] = d->site_xpos[3*gripper_site_id + 0];
            cam.lookat[1] = d->site_xpos[3*gripper_site_id + 1];
            cam.lookat[2] = d->site_xpos[3*gripper_site_id + 2];

            // Optional: keep a fixed distance behind the end-effector
            cam.distance = 2.0;   // distance from target
            cam.azimuth += 0.2;   // rotate slowly around gripper
        }

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(mjrRect{0,0,1200,900}, &scn, &con);

        int gripper_site = mj_name2id(m, mjOBJ_SITE, "gripper");
        double x = d->site_xpos[3*gripper_site + 0];
        double y = d->site_xpos[3*gripper_site + 1];
        double z = d->site_xpos[3*gripper_site + 2];
        std::cout << "End-effector: " << x << "," << y << "," << z << std::endl;

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(d);
    mj_deleteModel(m);

    std::cout << "Simulation closed.\n";
    return 0;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
