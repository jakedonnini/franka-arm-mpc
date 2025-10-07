#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include "Optimizer.h"
#include <GLFW/glfw3.h>

// for delay
#include <thread>
#include <chrono>

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
    // MuJoCo may have extra base joints (e.g., floating base), so apply offset
    const int base_offset = 0; // Set to 1 or 2 if your model has extra base joints
    char error_msg[1000] = "Could not load XML";
    mjModel* m = mj_loadXML(MODEL_XML, NULL, error_msg, sizeof(error_msg));
    if (!m) { std::cerr << "Failed to load model: " << error_msg << std::endl; return 1; }
    mjData* d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); std::cerr << "Failed to allocate data\n"; return 1; }

    if (!glfwInit()) { std::cerr << "GLFW init failed\n"; return 1; }
    window = glfwCreateWindow(1200, 900, "MuJoCo Panda", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // create the optimizer class
    Optimizer optimizer;

    // camera
    mjv_defaultCamera(&cam);
    cam.distance = 4.0;   // distance from target
    cam.azimuth = 45.0;   // azimuth angle
    cam.elevation = -30.0; // elevation angle

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

    std::cout << "joint_ids.size(): " << joint_ids.size() << std::endl;
    std::cout << "qpos_addr.size(): " << qpos_addr.size() << std::endl;
    std::cout << "dof_addr.size(): " << dof_addr.size() << std::endl;

    // Check we found all 7 joints
    const int controlled_dofs = 7;
    if (controlled_dofs != (int)dof_addr.size()) {
        std::cerr << "Warning: Found " << dof_addr.size() << " DOFs, but expected " << controlled_dofs << ".\n";
    }

    std::cout << "Found " << controlled_dofs << " Panda joints to control.\n";

    // Initialize target joint positions to zero (home position)
    // Set initial joint positions to home seed: [0, 0, 0, -pi/2, 0, pi/2, pi/4]
    Eigen::Vector<double, controlled_dofs> q_target;
    q_target << 0.0, 0.0, 0.0, -M_PI/2, 0.0, M_PI/2, M_PI/4;
    Eigen::Vector<double, controlled_dofs> q_current = q_target;


    const double Kp = 0.01;
    // Removed Kd as it was unused - add back if needed for damping control

    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;
    // jacobian
    Eigen::Matrix<double, 6, controlled_dofs> J;
    Eigen::Matrix<double, 6, controlled_dofs> J_mujoco;
    // target end-effector position (4x4 homogeneous transform)
    Eigen::Matrix4d T_target = Eigen::Matrix4d::Identity();
    Eigen::Vector<double, controlled_dofs> u;
    Eigen::Vector<double, controlled_dofs> dq;



    while (!glfwWindowShouldClose(window)) {
        for (int i=0;i<m->nv;i++) d->qfrc_applied[i] = 0.0;
        // check the current joint angles
        for (int i = 0; i < controlled_dofs; ++i) {
            q_current[i] = d->qpos[qpos_addr[i + base_offset]];
        }

        // Debug: print current joint angles
        std::cout << "q_target: " << q_target.transpose() << std::endl;
        std::cout << "q_current: " << q_current.transpose() << std::endl;
        
        // move end-effector to target
        T_target.block<3,1>(0,3) = Eigen::Vector3d(-0.2, 0.0, 0.5);
        // rotation
        T_target.block<3,3>(0,0) << 
            0.0, -1.0, 0.0,   // row 1
            -1.0, 0.0, 0.0,   // row 2
            0.0, 0.0, -1.0;   // row 3

        // std::cout << "T_target:\n" << T_target << std::endl;

        // Eigen::Vector<double, 7>  dq_step = inverse_kinematics_step(q_current, T_target, 0.1, 0.05);
        static KinematicsCache cache;
        cache.setConfiguration(q_current);
        Eigen::Vector<double, 7> dq_step = inverse_kinematics_step_optimized(cache, T_target, 0.1, 0.1);

        std::cout << "dq_step: " << dq_step.transpose() << std::endl;
        q_target += dq_step; // scale step size

        // check if we are close enough to target
        if (is_valid_solution(q_current, T_target, 0.2, 0.2)) {
            std::cout << "Reached target within tolerance.\n";
            // Optionally break or set a new target
            break;
        }

        for (int i = 0; i < controlled_dofs; ++i) {
            int idx = i + base_offset;
            // check to make sure we don't exceed joint limits
            if (q_target[i] < m->jnt_range[joint_ids[idx]*2]) {
                q_target[i] = m->jnt_range[joint_ids[idx]*2];
            } else if (q_target[i] > m->jnt_range[joint_ids[idx]*2 + 1]) {
                q_target[i] = m->jnt_range[joint_ids[idx]*2 + 1];
            }
            d->qpos[qpos_addr[idx]] = q_target[i];
        }

        mj_step(m, d);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(mjrRect{0,0,1200,900}, &scn, &con);


        // ---- get end-effector position from simulation ----
        int gripper_site = mj_name2id(m, mjOBJ_SITE, "gripper");
        double x = d->site_xpos[3*gripper_site + 0];
        double y = d->site_xpos[3*gripper_site + 1];
        double z = d->site_xpos[3*gripper_site + 2];
        std::cout << "End-effector: " << x << "," << y << "," << z << std::endl;
        // get gripper rotation matrix
        Eigen::Matrix3d gripper_rot;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                gripper_rot(i, j) = d->site_xmat[gripper_site * 9 + 3 * i + j];
        std::cout << "Gripper rotation matrix:\n" << gripper_rot << std::endl;

        glfwSwapBuffers(window);
        glfwPollEvents();

        // sleep to control simulation speed
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
