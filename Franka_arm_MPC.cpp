#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include "Optimizer.h"
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
const double M_PI = 3.14159265358979323846;

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

    // Check we found all 7 joints
    const int controlled_dofs = 7;
    if (controlled_dofs != (int)dof_addr.size()) {
        std::cerr << "Warning: Found " << dof_addr.size() << " DOFs, but expected " << controlled_dofs << ".\n";
    }

    std::cout << "Found " << controlled_dofs << " Panda joints to control.\n";

    // Initialize target joint positions to zero (home position)
    Eigen::Vector<double, controlled_dofs> q_target = Eigen::Vector<double, controlled_dofs>::Zero(); // calculated current joint angles
    Eigen::Vector<double, controlled_dofs> q_current = Eigen::Vector<double, controlled_dofs>::Zero(); // actual current joint angles


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
            q_current[i] = d->qpos[qpos_addr[i]];
        }

        // Debug: print current joint angles
        std::cout << "q_target: " << q_target.transpose() << std::endl;
        
        // move end-effector to target
        T_target.block<3,1>(0,3) = Eigen::Vector3d(0.3, 0.0, 0.5);
        // rotation
        T_target.block<3,3>(0,0) << 
            1.0, 0.0, 0.0,   // row 1
            0.0, 1.0, 0.0,   // row 2
            0.0, 0.0, 1.0;   // row 3

        Eigen::Vector<double, 7> q_next = optimizer.step(q_current, T_target);

        // Compute Euclidean position difference (3D)
        Eigen::Vector3d dp = diff_to_target(T0e, T_target);
        std::cout << "T0e:\n" << T0e << std::endl;
        std::cout << "T_target:\n" << T_target << std::endl;
        std::cout << "dp: " << dp.transpose() << std::endl;

        // get the change in joint angles to move towards target
        Eigen::Vector3d omega = calcAngDiff(T_target.block<3,3>(0,0), T0e.block<3,3>(0,0));
        std::cout << "omega (orientation error): " << omega.transpose() << std::endl;
        
        q_target = q_next;

        for (int i = 0; i < controlled_dofs; ++i) {
            // check to make sure we don't exceed joint limits
            if (q_target[i] < m->jnt_range[joint_ids[i]*2]) {
                q_target[i] = m->jnt_range[joint_ids[i]*2];
            } else if (q_target[i] > m->jnt_range[joint_ids[i]*2 + 1]) {
                q_target[i] = m->jnt_range[joint_ids[i]*2 + 1];
            }
            d->qpos[qpos_addr[i]] = q_target[i];
        }

        mj_step(m, d);

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
