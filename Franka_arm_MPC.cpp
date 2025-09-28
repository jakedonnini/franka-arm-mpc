#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include "kinematics.h"
#include <GLFW/glfw3.h>

mjvCamera cam; 
mjvOption opt; 
mjvScene scn; 
mjrContext con; 
GLFWwindow* window;

const char* MODEL_XML = "C:/Users/jaked/Documents/Physics_Sim/mujoco_menagerie-main/mujoco_menagerie-main/franka_emika_panda/mjx_panda.xml";

int main() {
    char err[1000] = "Could not load XML";
    mjModel* m = mj_loadXML(MODEL_XML, NULL, err, sizeof(err));
    if (!m) { std::cerr << "Failed to load model: " << err << std::endl; return 1; }
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
    int controlled_dofs = (int)dof_addr.size();
    std::cout << "Found " << controlled_dofs << " Panda joints to control.\n";

    std::vector<double> q_target(controlled_dofs, 0.0);
    if (controlled_dofs >= 1) q_target[0] = 0.4;

    const double Kp = 200.0;
    const double Kd = 2.0 * std::sqrt(Kp);

    Eigen::Matrix<double, 8, 3> jointPositions;
    Eigen::Matrix4d T0e;
    std::vector<Eigen::Matrix4d> T_list;

    while (!glfwWindowShouldClose(window)) {
        for (int i=0;i<m->nv;i++) d->qfrc_applied[i] = 0.0;
        for (int i = 0; i < controlled_dofs; ++i) {
            int qadr = qpos_addr[i];
            int dadr = dof_addr[i];
            double q = d->qpos[qadr];
            double qdot = d->qvel[dadr];
            double err = q_target[i] - q;
            double tau = Kp*err - Kd*qdot;
            d->qfrc_applied[dadr] += tau;
        }

        forwardKinematics(q_target, /*out*/ jointPositions, /*out*/ T0e, /*out*/ T_list);
        // print T0e
        //std::cout << "T0e:\n" << T0e << std::endl;
        // print end-effector position
        std::cout << "End-effector Position:\n" << T0e.block<3,1>(0,3) << std::endl;

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
