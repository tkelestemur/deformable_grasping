#include "mujoco_wrapper.h"
//#include <boost/bind.hpp>

MuJoCoWrapper::MuJoCoWrapper(bool view) {

    view_flag = view;
    // activate software
    const char *mjKeyPath = std::getenv("MUJOCO_KEY");
    std::cout << "Key Path: " << mjKeyPath << std::endl;
    mj_activate(mjKeyPath);

    // load and compile model
    const char *modelPath = "../models/env.xml";
    std::cout << "Model Path: " << modelPath << std::endl;
    m = mj_loadXML(modelPath, NULL, NULL, 0);
    if (!m) mju_error("Model cannot be loaded");

    // make data
    d = mj_makeData(m);
    nv = m->nv;

    std::cout << "# of DoF: " << m->nv << " # of Actuators: " << m->nu
    << " # of Coordinates: " << m->nq << std::endl;

    if (view_flag) {
        mujoco_viewer::m = m;
        mujoco_viewer::d = d;
        initRendering();
    }

    eefBodyIdx = mj_name2id(m, mjOBJ_BODY, "tool0");
    wideFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "wide_finger_joint");
    narrowFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "narrow_finger_joint");
    num_control_steps = 10;
    num_arm_dof = 6;
    num_gripper_dof = 2;

    armCtrl.resize(num_arm_dof);
    gripperCtrl.resize(num_gripper_dof);
    mju_zero(d->userdata, 6);
}

MuJoCoWrapper::~MuJoCoWrapper() {
    //free visualization storage
    mjv_freeScene(&mujoco_viewer::scn);
    mjr_freeContext(&mujoco_viewer::con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

bool MuJoCoWrapper::initRendering() {

    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
        return false;
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1080, 900, "", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&mujoco_viewer::cam);
    mujoco_viewer::cam.distance = 5;
    mjv_defaultOption(&mujoco_viewer::opt);
    mujoco_viewer::opt.frame = mjFRAME_WORLD; // Show the world frame
    mujoco_viewer::opt.jointgroup[0] = 1;
    mujoco_viewer::opt.flags[mjVIS_CONTACTPOINT] = 1;
    mjv_defaultScene(&mujoco_viewer::scn);
    mjr_defaultContext(&mujoco_viewer::con);

    // data figure settings
    mjv_defaultFigure(&figdata);
    strcpy(figdata.title, "Wide-Finger Joint Torque");
    figdata.flg_extend = false;
//    figdata.flg_legend = true;
    figdata.figurergba[2] = 0.2f;
    figdata.figurergba[3] = 0.5f;
    figdata.gridsize[0] = 2;
    figdata.gridsize[1] = 5;

    figdata.range[0][0] = -100;
    figdata.range[0][1] = 0;
    figdata.range[1][0] = -4;
    figdata.range[1][1] = 4;

    figdata.linergb[0][0] = 1.0;
    figdata.linergb[0][1] = 0.0;
    figdata.linergb[0][2] = 0.0;


    strcpy(figdata.linename[0], "actuator torque");
    strcpy(figdata.linename[1], "external torque");
//    strcpy(figdata.linename[2], "inverse torque");
    figdata.linergb[1][0] = 0.0;
    figdata.linergb[1][1] = 1.0;
    figdata.linergb[1][2] = 0.0;
    figdata.linewidth[0] = 1.0;
    figdata.linewidth[1] = 1.0;

    for (int i = 0; i < mjMAXLINEPNT; i++) {
        figdata.linedata[0][2 * i] = (float) -i;
        figdata.linedata[1][2 * i] = (float) -i;
//        figdata.linedata[2][2*i] = (float)-i;
    }

    // create scene and context
    mjv_makeScene(m, &mujoco_viewer::scn, 2000);
    mjr_makeContext(m, &mujoco_viewer::con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, mujoco_viewer::keyboard);
    glfwSetCursorPosCallback(window, mujoco_viewer::mouseMove);
    glfwSetMouseButtonCallback(window, mujoco_viewer::mouseButton);
    glfwSetScrollCallback(window, mujoco_viewer::scroll);

    return true;
}

void MuJoCoWrapper::reset() {
    mj_resetData(m, d);
    double armQInit[6] = {0.0, -0.90, 0.90, 0.0, mjPI / 2, -mjPI / 2};
    double qPosGrasp[8] = {-0.18368017, -0.76765977, 1.60432432,
                           -0.9, 1.39290778, -1.57170523, 0.3, 0.3};
    for (int j = 0; j < 8; j++) {
        d->qpos[j] = qPosGrasp[j];
    }
//    d->qpos[mj_name2id(m, mjOBJ_JOINT, "narrow_finger_joint")] = 0.5;
//    d->qpos[mj_name2id(m, mjOBJ_JOINT, "wide_finger_joint")] = 0.5;
    mj_forward(m, d);
}

void MuJoCoWrapper::updateFigData() {
//    figdata.linepnt[0] = 100;
    // shift data
    int pnt = mjMIN(201, figdata.linepnt[0] + 1);
    int pnt_2 = mjMIN(201, figdata.linepnt[1] + 1);
//    int pnt_3 = mjMIN(201, figdata.linepnt[2]+1);
    for (int i = pnt - 1; i > 0; i--) {
        figdata.linedata[0][2 * i + 1] = figdata.linedata[0][2 * i - 1];
        figdata.linedata[1][2 * i + 1] = figdata.linedata[1][2 * i - 1];
//        figdata.linedata[2][2*i+1] = figdata.linedata[2][2*i-1];
    }

    // assign new
    figdata.linepnt[0] = pnt;
    figdata.linepnt[1] = pnt_2;
//    figdata.linepnt[2] = pnt_3;
    figdata.linedata[0][1] = (float) d->qfrc_actuator[wideFingerJntIdx];
    figdata.linedata[1][1] = (float) d->qfrc_passive[wideFingerJntIdx];
//    figdata.linedata[2][1] = (float)d->qfrc_inverse[wideFingerJntIdx];
}

void MuJoCoWrapper::render() {
    if (!view_flag || glfwWindowShouldClose(window)) {
        exit(1);
    }

    mj_step(m, d);
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    mjrRect sensor_viewport = {0, 0, 350, 350};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &mujoco_viewer::opt, NULL, &mujoco_viewer::cam, mjCAT_ALL, &mujoco_viewer::scn);
    mjr_render(viewport, &mujoco_viewer::scn, &mujoco_viewer::con);

    // Show simulation status
    char *status = new char[100];
    sprintf(status, "%s", !mujoco_viewer::paused ? "Running" : "Stopped");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "Status: ", status, &mujoco_viewer::con);

    // Show data figure
    updateFigData();
    mjr_figure(sensor_viewport, &figdata, &mujoco_viewer::con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

}

void MuJoCoWrapper::setControl(const Eigen::VectorXd &arm, const Eigen::VectorXd &gripper) {
    armCtrl = arm;
    gripperCtrl = gripper;
}


void MuJoCoWrapper::run() {
    for (int i = 0; i < num_control_steps; i++) {
        mj_step1(m, d);
        // arm control
        for (int i = 0; i < num_arm_dof; i++) {
            // gravity compensation for arm
            d->qfrc_applied[i] = d->qfrc_bias[i];
            // velocity commands
            d->ctrl[i] = armCtrl[i];
        }
        // gripper control
        for (int i = num_arm_dof; i < num_arm_dof + num_gripper_dof; i++) {
            d->qfrc_applied[i] = d->qfrc_bias[i];
            d->ctrl[i] = gripperCtrl[i-6];
        }

        mj_step2(m, d);
    }
}

double MuJoCoWrapper::getSimTime() {
    return d->time;
}

void MuJoCoWrapper::printData() {
    std::cout << "object mass: " << m->body_mass[12] << std::endl;
}

Eigen::VectorXd MuJoCoWrapper::getEEFCmd() {
    Eigen::VectorXd eefCmd(num_arm_dof);
    for (int i = 0; i < num_arm_dof; ++i) {
        eefCmd(i) = d->userdata[i];
    }
    return eefCmd;
}

Eigen::VectorXd MuJoCoWrapper::getGripperCmd() {
    Eigen::VectorXd gripperCmd(2);
    gripperCmd(0) = d->userdata[6];
    gripperCmd(1) = d->userdata[7];
    return gripperCmd;
}

Eigen::VectorXd MuJoCoWrapper::eefVelToQDot(const Eigen::VectorXd &eefVelCmd) {

    mjtNum *eefJacPosMJ = mj_stackAlloc(d, 3 * nv);
    mjtNum *eefJacRotMJ = mj_stackAlloc(d, 3 * nv);
    mjtNum *armQDotCmdMJ = mj_stackAlloc(d, nv);

    mj_jacBody(m, d, eefJacPosMJ, eefJacRotMJ, eefBodyIdx);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eefJacPos, eefJacRot;
    eefJacPos = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacPosMJ, 3, nv);
    eefJacRot = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacRotMJ, 3, nv);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacFull, jacFullInv;

    jacFull << eefJacPos.block<3,6>(0,0), eefJacRot.block<3,6>(0,0);

    jacFullInv = jacFull.inverse();
    mju_mulMatVec(armQDotCmdMJ, jacFullInv.data(), eefVelCmd.data(), 6, 6);

    Eigen::VectorXd armQDotCmd = Eigen::Map<Eigen::VectorXd>(armQDotCmdMJ, 6);
    return armQDotCmd;
//    armQDotCmd = Eigen::VectorXd(armQDotCmdMJ);
}

void MuJoCoWrapper::setObjectMass(const double mass) {
    m->body_mass[12] = mass;
}

void MuJoCoWrapper::setObjectStiffness(const double k) {
//    for (int i = 14; i < ; ++i) {
//
//    }
}