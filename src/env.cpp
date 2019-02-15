#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

// MuJoCo headers
#include "mujoco.h"
#include "glfw3.h"


// MuJoCo data structures
mjModel *m = NULL;                  // MuJoCo model
mjData *d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjvFigure figdata;
GLFWwindow *window = NULL;


// mouse interaction
bool paused = false;
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// eef control flags
// up, down, left, right, forward, backward;
mjtNum eefCmd[6] = {0.0};
mjtNum gripperCmd[2] = {0.0};
mjtNum transEEFVel = 0.25;



void reset(void) {
    mj_resetData(m, d);
    double armQInit[6] = {0.0, -0.90, 0.90, 0.0, mjPI / 2, -mjPI / 2};
    for (int j = 0; j < 6; j++) {
        d->qpos[j] = armQInit[j];
    }
//    d->qpos[mj_name2id(m, mjOBJ_JOINT, "narrow_finger_joint")] = 0.5;
//    d->qpos[mj_name2id(m, mjOBJ_JOINT, "wide_finger_joint")] = 0.5;
    mj_forward(m, d);
}

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {

    if (act == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_BACKSPACE:
                reset();
                break;
            case GLFW_KEY_SPACE:
                paused = !paused;
                break;
            case GLFW_KEY_PAGE_UP:
                eefCmd[2] = transEEFVel;
                break;
            case GLFW_KEY_PAGE_DOWN:
                eefCmd[2] = -transEEFVel;
                break;
            case GLFW_KEY_UP:
                eefCmd[0] = transEEFVel;
                break;
            case GLFW_KEY_DOWN:
                eefCmd[0] = -transEEFVel;
                break;
            case GLFW_KEY_LEFT:
                eefCmd[1] = transEEFVel;
                break;
            case GLFW_KEY_RIGHT:
                eefCmd[1] = -transEEFVel;
                break;
            case GLFW_KEY_C:
                gripperCmd[0] = -0.1;
                gripperCmd[1] = -0.1;
                break;
            case GLFW_KEY_O:
                gripperCmd[0] = 0.1;
                gripperCmd[1] = 0.1;
                break;
            default:
                mju_warning("Unkown keyboard command!");
                break;
        }
    } else if (act == GLFW_RELEASE) {
        mju_zero(eefCmd, 6);
        mju_zero(gripperCmd, 2);
    }
}


// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void init_mujoco(void) {
    // activate software
    const char *mjKeyPath = std::getenv("MUJOCO_KEY");
    std::cout << "Key Path: " << mjKeyPath << std::endl;
    mj_activate(mjKeyPath);

    // load and compile model
    const char *modelPath = "/home/tarik/projects/ur_mujoco/models/env.xml";
    std::cout << "Model Path: " << modelPath << std::endl;
    m = mj_loadXML(modelPath, NULL, NULL, 0);
    if (!m) mju_error("Model cannot be loaded");

    // make data
    d = mj_makeData(m);

    std::cout << "# of DoF: " << m->nv << " # of Actuators: " << m->nu << " # of Coordinates: " << m->nq << std::endl;

    std::cout << "Joint names: " << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << mj_id2name(m, mjOBJ_JOINT, i) << std::endl;
    }

}

void init_rendering(void) {

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(960, 1080, "", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    cam.distance = 5;
    mjv_defaultOption(&opt);
    opt.frame = mjFRAME_WORLD; // Show the world frame
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // data figure settings
    mjv_defaultFigure(&figdata);
    strcpy(figdata.title, "Finger Joint Torque");
    figdata.flg_extend = false;
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

    figdata.linergb[1][0] = 0.0;
    figdata.linergb[1][1] = 1.0;
    figdata.linergb[1][2] = 0.0;
    figdata.linewidth[0] = 1.0;
    figdata.linewidth[1] = 1.5;

    for(int i=0; i<mjMAXLINEPNT; i++ ){
        figdata.linedata[0][2*i] = (float)-i;
        figdata.linedata[1][2*i] = (float)-i;
    }


    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

void update_fig_data(){
//    figdata.linepnt[0] = 100;
    int wideFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "wide_finger_joint");
    int narrowFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "narrow_finger_joint");

    // shift data
    int pnt = mjMIN(201, figdata.linepnt[0]+1);
    int pnt_2 = mjMIN(201, figdata.linepnt[1]+1);
    for(int i=pnt-1; i>0; i-- ){
        figdata.linedata[0][2*i+1] = figdata.linedata[0][2*i-1];
        figdata.linedata[1][2*i+1] = figdata.linedata[1][2*i-1];
    }

    // assign new
    figdata.linepnt[0] = pnt;
    figdata.linepnt[1] = pnt;
    figdata.linedata[0][1] = (float)d->qfrc_actuator[wideFingerJntIdx];
    figdata.linedata[1][1] = (float)d->qfrc_passive[wideFingerJntIdx];
}

void render() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    mjrRect sensor_viewport = {0, 0, 350, 350};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // Show simulation status
    char *status = new char[100];
    sprintf(status, "%s", !paused ? "Running" : "Stopped");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "Status: ", status, &con);

    // Show data figure
    update_fig_data();
    mjr_figure(sensor_viewport, &figdata, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void kill(void) {
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

}

int main(int argc, const char **argv) {

    init_mujoco();

    const int nv = m->nv;

    init_rendering();

    int eefBodyIdx = mj_name2id(m, mjOBJ_BODY, "tool0");
    int wideFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "wide_finger_joint");
    int narrowFingerJntIdx = mj_name2id(m, mjOBJ_JOINT, "narrow_finger_joint");

    reset();

    mjtNum *eefJacPosMJ = mj_stackAlloc(d, 3 * nv);
    mjtNum *eefJacRotMJ = mj_stackAlloc(d, 3 * nv);
    mjtNum *qDotCmd = mj_stackAlloc(d, nv);
    mju_zero(qDotCmd, nv);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eefJacPos, eefJacRot;
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> eefJacFull, eefJacFullInv;
    while (!glfwWindowShouldClose(window)) {

        mj_jacBody(m, d, eefJacPosMJ, eefJacRotMJ, eefBodyIdx);
        eefJacPos = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacPosMJ, 3, nv);
        eefJacRot = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(eefJacRotMJ, 3, nv);
        eefJacFull << eefJacPos.block<3,6>(0,0), eefJacRot.block<3,6>(0,0);
        eefJacFullInv = eefJacFull.inverse();
        mju_mulMatVec(qDotCmd, eefJacFullInv.data(), eefCmd, 6, 6);

        double simTime = d->time;
        // control loop
        if (!paused) {
            for (int i = 0; i < 10; i++) {
                mj_step1(m, d);
                // arm control
                for (int i = 0; i < 6; i++) {
                    // gravity compensation for arm
                    d->qfrc_applied[i] = d->qfrc_bias[i];
                    // velocity commands
                    d->ctrl[i] = qDotCmd[i];
                }
                // gripper control
                for (int i = 6; i < 8; i++) {
                    d->qfrc_applied[i] = d->qfrc_bias[i];
                    d->ctrl[i] = gripperCmd[i-6];
                }

                mj_step2(m, d);
            }
        }
//        std::cout << "Gripper joint torques: " << d->qfrc_actuator[wideFingerJntIdx]
//        << " " << d->qfrc_actuator[narrowFingerJntIdx] << std::endl;

        render();
    }
    kill();
    return 1;
}
