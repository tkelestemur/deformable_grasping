#include "mujoco_wrapper.h"
#include <boost/bind.hpp>

MuJoCoWrapper::MuJoCoWrapper(){
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

    std::cout << "# of DoF: " << m->nv << " # of Actuators: " << m->nu << " # of Coordinates: " << m->nq << std::endl;

    std::cout << "Joint names: " << std::endl;
    for (int i = 0; i < 6; ++i) {
        std::cout << mj_id2name(m, mjOBJ_JOINT, i) << std::endl;
    }
}

MuJoCoWrapper::~MuJoCoWrapper(){
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

bool MuJoCoWrapper::initRendering(){

    // init GLFW
    if (!glfwInit()){
        mju_error("Could not initialize GLFW");
        return false;
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1400, 1000, "", NULL, NULL);
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

    for(int i=0; i<mjMAXLINEPNT; i++ ){
        figdata.linedata[0][2*i] = (float)-i;
        figdata.linedata[1][2*i] = (float)-i;
//        figdata.linedata[2][2*i] = (float)-i;
    }

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

}

void MuJoCoWrapper::reset(){
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
void MuJoCoWrapper::keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {

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

void MuJoCoWrapper::mouseButton(GLFWwindow *window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
void MuJoCoWrapper::mouseMove(GLFWwindow *window, double xpos, double ypos){
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
void MuJoCoWrapper::scroll(GLFWwindow *window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
void MuJoCoWrapper::updateFigData(){

}
void MuJoCoWrapper::render(){
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
//    update_fig_data();
    mjr_figure(sensor_viewport, &figdata, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}