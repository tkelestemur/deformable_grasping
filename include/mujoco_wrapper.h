#ifndef MUJOCO_WRAPPER_H
#define MUJOCO_WRAPPER_H

#include <iostream>
#include <cstring>

// MuJoCo headers
#include "mujoco.h"
#include "glfw3.h"

namespace mujoco_viewer {
    // mouse interaction
    static bool paused;
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;

    static mjModel *m;                  // MuJoCo model
    static mjData *d;                   // MuJoCo data
    static mjvCamera cam;                      // abstract camera
    static mjvOption opt;                      // visualization options
    static mjvScene scn;                       // abstract scene
    static mjrContext con;

    // eef control flags
// up, down, left, right, forward, backward;
    static mjtNum eefCmd[6];
    static mjtNum gripperCmd[2];
    static mjtNum transEEFVel;

    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {

        if (act == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_BACKSPACE:
//                reset();
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

    static void mouseButton(GLFWwindow *window, int button, int act, int mods) {
        // update button state
        button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }

    static void mouseMove(GLFWwindow *window, double xpos, double ypos) {
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

    static void scroll(GLFWwindow *window, double xoffset, double yoffset) {
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
    }
}


class MuJoCoWrapper {

public:
    MuJoCoWrapper(bool view = true);

    ~MuJoCoWrapper();

    bool initRendering();

    void updateFigData();

    void render();

    void reset();

    void run();


private:

    // MuJoCo data structures
    mjModel *m = NULL;                  // MuJoCo model
    mjData *d = NULL;                   // MuJoCo data
    // custom GPU context
    mjvFigure figdata;
    GLFWwindow *window = NULL;
    bool view_flag = true;
    int num_control_steps;
    int num_arm_dof, num_gripper_dof;
};


#endif //MUJOCO_WRAPPER_H
