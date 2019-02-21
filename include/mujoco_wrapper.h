#ifndef MUJOCO_WRAPPER_H
#define MUJOCO_WRAPPER_H

#include <iostream>

// MuJoCo headers
#include "mujoco.h"
#include "glfw3.h"


class MuJoCoWrapper {

public:
    MuJoCoWrapper();
    ~MuJoCoWrapper();

    void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
    static void mouseButton(GLFWwindow *window, int button, int act, int mods);
    void mouseMove(GLFWwindow *window, double xpos, double ypos);
    void scroll(GLFWwindow *window, double xoffset, double yoffset);
    void updateFigData();
    void render();
    void reset();
    bool initMuJoCo();
    bool initRendering();


private:

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
    mjtNum eefCmd[6];
    mjtNum gripperCmd[2];
    mjtNum transEEFVel;
};


#endif //MUJOCO_WRAPPER_H
