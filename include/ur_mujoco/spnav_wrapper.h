#include <iostream>
#include <vector>
#include <spnav.h>

#ifndef UR_MUJOCO_SPNAV_WRAPPER_H
#define UR_MUJOCO_SPNAV_WRAPPER_H



namespace spnav_wrapper {
    struct motion {
        std::vector<double> pos;
        std::vector<double> rot;
        std::vector<int> buttons;
    };

    bool open_spnav();
    motion get_event();
}

#endif //UR_MUJOCO_SPNAV_WRAPPER_H
