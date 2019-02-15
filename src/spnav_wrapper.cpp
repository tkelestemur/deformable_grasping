#include <ur_mujoco/spnav_wrapper.h>

namespace spnav_wrapper{
    spnav_event sev;
    bool open_spnav(){
        if(spnav_open() == -1)
            return false;
        else
            return true;
    }

    void close_spnav(){
        spnav_close();
    }

    motion get_event(){
        int event = spnav_poll_event(&sev);
        motion m;
        m.pos.resize(3);
        m.rot.resize(3);
//        m.pos.push_back(0);
//        m.pos.push_back(0);
//        m.pos.push_back(0);
//
//        m.rot.push_back(0);
//        m.rot.push_back(0);
//        m.rot.push_back(0);


        if(event == SPNAV_EVENT_MOTION){

            m.pos[0] = sev.motion.z;
            m.pos[1] = -sev.motion.x;
            m.pos[2] = sev.motion.y;

            m.rot[0] = sev.motion.rx;
            m.rot[1] = sev.motion.ry;
            m.rot[2] = sev.motion.rz;
            m.received_event = true;

        }
        else if(event == 0){
            m.received_event = false;
        }
//        else if(event == SPNAV_EVENT_BUTTON){
//            m.buttons[sev.button.bnum] = sev.button.press;
//        }
        return m;
    }

}