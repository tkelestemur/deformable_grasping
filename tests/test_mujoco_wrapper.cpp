#include <mujoco_wrapper.h>

int main(){
    MuJoCoWrapper mjw;

    while (true){
        mjw.run();
        mjw.render();
    }

    return 1;
}