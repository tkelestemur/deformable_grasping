#include <mujoco_wrapper.h>

int main(){
    MuJoCoWrapper mjw;
    mjw.reset();
    Eigen::VectorXd armQDotCmd, gripperCmd;
    while (true){
        armQDotCmd = mjw.eefVelToQDot(mjw.getEEFCmd());
        gripperCmd = mjw.getGripperCmd();
        mjw.setControl(armQDotCmd, gripperCmd);
        mjw.run();
        mjw.render();
    }

    return 0;
}