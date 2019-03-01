#include <mujoco_wrapper.h>

int main(){
    MuJoCoWrapper mjw;
    mjw.reset();

    Eigen::VectorXd armCtrlZero = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd eefCmdUp = Eigen::VectorXd::Zero(6); eefCmdUp[2] = 0.2;
    Eigen::VectorXd gripperCtrlZero = Eigen::VectorXd::Zero(2);
    Eigen::Vector2d gripperCtrlClose(0, 0);
    Eigen::VectorXd armQDotCmd;

    double simTime = 0.0;
    for (int i = 1; i <= 2; i++) {
        while(true){
            simTime = mjw.getSimTime();
            if (simTime < 2.5) {
                std::cout << "Grasping..." << std::endl;
                mjw.setControl(armCtrlZero, gripperCtrlClose);
            } else if (simTime >= 2.5 && simTime < 3.5) {
                std::cout << "Lifting..." << std::endl;
                armQDotCmd = mjw.eefVelToQDot(eefCmdUp);
                mjw.setControl(armQDotCmd, gripperCtrlClose);
            } else if (simTime >= 3.5 && simTime < 10.0) {
                std::cout << "Testing..." << std::endl;
                mjw.setControl(armCtrlZero, gripperCtrlClose);
            } else if (simTime >= 20.0) {
                std::cout << "Reseting..." << std::endl;
                mjw.reset();
                break;
            }

            mjw.run();
//            mjw.printData();
            mjw.render();
//        std::cout  " | sim time: " << simTime << std::endl;
//            usleep(100000);
        }
//        mjw.printData();


    }
    return 0;
}