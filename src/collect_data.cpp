#include <mujoco_wrapper.h>

int main(){
    MuJoCoWrapper mjw;
    mjw.reset();

    Eigen::VectorXd armCtrlZero = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd eefCmdUp = Eigen::VectorXd::Zero(6); eefCmdUp[2] = 0.2;
    Eigen::VectorXd gripperCtrlZero = Eigen::VectorXd::Zero(2);
    Eigen::Vector2d gripperCtrlClose(-5.0, -5.0);
    Eigen::VectorXd armQDotCmd;

    double simTime = 0.0;
    for (int i = 1; i <= 10; i++) {
//        mjw.setObjectMass(i*1);
        while(true){
            simTime = mjw.getSimTime();
            if (simTime < 2.5) {
                std::cout << "Grasping..." << std::endl;
                mjw.setControl(armCtrlZero, gripperCtrlClose);
            } else if (simTime >= 2.5 && simTime < 5.0) {
                std::cout << "Lifting..." << std::endl;
                armQDotCmd = mjw.eefVelToQDot(eefCmdUp);
                mjw.setControl(armQDotCmd, gripperCtrlZero);
            } else if (simTime >= 5.0 && simTime < 7.5) {
                std::cout << "Testing..." << std::endl;
                mjw.setControl(armCtrlZero, gripperCtrlZero);
            } else if (simTime >= 7.5) {
                std::cout << "Reseting..." << std::endl;
                mjw.reset();
                break;
            }
            mjw.run();
            mjw.render();
//        usleep(1000);
        }
//        std::cout << "Iter # : " << i << std::endl;
//        mjw.printData();
    }
    return 0;
}