#include <fstream>
#include <mutex>
#include <random>
#include <mujoco_wrapper.h>

int main(){

    std::mutex mutex;
    MuJoCoWrapper mjWrapper(false);
    mjWrapper.reset();

    Eigen::VectorXd armCtrlZero = Eigen::VectorXd::Zero(6); 
    Eigen::VectorXd eefCmdUp = Eigen::VectorXd::Zero(6); eefCmdUp[2] = 0.2;
    Eigen::Vector2d gripperCtrlClose(0, 0);
    Eigen::VectorXd armQDotCmd;

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<double> objectStifnessDist{0.0 , 100.0};

    std::string dataPath = "/home/tarik/projects/ur_mujoco/data/states_50k.csv";

    std::ofstream dataLogFile;
    dataLogFile.open(dataPath);
    int num_runs = 50000;
    double simTime = 0.0;
    for (int i = 1; i <= num_runs; i++) {
        if (i % 100 == 0){
            std::cout << "Completed %" << 100*i/num_runs << std::endl;
        }
        mjWrapper.setObjectStiffness(objectStifnessDist(gen));
        std::vector<double> state;
        std::string stateStr;
        while(true){
            simTime = mjWrapper.getSimTime();
            state = mjWrapper.getState();
            int grasp = mjWrapper.checkGrasping();
            mutex.lock();
            dataLogFile << simTime << "," << std::to_string(state[0]) << "," << std::to_string(state[1])
                        << "," << std::to_string(state[2]) << "," << std::to_string(state[3])
                        << "," << std::to_string(state[4]) << "," << std::to_string(state[5])
                        << "," << grasp << std::endl;
            mutex.unlock();
            if (simTime < 2.5) {
//                std::cout << "Grasping..." << std::endl;
                mjWrapper.setControl(armCtrlZero, gripperCtrlClose);
            } else if (simTime >= 2.5 && simTime < 5.0) {
//                std::cout << "Lifting..." << std::endl;
                armQDotCmd = mjWrapper.eefVelToQDot(eefCmdUp);
                mjWrapper.setControl(armQDotCmd, gripperCtrlClose);
            } else if (simTime >= 5.0 && simTime < 15.0) {
//                std::cout << "Testing..." << std::endl;
                mjWrapper.setControl(armCtrlZero, gripperCtrlClose);
            } else if (simTime >= 15.0) {
//                std::cout << "Reseting..." << std::endl;
                mjWrapper.reset();
                break;
            }
//            mjWrapper.render();
            mjWrapper.run();
//            usleep(100000);
        }
    }
    dataLogFile.close();
    return 0;
}