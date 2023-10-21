/**
 * Matlab, Coppeliasim, ROS co-simulation
 * 
 * Coppeliasim C++ communication test
 * 
 * Dknt 2023.10.21
*/

#include "copCar.h"


int main(int argc, char **argv) {
    simxFinish(-1);
    int clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);
    if (clientID == -1) {
        std::cout << "Failed to connect to coppliasim server, exiting..." << std::endl;
        return -1;
    }
    
    Car car1(clientID, "Omnirob_1");
    std::this_thread::sleep_for(2s);

    car1.setVelocity(1.0, 0.2);

    while (true) {
        
    }
    return 0;
}
