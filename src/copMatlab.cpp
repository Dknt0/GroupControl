/**
 * Matlab, Coppeliasim, ROS co-simulation
 * 
 * Comunication between Coppeliasim and Matlab
 * 
 * Dknt 2023.10.21
*/

#include "copCar.h"
#include <ros/ros.h>

class MultiCars {

};

int main(int argc, char **argv) {
    simxFinish(-1);
    int clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);
    if (clientID == -1) {
        std::cout << "Failed to connect to coppliasim server, exiting..." << std::endl;
        return -1;
    }
    
    Car car1(clientID, "Omnirob_1");

    while (ros::ok()) {
        
    }
    return 0;
}
