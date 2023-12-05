/**
 * Matlab, Coppeliasim, ROS co-simulation
 *
 * Comunication between Coppeliasim and Matlab
 *
 * Dknt 2023.12.4
 */

#include <ros/ros.h>

#include "velocity_controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "copMatlab");

  simxFinish(-1);
  int clientID = simxStart((simxChar *)"127.0.0.1", 19997, true, true, 2000, 5);
  if (clientID == -1) {
    std::cout << "Failed to connect to coppliasim server, exiting..."
              << std::endl;
    return -1;
  }

  Car car1(clientID, "Omnirob_1");
  Car car2(clientID, "Omnirob_2", "Omnirob_1");
  Car car3(clientID, "Omnirob_3", "Omnirob_2");
  Car car4(clientID, "Omnirob_4", "Omnirob_3");
  Car car5(clientID, "Omnirob_5", "Omnirob_4");

  ros::spin();

  return 0;
}
