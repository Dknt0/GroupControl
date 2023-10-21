/**
 * Matlab, Coppeliasim, ROS co-simulation
 * 
 * Coppeliasim C++ communication class
 * 
 * Dknt 2023.10.21
*/

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>

// #include <ros/ros.h>
#include <geometry_msgs/Twist.h>

extern "C" {
    #include "extApi.h"
}

using namespace std::chrono_literals;

/**
 * Class communication with Coppeliasim 
 * 
*/
class Car {
public:
    Car(int clientID, std::string name):
        m_clientID(clientID),
        m_name(name) {
        simxGetObjectHandle(clientID, ('/' + name).c_str(), &m_carHandel, simx_opmode_blocking);
        m_LV_signal = name + "_linear_velocity";
        m_AV_signal = name + "_angular_velocity";
        mthread_updatePose = std::thread(&Car::thread_getPose, this);
    }

    void setVelocity(const double linear_velocity, const double angular_velocity) {
        simxSetFloatSignal(m_clientID, (simxChar*)m_LV_signal.c_str(), linear_velocity, simx_opmode_oneshot_wait);
        simxSetFloatSignal(m_clientID, (simxChar*)m_AV_signal.c_str(), angular_velocity, simx_opmode_oneshot_wait);
    }

    geometry_msgs::Twist getPose() {
        std::unique_lock<std::mutex>(mlock_pose);
        geometry_msgs::Twist pose;
        pose.linear.x = ms_position[0];
        pose.linear.y = ms_position[1];
        pose.angular.z = ms_orientation[2];
        return pose;
    }
    
private:
    void thread_getPose() {
        while (true) {
            std::unique_lock<std::mutex>(mlock_pose);
            simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
            simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
            // std::cout << "Pose: [" << ms_position[0] << ", " << ms_position[1] << ", " << ms_position[2] << ", "
            //                        << ms_orientation[0] << ", " << ms_orientation[1] << ", " << ms_orientation[2]
            //                        << "]" << std::endl;
            std::this_thread::sleep_for(0.1s);
        }
    }

    int m_clientID;
    std::string m_name;
    std::string m_LV_signal;
    std::string m_AV_signal;
    simxInt m_carHandel;

    std::thread mthread_updatePose;

    double ms_linear_velocity = 0.0;
    double ms_angular_velocity = 0.0;

    std::mutex mlock_pose;
    simxFloat ms_position[3] = {0., 0., 0.};
    simxFloat ms_orientation[3] = {0., 0., 0.};

};

