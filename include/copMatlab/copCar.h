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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

extern "C" {
    #include "extApi.h"
}

using namespace std::chrono_literals;
using std::placeholders::_1;

const double maxLVelocity = 3.0;
const double maxAVelocity = 2;

template <typename T>
T clip(T v1, T v2) {
    if (v1 > v2) return v2;
    if (v1 < -v2) return -v2;
    return v1;
}

template <typename T>
T sign(T v1) {
    if (v1 > 0) return 1;
    if (v1 < 0) return -1;
    return 0;
}

/**
 * Class communication with Coppeliasim 
*/
class Car {
public:
    Car(int clientID, std::string name):
        nh("~"),
        m_clientID(clientID),
        m_name(name) {
        simxGetObjectHandle(clientID, ('/' + name).c_str(), &m_carHandel, simx_opmode_blocking);
        mthread_updatePose = std::thread(&Car::thread_getPose, this);
        mthread_gotoTarget = std::thread(&Car::thread_gotoTarget, this);

        m_LV_signal = name + "_linear_velocity";
        m_AV_signal = name + "_angular_velocity";

        m_posePub = nh.advertise<geometry_msgs::Twist>(m_name + "/pose", 10);
        m_targetSub = nh.subscribe<geometry_msgs::Twist>(m_name + "/target", 10, std::bind(&Car::cb_target, this, std::placeholders::_1));
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
        pose.angular.z = ms_orientation[2] - M_PI_2;
        return pose;
    }
    
private:
    void thread_getPose() {
        while (ros::ok()) {
            {
                std::unique_lock<std::mutex>(mlock_pose);
                simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
                simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
            }
            // std::cout << "\rPose: [" << ms_position[0] << ", " << ms_position[1] << ", " << ms_position[2] << ", "
            //                        << ms_orientation[0] << ", " << ms_orientation[1] << ", " << (ms_orientation[2] - M_PI_2)
            //                        << "]        " << std::flush;
            std::this_thread::sleep_for(0.05s);
        }
    }

    void thread_gotoTarget() {
        {
            std::unique_lock<std::mutex>(mlock_pose);
            simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
            simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
        }
        ms_currentTarget.linear.x = ms_position[0];
        ms_currentTarget.linear.y = ms_position[1];
        ms_currentTarget.angular.z = 0.0;
        std::this_thread::sleep_for(0.2s);
        while (ros::ok()) {
            if (!m_moveState) {
                std::this_thread::sleep_for(0.05s);
                continue;;
            }
            double angleDistance, distance, phi, phiDistance;
            {
                std::unique_lock<std::mutex>(mlock_target);
                // 仿真中小车前向是 y 负方向
                angleDistance = angles::normalize_angle(ms_currentTarget.angular.z - (ms_orientation[2] - M_PI_2));
                distance = sqrt(pow((ms_currentTarget.linear.x - ms_position[0]), 2.) +
                                    pow((ms_currentTarget.linear.y - ms_position[1]), 2.));
                phi = atan2(ms_currentTarget.linear.y - ms_position[1], ms_currentTarget.linear.x - ms_position[0]);
                phiDistance = angles::normalize_angle(phi - (ms_orientation[2] - M_PI_2));
            }
            if (distance > 0.5 && abs(phiDistance) > 0.3) {
                double angularVelocity = sign(phiDistance) * maxAVelocity;
                // std::cout << "State 1 - distance: " << distance << " phiDistance: " << phi << std::endl;
                setVelocity(0, angularVelocity);
            }
            else if (distance > 0.2) {
                // std::cout << "State 2 - distance: " << distance << " phi: " << phi << std::endl;
                // 滑移
                double angularVelocity = 5 * phiDistance;
                double linearVelocity = clip(3 * distance, maxAVelocity);
                setVelocity(linearVelocity, angularVelocity);
            }
            else if (abs(angleDistance) > 0.05) {
                // std::cout << "State 3 - distance: " << distance << " angleDistance: " << angleDistance << std::endl;
                double angularVelocity = clip(2 * angleDistance, maxAVelocity);
                setVelocity(0, angularVelocity);
            }
            else {
                // std::cout << "Finish" << angleDistance << std::endl;
                setVelocity(0, 0);
                m_moveState = false;
            }
            std::this_thread::sleep_for(0.05s);
        }
    }

    bool cb_target(const geometry_msgs::Twist::ConstPtr target) {
        std::unique_lock<std::mutex>(mlock_target);
        ms_currentTarget.linear.x = target->linear.x;
        ms_currentTarget.linear.y = target->linear.y;
        ms_currentTarget.angular.z = target->angular.z;
        m_moveState = true;
        return true;
    }

    int m_clientID;
    std::string m_name;
    std::string m_LV_signal;
    std::string m_AV_signal;
    simxInt m_carHandel;
    bool m_moveState = false; // without lock here!

    std::thread mthread_updatePose;
    std::thread mthread_gotoTarget;

    double ms_linear_velocity = 0.0;
    double ms_angular_velocity = 0.0;

    std::mutex mlock_target;
    geometry_msgs::Twist ms_currentTarget;

    std::mutex mlock_pose;
    simxFloat ms_position[3] = {0., 0., 0.};
    simxFloat ms_orientation[3] = {0., 0., 0.};

    ros::NodeHandle nh;
    ros::Publisher m_posePub;
    ros::Subscriber m_targetSub;

};
