/**
 * Matlab, Coppeliasim, ROS co-simulation
 * 
 * Coppeliasim C++ communication class
 * 
 * Dknt 2023.10.21
*/

#ifndef COPCAR_H
#define COPCAR_H

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

double maxLVelocity = 6.0;
const double maxAVelocity = 3;

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
        m_avoid = false;
        m_LV_signal = name + "_linear_velocity_x";
        m_AV_signal = name + "_angular_velocity";

        m_posePub = nh.advertise<geometry_msgs::Twist>(m_name + "/pose", 10);
        m_targetSub = nh.subscribe<geometry_msgs::Twist>(m_name + "/target", 10, std::bind(&Car::cb_target, this, std::placeholders::_1));
    }

    Car(int clientID, std::string name, std::string avoidCarName):
        nh("~"),
        m_clientID(clientID),
        m_name(name),
        m_avoidCarName(avoidCarName) {
        simxGetObjectHandle(clientID, ('/' + name).c_str(), &m_carHandel, simx_opmode_blocking);
        simxGetObjectHandle(clientID, ('/' + avoidCarName).c_str(), &m_avoidCarHandel, simx_opmode_blocking);
        m_avoid = true;

        mthread_updatePose = std::thread(&Car::thread_getPose, this);
        mthread_gotoTarget = std::thread(&Car::thread_gotoTarget, this);

        m_LV_signal = name + "_linear_velocity_x";
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
            geometry_msgs::Twist pose;
            {
                std::unique_lock<std::mutex>(mlock_pose);
                simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
                simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
                pose.linear.x = ms_position[0];
                pose.linear.y = ms_position[1];
                pose.angular.z = angles::normalize_angle(ms_orientation[2] - M_PI_2);
            }
            // std::cout << "\rPose: [" << ms_position[0] << ", " << ms_position[1] << ", " << ms_position[2] << ", "
            //                        << ms_orientation[0] << ", " << ms_orientation[1] << ", " << (ms_orientation[2] - M_PI_2)
            //                        << "]        " << std::flush;
            m_posePub.publish(pose);
            if (m_avoid == true) {
                std::unique_lock<std::mutex>(mlock_avoidPose);
                simxGetObjectPosition(m_clientID, m_avoidCarHandel, -1, ms_avoidPosition, simx_opmode_blocking);
            }
            std::this_thread::sleep_for(0.05s);
        }
    }

    void thread_gotoTarget() {
        if (m_avoid == true) {
            std::unique_lock<std::mutex>(mlock_avoidPose);
            simxGetObjectPosition(m_clientID, m_avoidCarHandel, -1, ms_avoidPosition, simx_opmode_blocking);
        }
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
            if (m_avoid == true) {
                std::unique_lock<std::mutex>(mlock_avoidPose);
                std::unique_lock<std::mutex>(mlock_pose);
                double distance;
                distance = pow((ms_position[0] - ms_avoidPosition[0]), 2.0);
                distance += pow((ms_position[1] - ms_avoidPosition[1]), 2.0);
                distance = sqrt(distance);
                // std::cout << "[" << m_name << "]" << " Current distance: " << distance << std::endl;

                maxLVelocity = 6.0;
                if (distance < 4) {
                    maxLVelocity = 2.0;
                }
                else if (distance < 2.2) {
                    maxLVelocity = 0.8;
                }
                

                if (distance < 1.5) {
                    setVelocity(-0.2, 0);
                    std::this_thread::sleep_for(0.01s);
                    continue;
                }
                if (distance < 1.8) {
                    setVelocity(0, 0);
                    std::this_thread::sleep_for(0.01s);
                    continue;
                }
            }
            else {
                maxLVelocity = 6.0;
            }
            if (!m_moveState) {
                setVelocity(0, 0);
                std::this_thread::sleep_for(0.01s);
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
            if (distance > 1.0 && abs(phiDistance) > 0.15) {
                double angularVelocity = clip(10 * phiDistance, maxAVelocity);
                // std::cout << "State 1 - distance: " << distance << " phiDistance: " << phi << std::endl;
                setVelocity(0, angularVelocity);
            }
            else if (distance > 0.5) {
                // std::cout << "State 2 - distance: " << distance << " phi: " << phi << std::endl;
                // 滑移
                double angularVelocity = clip(15.0 * phiDistance, maxAVelocity);
                double linearVelocity = clip(1.2 * distance, maxLVelocity);
                setVelocity(linearVelocity, angularVelocity);
            }
            else if (abs(angleDistance) > 0.1) {
                // std::cout << "State 3 - distance: " << distance << " angleDistance: " << angleDistance << std::endl;
                double angularVelocity = clip(10 * angleDistance, maxAVelocity);
                setVelocity(0, angularVelocity);
            }
            else {
                // std::cout << "Finish" << angleDistance << std::endl;
                setVelocity(0, 0);
                m_moveState = false;
            }
            std::this_thread::sleep_for(0.01s);
        }
    }

    bool cb_target(const geometry_msgs::Twist::ConstPtr target) {
        {
            std::unique_lock<std::mutex>(mlock_target);
            double distance = 0.0;
            distance = sqrt(pow(ms_currentTarget.linear.x - target->linear.x, 2.0) + pow(ms_currentTarget.linear.y - target->linear.y, 2.0));
            distance += 10 * abs(ms_currentTarget.angular.z - target->angular.z);
            if (distance < 1.0) {
                return true;
            }
            ms_currentTarget.linear.x = target->linear.x;
            ms_currentTarget.linear.y = target->linear.y;
            ms_currentTarget.angular.z = target->angular.z;
            ROS_INFO("%s new target: x:%f y:%f phi:%f", m_name.c_str(), ms_currentTarget.linear.x, ms_currentTarget.linear.y, ms_currentTarget.angular.z);
        }
        m_moveState = true;
        return true;
    }

    int m_clientID;
    std::string m_name;
    std::string m_avoidCarName;

    std::string m_LV_signal;
    std::string m_AV_signal;
    simxInt m_carHandel;
    simxInt m_avoidCarHandel;
    bool m_moveState = false; // without lock here!
    bool m_avoid = false;

    std::thread mthread_updatePose;
    std::thread mthread_gotoTarget;

    double ms_linear_velocity = 0.0;
    double ms_angular_velocity = 0.0;

    std::mutex mlock_target;
    geometry_msgs::Twist ms_currentTarget;

    std::mutex mlock_pose;
    simxFloat ms_position[3] = {0., 0., 0.};
    simxFloat ms_orientation[3] = {0., 0., 0.};

    std::mutex mlock_avoidPose;
    simxFloat ms_avoidPosition[3] = {0., 0., 0.};

    ros::NodeHandle nh;
    ros::Publisher m_posePub;
    ros::Subscriber m_targetSub;

};

#endif // COPCAR_H