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

const double maxLVelocity = 4.0;
const double maxAVelocity = 5.0;
const auto stateMechinePeriod = 0.05s;

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
            geometry_msgs::Twist pose;
            {
                std::unique_lock<std::mutex>(mlock_pose);
                simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
                simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
                pose.linear.x = ms_position[0];
                pose.linear.y = ms_position[1];
                pose.angular.z = angles::normalize_angle(ms_orientation[2] - M_PI_2);
            }
            if (m_avoid == true) {
                std::unique_lock<std::mutex>(mlock_avoidPose);
                simxGetObjectPosition(m_clientID, m_avoidCarHandel, -1, ms_avoidPosition, simx_opmode_blocking);
            }
            // std::cout << "\rPose: [" << ms_position[0] << ", " << ms_position[1] << ", " << ms_position[2] << ", "
            //                        << ms_orientation[0] << ", " << ms_orientation[1] << ", " << (ms_orientation[2] - M_PI_2)
            //                        << "]        " << std::flush;
            m_posePub.publish(pose);
            std::this_thread::sleep_for(0.05s);
        }
    }

    enum controllerState {
        Waiting = 0,
        TurningToTarget = 1,
        MovingToTarget = 2,
        ChangingDirection = 3,
    };

    void thread_gotoTarget() {
        {
            std::unique_lock<std::mutex>(mlock_pose);
            simxGetObjectPosition(m_clientID, m_carHandel, -1, ms_position, simx_opmode_blocking);
            simxGetObjectOrientation(m_clientID, m_carHandel, -1, ms_orientation, simx_opmode_blocking);
        }
        if (m_avoid == true) {
            std::unique_lock<std::mutex>(mlock_avoidPose);
            simxGetObjectPosition(m_clientID, m_avoidCarHandel, -1, ms_avoidPosition, simx_opmode_blocking);
        }
        ms_currentTarget.linear.x = ms_position[0];
        ms_currentTarget.linear.y = ms_position[1];
        ms_currentTarget.angular.z = 0.0;
        while (ros::ok()) {
            if (m_avoid == true) {
                std::unique_lock<std::mutex>(mlock_avoidPose);
                std::unique_lock<std::mutex>(mlock_pose);
                double distance;
                distance = pow(ms_position[0] - ms_avoidPosition[0], 2.0);
                distance += pow(ms_position[1] - ms_avoidPosition[1], 2.0);
                distance = sqrt(distance);
                std::cout << "[" << m_name << "]" << " Current distance: " << distance << std::endl; 

                if (distance < 1.5) {
                    setVelocity(0, 0);
                    std::this_thread::sleep_for(stateMechinePeriod);
                    continue;
                }
            }
            double angleDistance, distance, phi, phiDistance;
            controllerState currentState;
            {
                std::unique_lock<std::mutex>(mlock_target);
                std::unique_lock<std::mutex>(mlock_pose);
                // 仿真中小车前向是 y 负方向
                angleDistance = angles::normalize_angle(ms_currentTarget.angular.z - (ms_orientation[2] - M_PI_2));
                distance = sqrt(pow((ms_currentTarget.linear.x - ms_position[0]), 2.) +
                                    pow((ms_currentTarget.linear.y - ms_position[1]), 2.));
                phi = atan2(ms_currentTarget.linear.y - ms_position[1], ms_currentTarget.linear.x - ms_position[0]);
                phiDistance = angles::normalize_angle(phi - (ms_orientation[2] - M_PI_2));
            }
            {
                std::unique_lock<std::mutex>(mlock_state);
                currentState = m_state;
            }
            
            if (currentState == Waiting) {
                std::this_thread::sleep_for(stateMechinePeriod);
                setVelocity(0, 0);
                continue;
            }
            if (currentState == TurningToTarget) {
                if (distance < 0.2) {
                    std::unique_lock<std::mutex>(mlock_target);
                    m_state = ChangingDirection;
                    continue;
                }
                if (abs(phiDistance) < 0.1) {
                    std::unique_lock<std::mutex>(mlock_target);
                    m_state = MovingToTarget;
                    continue;
                }
                double angularVelocity = clip(6 * phiDistance, maxAVelocity);
                setVelocity(0, angularVelocity);
                std::this_thread::sleep_for(stateMechinePeriod);
                continue;
            }
            if (currentState == MovingToTarget) {
                if (distance < 0.2) {
                    std::unique_lock<std::mutex>(mlock_target);
                    m_state = ChangingDirection;
                    continue;
                }
                double angularVelocity = clip(6 * phiDistance, maxAVelocity);
                double linearVelocity = clip(6 * distance, maxAVelocity);
                setVelocity(linearVelocity, angularVelocity);
                std::this_thread::sleep_for(stateMechinePeriod);
                continue;
            }
            if (currentState == ChangingDirection) {
                if (abs(angleDistance) < 0.05) {
                    std::unique_lock<std::mutex>(mlock_target);
                    m_state = ChangingDirection;
                    continue;
                }
                double angularVelocity = clip(6 * angleDistance, maxAVelocity);
                setVelocity(0, angularVelocity);
                std::this_thread::sleep_for(stateMechinePeriod);
                continue;
            }
        }
    }

    bool cb_target(const geometry_msgs::Twist::ConstPtr target) {
        std::unique_lock<std::mutex>(mlock_target);
        double targetDistant;
        targetDistant = pow(ms_currentTarget.linear.x - target->linear.x, 2.0);
        targetDistant += pow(ms_currentTarget.linear.y - target->linear.y, 2.0);
        targetDistant += pow((ms_currentTarget.angular.z - target->angular.z) * 3, 2.0);
        targetDistant = sqrt(targetDistant);
        if (targetDistant < 0.5) {
            return true;
        }
        ms_currentTarget.linear.x = target->linear.x;
        ms_currentTarget.linear.y = target->linear.y;
        ms_currentTarget.angular.z = target->angular.z;
        {
            std::unique_lock<std::mutex>(mlock_target);
            m_state = TurningToTarget;
        }
        return true;
    }

    int m_clientID;
    std::string m_name;
    std::string m_LV_signal;
    std::string m_AV_signal;
    simxInt m_carHandel;

    bool m_avoid = false;
    std::string m_avoidCarName;
    simxInt m_avoidCarHandel;

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

    std::mutex mlock_state;
    controllerState m_state = Waiting;

    ros::NodeHandle nh;
    ros::Publisher m_posePub;
    ros::Subscriber m_targetSub;

};
