#pragma once

// CPP Standard
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

// ROS2
// #include "rclcpp/rclcpp.h"

// Linear Algebra
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR> 

// Math
#include "rci_dynamics_library/utils/math.hpp"
#include "rci_dynamics_library/utils/logger.hpp"

// SE3
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>

#define PI_ 3.14159265358979323846
class ForwardKinematics
{
    public:
        ForwardKinematics();

        // Joint Tree
        std::vector<Eigen::Vector3d> joint_trans_;
        std::vector<Eigen::Vector3d> joint_rpy_;
        std::vector<Eigen::Matrix3d> joint_rot_;
        std::vector<Eigen::Vector3d> local_joint_axis_;
        std::vector<Eigen::Vector3d> v_list_;
        std::vector<Eigen::Vector3d> omega_list_;
        
        
        std::pair<pinocchio::SE3, std::vector<pinocchio::SE3>> compute_chain_kinematics(std::vector<Eigen::Vector3d> joint_trans, std::vector<Eigen::Matrix3d> joint_rot);
        pinocchio::SE3 fk_eef(Eigen::VectorXd q);

        pinocchio::SE3 M_;

    private:

};