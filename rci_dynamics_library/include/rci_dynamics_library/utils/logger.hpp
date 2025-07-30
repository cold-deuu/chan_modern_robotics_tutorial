#pragma once

#include <sstream>
#include <string>
#include <vector>

// Pinocchio
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR> 

// ROS 
#include <rclcpp/rclcpp.hpp>

namespace rci_utils
{
    inline void loginfo_string(const std::string& name, const std::string& content, rclcpp::Logger logger)
    {
        RCLCPP_INFO(logger, "%s: %s", name.c_str(), content.c_str());
    }

    inline void logwarn_string(const std::string& name, const std::string& content, rclcpp::Logger logger)
    {
        RCLCPP_WARN(logger, "%s: %s", name.c_str(), content.c_str());
    }    

    inline void loginfo_vector(const std::string& name, const Eigen::VectorXd& vec, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "[";
        for (int i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i != vec.size() - 1) ss << ", ";
        }
        ss << "]";
        RCLCPP_INFO(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }

    inline void loginfo_matrix(const std::string& name, const Eigen::MatrixXd& mat, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "\n" << mat;
        RCLCPP_INFO(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }

    inline void loginfo_se3(const std::string& name, const pinocchio::SE3& pose, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "\n[Translation]:\n" << pose.translation().transpose();
        ss << "\n[Rotation]:\n" << pose.rotation();
        RCLCPP_INFO(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }

    inline void logwarn_vector(const std::string& name, const Eigen::VectorXd& vec, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "[";
        for (int i = 0; i < vec.size(); ++i)
        {
            ss << vec[i];
            if (i != vec.size() - 1) ss << ", ";
        }
        ss << "]";
        RCLCPP_WARN(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }

    inline void logwarn_matrix(const std::string& name, const Eigen::MatrixXd& mat, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "\n" << mat;
        RCLCPP_WARN(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }

    inline void logwarn_se3(const std::string& name, const pinocchio::SE3& pose, rclcpp::Logger logger)
    {
        std::stringstream ss;
        ss << "\n[Translation]: " << pose.translation().transpose();
        ss << "\n[Rotation]:\n" << pose.rotation();
        RCLCPP_WARN(logger, "%s: %s", name.c_str(), ss.str().c_str());
    }
}