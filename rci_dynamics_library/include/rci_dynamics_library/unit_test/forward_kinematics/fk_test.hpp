#pragma once

// RCLCPP
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// ROS MSGS
#include "sensor_msgs/msg/joint_state.hpp"

// Eigen : Linear Algebra
#include <Eigen/QR>
#include <Eigen/Dense>

// Logger
#include "rci_dynamics_library/utils/logger.hpp"

// Forward Kinematics
#include "rci_dynamics_library/kinematics/forward_kinematics.hpp"

// pinocchio
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

// Random
#include <random>

namespace unit_test
{
    class Fk_Tester : public rclcpp::Node
    {
    public:
        Fk_Tester();
    private:
        // Timer
        void ctrl_timer();
        rclcpp::TimerBase::SharedPtr timer_;

        // FK
        std::shared_ptr<ForwardKinematics> fk_;

        // Pinocchio
        pinocchio::Model model_;
        pinocchio::Data data_;

        // Joint Info
        Eigen::VectorXd q_, v_;
        Eigen::VectorXd q_lower_, q_upper_;
        
        // Random Joint
        Eigen::VectorXd sample_uniform_q();


    };
}