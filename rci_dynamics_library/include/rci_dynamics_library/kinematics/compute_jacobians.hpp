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

// Forward Kinematics
#include "rci_dynamics_library/kinematics/forward_kinematics.hpp"

#define PI_ 3.14159265358979323846
class ComputeJacobians
{
    public:
        ComputeJacobians();

        // Forward Kinematics
        std::shared_ptr<ForwardKinematics> fk_;
        
        // Functions
        Eigen::MatrixXd compute_spatial_jacobians(Eigen::VectorXd q);
 

    private:

};