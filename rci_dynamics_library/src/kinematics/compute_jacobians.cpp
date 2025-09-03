#include "rci_dynamics_library/kinematics/compute_jacobians.hpp"

ComputeJacobians::ComputeJacobians()
{
    std::cout<<"Jacobian Calculator"<<std::endl;
    fk_ = std::make_shared<ForwardKinematics>();
}

// Need to Fix
// Hard Coded
Eigen::MatrixXd ComputeJacobians::compute_spatial_jacobians(Eigen::VectorXd q)
{
    auto [eef, exp_list] = fk_->fk_eef_v2(q);   
    auto omega_list = fk_->omega_list_;
    auto v_list = fk_->v_list_;

    // Hard Coding
    Eigen::MatrixXd jacob;
    jacob.setZero(6,7);

    Eigen::MatrixXd j1(6,1);
    j1.topLeftCorner(3,1) = v_list[0];
    j1.bottomLeftCorner(3,1) = omega_list[0];

    // std::cout<<"J1 :"<<j1<<std::endl;

    Eigen::MatrixXd j2(6,1);
    j2.topLeftCorner(3,1) = v_list[1];
    j2.bottomLeftCorner(3,1) = omega_list[1];

    Eigen::MatrixXd adj1 = rci_utils::compute_adj(exp_list[0]);
    // std::cout<<"J2 :"<<adj1<<std::endl;

    j2 = adj1 * j2;


    Eigen::MatrixXd j3(6,1);
    j3.topLeftCorner(3,1) = v_list[2];
    j3.bottomLeftCorner(3,1) = omega_list[2];
    Eigen::MatrixXd adj2 = rci_utils::compute_adj(exp_list[0] * exp_list[1]);
    // std::cout<<"J3 :"<<adj2<<std::endl;

    j3 = adj2 * j3;    

    Eigen::MatrixXd j4(6,1);
    j4.topLeftCorner(3,1) = v_list[3];
    j4.bottomLeftCorner(3,1) = omega_list[3];
    Eigen::MatrixXd adj3 = rci_utils::compute_adj(exp_list[0] * exp_list[1] * exp_list[2]);
    j4 = adj3 * j4;

    Eigen::MatrixXd j5(6,1);
    j5.topLeftCorner(3,1) = v_list[4];
    j5.bottomLeftCorner(3,1) = omega_list[4];
    Eigen::MatrixXd adj4 = rci_utils::compute_adj(exp_list[0] * exp_list[1] * exp_list[2] * exp_list[3]);
    j5 = adj4 * j5;

    Eigen::MatrixXd j6(6,1);
    j6.topLeftCorner(3,1) = v_list[5];
    j6.bottomLeftCorner(3,1) = omega_list[5];
    Eigen::MatrixXd adj5 = rci_utils::compute_adj(exp_list[0] * exp_list[1] * exp_list[2] * exp_list[3] * exp_list[4]);
    j6 = adj5 * j6;

    Eigen::MatrixXd j7(6,1);
    j7.topLeftCorner(3,1) = v_list[6];
    j7.bottomLeftCorner(3,1) = omega_list[6];
    Eigen::MatrixXd adj6 = rci_utils::compute_adj(exp_list[0] * exp_list[1] * exp_list[2] * exp_list[3] * exp_list[4] * exp_list[5]);
    j7 = adj6 * j7;

    jacob.block(0,0,6,1) = j1;
    jacob.block(0,1,6,1) = j2;
    jacob.block(0,2,6,1) = j3;
    jacob.block(0,3,6,1) = j4;
    jacob.block(0,4,6,1) = j5;
    jacob.block(0,5,6,1) = j6;
    jacob.block(0,6,6,1) = j7;
    
    return jacob;
}