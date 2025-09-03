#include "rci_dynamics_library/kinematics/forward_kinematics.hpp"

ForwardKinematics::ForwardKinematics()
{
    std::cout<<"Forward Kinematics Library"<<std::endl;
    joint_trans_.resize(7);
    joint_trans_[0] = {0,0,0.333};
    joint_trans_[1] = {0,0,0};
    joint_trans_[2] = {0,-0.316,0};
    joint_trans_[3] = {0.0825, 0, 0};
    joint_trans_[4] = {-0.0825, 0.384, 0};
    joint_trans_[5] = {0,0,0};
    joint_trans_[6] = {0.088, 0, 0};
    
    joint_rpy_.resize(7);
    joint_rpy_[0] = {0, 0, 0};
    joint_rpy_[1] = {-PI_/2, 0, 0};
    joint_rpy_[2] = {PI_/2, 0, 0};
    joint_rpy_[3] = {PI_/2, 0, 0};
    joint_rpy_[4] = {-PI_/2, 0, 0};
    joint_rpy_[5] = {PI_/2, 0, 0};
    joint_rpy_[6] = {PI_/2, 0, 0};

    

    joint_rot_.resize(7);
    local_joint_axis_.resize(7);
    for(int i=0; i<joint_rpy_.size(); i++)
    {
        joint_rot_[i] = rci_utils::rpy_to_rot(joint_rpy_[i]);
        local_joint_axis_[i] << 0,0,1;
    }
    
    auto [T_eef, chain_list] = this->compute_chain_kinematics(joint_trans_, joint_rot_);
    M_ = T_eef;
    // chain_list : q
    // omega 
    for(int i=0; i<joint_rpy_.size(); i++)
    {
        Eigen::VectorXd spatial_joint_axis = chain_list[i].rotation() * local_joint_axis_[i];
        omega_list_.push_back(spatial_joint_axis);
        std::cout<<i<<"-th T : "<<chain_list[i].translation()<<std::endl;

        v_list_.push_back(-omega_list_[i].cross(chain_list[i].translation()));
    }


    std::cout<<"init M\n"<<M_<<std::endl;

}

std::pair<pinocchio::SE3, std::vector<pinocchio::SE3>> ForwardKinematics::compute_chain_kinematics(std::vector<Eigen::Vector3d> joint_trans, std::vector<Eigen::Matrix3d> joint_rot)
{
    assert((joint_trans.size() == joint_rot.size()) && "joint_trans and joint_rot must be the same size");
    
    std::vector<pinocchio::SE3> chain_list;
    pinocchio::SE3 T = pinocchio::SE3::Identity();
    for(int i=0; i<joint_trans.size(); i++)
    {
        pinocchio::SE3 T_chain(joint_rot[i], joint_trans[i]);
        T = T * T_chain;
        chain_list.push_back(T);
    }
    
    return std::make_pair(T, chain_list);
}

pinocchio::SE3 ForwardKinematics::fk_eef(Eigen::VectorXd q)
{
    assert((omega_list_.size() == q.size()) && "joint_trans and joint_rot must be the same size");
    pinocchio::SE3 T = M_;

    std::vector<pinocchio::SE3> exp_list;
    exp_list.resize(q.size());

    for (std::size_t i = q.size(); i-- > 0; ) 
    {
        Eigen::Vector3d omega = omega_list_[i];
        Eigen::Vector3d v = v_list_[i];
        pinocchio::SE3 chain_T = rci_utils::exp6(v, omega, q(i));
        T = chain_T * T;
        exp_list[i] = chain_T;
        // std::cout<<"omega \n"<<omega_list_[i].transpose()<<std::endl;
        // std::cout<<"v \n"<<v.transpose()<<std::endl;
    }
    
    return T;
}


std::pair<pinocchio::SE3, std::vector<pinocchio::SE3>> ForwardKinematics::fk_eef_v2(Eigen::VectorXd q)
{
    assert((omega_list_.size() == q.size()) && "joint_trans and joint_rot must be the same size");
    pinocchio::SE3 T = M_;

    std::vector<pinocchio::SE3> exp_list;
    exp_list.resize(q.size());

    for (std::size_t i = q.size(); i-- > 0; ) 
    {
        Eigen::Vector3d omega = omega_list_[i];
        Eigen::Vector3d v = v_list_[i];
        pinocchio::SE3 chain_T = rci_utils::exp6(v, omega, q(i));
        T = chain_T * T;
        exp_list[i] = chain_T;
        // std::cout<<"omega \n"<<omega_list_[i].transpose()<<std::endl;
        // std::cout<<"v \n"<<v.transpose()<<std::endl;
    }
    
    return std::make_pair(T, exp_list);
}