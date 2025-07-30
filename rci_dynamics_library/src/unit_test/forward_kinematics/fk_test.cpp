#include "rci_dynamics_library/unit_test/forward_kinematics/fk_test.hpp"



namespace unit_test
{
    Fk_Tester::Fk_Tester()
    : Node("fk_test") 
    {
        // 10ms == 100Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Fk_Tester::ctrl_timer, this));

        // FK
        fk_ = std::make_shared<ForwardKinematics>();

        // Pinocchio
        std::string pkg_path = ament_index_cpp::get_package_share_directory("franka_description");
        std::string urdf_path = "/robots/panda/panda.urdf";
        std::string urdf_file_path = pkg_path + urdf_path;
        pinocchio::urdf::buildModel(urdf_file_path, model_);
        data_ = pinocchio::Data(model_);

        q_.setZero(model_.nq);
        v_.setZero(model_.nv);
        q_lower_.setZero(model_.nq);
        q_upper_.setZero(model_.nq);
        
        q_upper_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
        q_lower_ <<-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

    }

    void Fk_Tester::ctrl_timer()
    {
        
        std::cout<<"======================================================="<<std::endl;
        q_ = this->sample_uniform_q();
        // rci_utils::loginfo_vector("random q", q_, this->get_logger());
        // q_ << 0.455931, -1.07335, 2.14592, -0.395789, -0.189924, 1.23928, 2.22287;
        // q_ << 0.4, 0.1, 0.3, 0.2, -0.5, 0.2, 0.1;
        
        pinocchio::computeAllTerms(model_, data_, q_, v_);
        pinocchio::SE3 oMi = data_.oMi[model_.getJointId("panda_joint7")];
        rci_utils::loginfo_se3("EEF Pin ",oMi, this->get_logger());

        pinocchio::SE3 M_fk_ = fk_->fk_eef(q_);
        rci_utils::loginfo_se3("EEF FK ",M_fk_, this->get_logger());
        
        
        // pinocchio::computeAllTerms(model_, data_, q_, v_);
        // pinocchio::SE3 oMi1 = data_.oMi[model_.getJointId("panda_joint1")];
        // pinocchio::SE3 oMi2 = data_.oMi[model_.getJointId("panda_joint2")];
        // pinocchio::SE3 oMi3 = data_.oMi[model_.getJointId("panda_joint3")];
        // pinocchio::SE3 oMi4 = data_.oMi[model_.getJointId("panda_joint4")];
        // pinocchio::SE3 oMi5 = data_.oMi[model_.getJointId("panda_joint5")];
        // pinocchio::SE3 oMi6 = data_.oMi[model_.getJointId("panda_joint6")];
        // pinocchio::SE3 oMi7 = data_.oMi[model_.getJointId("panda_joint7")];

        // rci_utils::loginfo_vector("EEF Pin ",oMi1.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi2.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi3.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi4.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi5.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi6.translation(), this->get_logger());
        // rci_utils::loginfo_vector("EEF Pin ",oMi7.translation(), this->get_logger());



    }

    Eigen::VectorXd Fk_Tester::sample_uniform_q()
    {
        assert(q_lower_.size() == q_upper_.size());

        Eigen::VectorXd q_random(q_lower_.size());
        
        // 랜덤 엔진 및 분포 생성
        std::random_device rd;
        std::mt19937 gen(rd());

        for (int i = 0; i < q_random.size(); ++i)
        {
            std::uniform_real_distribution<double> dist(q_lower_(i), q_upper_(i));
            q_random(i) = dist(gen);
        }

        return q_random;
    }

}
