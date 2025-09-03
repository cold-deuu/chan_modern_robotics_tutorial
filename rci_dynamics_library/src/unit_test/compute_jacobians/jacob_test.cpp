#include "rci_dynamics_library/unit_test/compute_jacobians/jacob_test.hpp"



namespace unit_test
{
    Jacob_Tester::Jacob_Tester()
    : Node("jacobian_test") 
    {
        // 10ms == 100Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Jacob_Tester::ctrl_timer, this));

        // FK
        jacob_ = std::make_shared<ComputeJacobians>();

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

    void Jacob_Tester::ctrl_timer()
    {
        q_ = this->sample_uniform_q();
        pinocchio::computeAllTerms(model_, data_, q_, v_);
        
        std::cout<<"======================================================="<<std::endl;
        Eigen::MatrixXd jacob_pin(6,7);
        pinocchio::getJointJacobian(model_, data_, model_.getJointId("panda_joint7"), pinocchio::WORLD, jacob_pin);
        
        Eigen::MatrixXd jacob_calc = jacob_->compute_spatial_jacobians(q_);

        
        rci_utils::loginfo_matrix("Jacob Pin ",jacob_pin, this->get_logger());
        rci_utils::loginfo_matrix("Jacob Calc ",jacob_calc, this->get_logger());


        // // rci_utils::loginfo_vector("random q", q_, this->get_logger());
        // // q_ << 0.455931, -1.07335, 2.14592, -0.395789, -0.189924, 1.23928, 2.22287;
        // // q_ << 0.4, 0.1, 0.3, 0.2, -0.5, 0.2, 0.1;
        
        // pinocchio::SE3 oMi = data_.oMi[model_.getJointId("panda_joint7")];
        // rci_utils::loginfo_se3("EEF Pin ",oMi, this->get_logger());

        // pinocchio::SE3 M_fk_ = fk_->fk_eef(q_);
        // rci_utils::loginfo_se3("EEF FK ",M_fk_, this->get_logger());
    


    }

    Eigen::VectorXd Jacob_Tester::sample_uniform_q()
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
