#pragma once

#include <sstream>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR> 

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

// ROS2 MSGS
#include "geometry_msgs/msg/pose.hpp"

// ROS 
#include <rclcpp/rclcpp.hpp>



namespace rci_utils
{
    inline Eigen::MatrixXd pinv(Eigen::MatrixXd mat)
    {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(mat);
        return cod.pseudoInverse();
    }

    inline Eigen::MatrixXd pinv_svd(const Eigen::MatrixXd &mat, double epsilon = 1e-6)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const auto &singular_values = svd.singularValues();
        Eigen::VectorXd inv_singular_values = singular_values;

        for (int i = 0; i < singular_values.size(); ++i)
        {
            if (singular_values(i) > epsilon)
                inv_singular_values(i) = 1.0 / singular_values(i);
            else
                inv_singular_values(i) = 0.0; // 제거 (널스페이스 방지)
        }

        return svd.matrixV() * inv_singular_values.asDiagonal() * svd.matrixU().transpose();
    }

    inline pinocchio::SE3 pose_msgs_to_se3(const geometry_msgs::msg::Pose pose)
    {
        Eigen::Quaterniond quat;
        quat.x() = pose.orientation.x;
        quat.y() = pose.orientation.y;
        quat.z() = pose.orientation.w;
        quat.w() = pose.orientation.z;

        Eigen::Vector3d p;
        p<< pose.position.x, pose.position.y, pose.position.z;
        
        return pinocchio::SE3(quat, p);
    }

    inline Eigen::Matrix3d AngleAngle_to_Rot(Eigen::Vector3d axis, double angle) {
        Eigen::Matrix3d Rot;
        double kx, ky, kz, theta, vt;
        kx = axis(0);
        ky = axis(1);
        kz = axis(2);
        theta = angle;
        vt = 1.0 - cos(theta);


        Rot(0, 0) = kx * kx*vt + cos(theta);
        Rot(0, 1) = kx * ky*vt - kz * sin(theta);
        Rot(0, 2) = kx * kz*vt + ky * sin(theta);
        Rot(1, 0) = kx * ky*vt + kz * sin(theta);
        Rot(1, 1) = ky * ky*vt + cos(theta);
        Rot(1, 2) = ky * kz*vt - kx * sin(theta);
        Rot(2, 0) = kx * kz*vt - ky * sin(theta);
        Rot(2, 1) = ky * kz*vt + kx * sin(theta);
        Rot(2, 2) = kz * kz*vt + cos(theta);

        return Rot;
    }

    inline Eigen::Vector3d GetPhi(Eigen::Matrix3d Rot, Eigen::Matrix3d Rotd)
    {
        Eigen::Vector3d phi;
        Eigen::Vector3d s[3], v[3], w[3];
        for (int i = 0; i < 3; i++) {
        v[i] = Rot.block(0, i, 3, 1);
        w[i] = Rotd.block(0, i, 3, 1);
        s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5* phi;
        return phi;
    }

    inline Eigen::Matrix3d rpy_to_rot(Eigen::Vector3d rpy)
    {
        Eigen::Matrix3d rot_z = Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d rot_y = Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d rot_x = Eigen::Matrix3d::Zero(3,3);
        double phi = rpy(0);
        double theta = rpy(1);
        double psi = rpy(2);
        
        rot_z(0,0) = cos(psi);
        rot_z(0,1) = -sin(psi);
        rot_z(1,0) = sin(psi);
        rot_z(1,1) = cos(psi);
        rot_z(2,2) = 1.0;
        
        rot_y(0,0) = cos(theta);
        rot_y(0,2) = sin(theta);
        rot_y(2,0) = -sin(theta);
        rot_y(2,2) = cos(theta);
        rot_y(1,1) = 1.0;
        
        rot_x(1,1) = cos(phi);
        rot_x(1,2) = -sin(phi);
        rot_x(2,1) = sin(phi);
        rot_x(2,2) = cos(phi);
        rot_x(0,0) = 1.0;

        return rot_z * rot_y * rot_x;
    }

    inline Eigen::Matrix3d skew(const Eigen::Vector3d& w)
    {
        Eigen::Matrix3d w_hat;
        w_hat <<     0,    -w(2),  w(1),
                w(2),      0,  -w(0),
                -w(1),   w(0),     0;
        return w_hat;
    }

    inline Eigen::Matrix3d exp3(Eigen::Vector3d w)
    {
        double theta = w.norm();
        Eigen::Vector3d omega_hat = w/theta;
        Eigen::Matrix3d skew_omega = skew(omega_hat);
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity() + sin(theta) * skew_omega + (1 - cos(theta)) * (skew_omega * skew_omega);

        return rot;
    }


    // inline pinocchio::SE3 exp6(Eigen::Vector3d v, Eigen::Vector3d w, double theta)
    // {
    //     assert((v.norm()==1 || w.norm()==1) && "Screw Rule Violated");
    //     Eigen::Matrix3d rot;
    //     Eigen::Vector3d p;
        
    //     std::cout<<"Theta : "<<theta<<std::endl;
    //     if (theta <1e-5)
    //     {
    //         rot = Eigen::Matrix3d::Identity(3,3);
    //         p = v * theta;

    //     }
    //     else{
    //         Eigen::Vector3d omega = theta * w; // w는 정규화 안 함
    //         double angle = omega.norm();
    //         Eigen::Vector3d omega_hat = omega / angle;
    //         Eigen::Matrix3d skew_omega = skew(omega_hat);

    //         Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();

    //         rot = eye + sin(angle) * skew_omega + (1 - cos(angle)) * (skew_omega * skew_omega);
    //         p = (eye * theta + (1-cos(angle))*skew_omega + (angle - sin(angle))*skew_omega*skew_omega)*v;
    //     }

    //     pinocchio::SE3 se3(rot, p);
    //     return se3;
    // }
    // 잘 이해가 안감
    inline pinocchio::SE3 exp6(const Eigen::Vector3d& v, const Eigen::Vector3d& w, double theta)
    {
        // Define twist vector as xi = [v; w], where w is not necessarily unit.
        Eigen::Vector3d omega = w;
        Eigen::Vector3d vel = v;

        Eigen::Matrix3d rot;
        Eigen::Vector3d p;

        double omega_norm = omega.norm();

        // 매우 작은 각도인 경우 선형 근사 사용
        if (omega_norm < 1e-5)
        {
            rot = Eigen::Matrix3d::Identity();
            p = vel * theta;
        }
        else
        {
            // Unit rotation axis
            Eigen::Vector3d omega_hat = omega / omega_norm;
            double angle = theta * omega_norm;

            Eigen::Matrix3d skew_omega = skew(omega_hat);
            Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();

            // Rodrigues' formula for rotation
            rot = eye + sin(angle) * skew_omega + (1 - cos(angle)) * (skew_omega * skew_omega);

            // Compute Jacobian term for translation
            Eigen::Matrix3d V = eye * theta
                            + (1 - cos(angle)) / omega_norm * skew_omega
                            + (angle - sin(angle)) / (omega_norm * omega_norm) * (skew_omega * skew_omega);

            p = V * vel;
        }

        return pinocchio::SE3(rot, p);
    }

    inline Eigen::MatrixXd skew_screw(Eigen::Vector3d v, Eigen::Vector3d w)
    {
        Eigen::Matrix3d skew_omega = skew(w);
        Eigen::MatrixXd skew_screw(4,4);
        skew_screw.topLeftCorner(3,3) = skew_omega;
        skew_screw.topRightCorner(3,1) = v;

        return skew_screw;
    }

    inline Eigen::MatrixXd compute_adj(pinocchio::SE3 se3)
    {
        Eigen::Matrix3d R = se3.rotation();
        Eigen::Vector3d p = se3.translation();
        Eigen::Matrix3d skew_p = skew(p);
        Eigen::MatrixXd adj(6,6);
        adj.setZero();
        adj.topLeftCorner(3,3) = adj.bottomRightCorner(3,3) = R;
        adj.topRightCorner(3,3) = skew_p * R;

        return adj;
    }

}