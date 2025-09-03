// STANDARD CPP
#include <iostream>
#include <memory>

// RCLCPP
#include "rclcpp/rclcpp.hpp"

// FK TESTER
#include "rci_dynamics_library/unit_test/forward_kinematics/fk_test.hpp"
#include "rci_dynamics_library/unit_test/compute_jacobians/jacob_test.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = nullptr;

    std::cout << "Select Tester:\n";
    std::cout << "1. Forward Kinematics\n";
    std::cout << "2. Compute Jacobians\n";
    std::cout << "Enter choice (number): ";
    int choice;
    std::cin >> choice;

    try
    {
        if (choice == 1)
        {
            node = std::make_shared<unit_test::Fk_Tester>();
        }
        else if (choice == 2)
        {
            node = std::make_shared<unit_test::Jacob_Tester>();
        }
        else
        {
            throw std::runtime_error("Invalid choice. No node selected.");
        }

        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
