/**
 * @file g1_mujoco_node.cpp
 * @brief Unitree G1 robot deployment node for MuJoCo simulation
 *
 * This node connects bridge_core (RL controller) with mujoco_ros (simulator)
 * via ROS2 topics. It subscribes to sensor data from mujoco_ros and publishes
 * control commands back.
 *
 * Supported algorithms:
 *   - Baseline: LSTM-based recurrent policy
 *   - Mod: GRU-based recurrent policy
 *   - DreamWAQ: Proprioceptive history-based estimator + actor
 *   - PIE: Depth image + proprioceptive history for parkour locomotion
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <bridge_core/utils/types.hpp>
#include <bridge_core/core/config_manager.hpp>
#include <bridge_core/core/rl_controller.hpp>
#include <bridge_core/core/algorithm_factory.hpp>
#include <bridge_core/interfaces/mujoco_interface.hpp>

#include <memory>

using namespace bridge_core;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("g1_mujoco_node");

    // Get model name (required parameter)
    node->declare_parameter<std::string>("model_name", "");
    std::string model_name = node->get_parameter("model_name").as_string();
    if (model_name.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Required parameter 'model_name' not provided");
        throw std::runtime_error("Required parameter 'model_name' not provided");
    }

    std::string package_share = ament_index_cpp::get_package_share_directory("bridge_g1");
    std::string config_path = package_share + "/models/" + model_name + "/config.yaml";

    RCLCPP_INFO(node->get_logger(), "Loading config from: %s", config_path.c_str());

    try
    {
        // Load configuration
        Config config = ConfigManager::loadConfig(config_path, node);

        // Create robot interface
        auto robot_interface = std::make_shared<MujocoSimInterface>(node);
        robot_interface->initialize(config.robot);

        // Create algorithm using factory
        auto algorithm = AlgorithmFactory::create(config.algorithm.name, node->get_logger());
        if (!algorithm)
        {
            throw std::runtime_error("Unknown algorithm: " + config.algorithm.name);
        }
        algorithm->initialize(node, config.algorithm, config.robot, config.control);

        // Create RL controller
        auto controller = std::make_shared<RLController>(
            node, robot_interface, algorithm, config);

        // Start controller
        controller->start();

        RCLCPP_INFO(node->get_logger(), "G1 MuJoCo node started successfully");
        RCLCPP_INFO(node->get_logger(), "  Robot: %s", config.robot.robot_name.c_str());
        RCLCPP_INFO(node->get_logger(), "  Algorithm: %s", config.algorithm.name.c_str());
        RCLCPP_INFO(node->get_logger(), "  DOFs: %d (active: %zu)",
                    config.robot.num_dof, config.robot.dof_activated_ids.size());

        rclcpp::spin(node);

        controller->stop();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
