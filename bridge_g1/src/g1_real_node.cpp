/**
 * @file g1_real_node.cpp
 * @brief Unitree G1 robot deployment node for real robot hardware
 *
 * This node connects bridge_core (RL controller) with the real G1 robot
 * via Unitree SDK 2.
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <bridge_core/utils/types.hpp>
#include <bridge_core/core/config_manager.hpp>
#include <bridge_core/core/rl_controller.hpp>
#include <bridge_core/core/algorithm_factory.hpp>
#include <bridge_core/interfaces/robot_interface.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

// Unitree SDK headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/common/thread/thread.hpp>

// Topic names
static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string HG_IMU_TORSO_TOPIC = "rt/secondary_imu";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;
using namespace bridge_core;

// -----------------------------------------------------------------------------
// Helpers from examples
// -----------------------------------------------------------------------------

template <typename T>
class DataBuffer
{
public:
    void SetData(const T &newData)
    {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }

    std::shared_ptr<const T> GetData()
    {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data ? data : nullptr;
    }

    void Clear()
    {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = nullptr;
    }

private:
    std::shared_ptr<T> data;
    std::shared_mutex mutex;
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// -----------------------------------------------------------------------------
// G1 Constants & Structs
// -----------------------------------------------------------------------------
const int G1_NUM_MOTOR = 29;

// Indices for Unitree G1 (matching enum from example)
enum G1JointIndex
{
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleRoll = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleRoll = 11,
    WaistYaw = 12,
    WaistRoll = 13,
    WaistPitch = 14,
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20,
    LeftWristYaw = 21,
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27,
    RightWristYaw = 28
};

// Static mapping from config joint names to Unitree SDK indices
// This allows the config file to use any naming convention
const std::unordered_map<std::string, int> JOINT_NAME_TO_SDK_INDEX = {
    // Left leg (multiple naming conventions supported)
    {"leg_l1_joint", LeftHipPitch},
    {"leg_l2_joint", LeftHipRoll},
    {"leg_l3_joint", LeftHipYaw},
    {"leg_l4_joint", LeftKnee},
    {"leg_l5_joint", LeftAnklePitch},
    {"leg_l6_joint", LeftAnkleRoll},
    {"left_hip_pitch_joint", LeftHipPitch},
    {"left_hip_roll_joint", LeftHipRoll},
    {"left_hip_yaw_joint", LeftHipYaw},
    {"left_knee_joint", LeftKnee},
    {"left_ankle_pitch_joint", LeftAnklePitch},
    {"left_ankle_roll_joint", LeftAnkleRoll},

    // Right leg
    {"leg_r1_joint", RightHipPitch},
    {"leg_r2_joint", RightHipRoll},
    {"leg_r3_joint", RightHipYaw},
    {"leg_r4_joint", RightKnee},
    {"leg_r5_joint", RightAnklePitch},
    {"leg_r6_joint", RightAnkleRoll},
    {"right_hip_pitch_joint", RightHipPitch},
    {"right_hip_roll_joint", RightHipRoll},
    {"right_hip_yaw_joint", RightHipYaw},
    {"right_knee_joint", RightKnee},
    {"right_ankle_pitch_joint", RightAnklePitch},
    {"right_ankle_roll_joint", RightAnkleRoll},

    // Waist
    {"waist_yaw_joint", WaistYaw},
    {"waist_roll_joint", WaistRoll},
    {"waist_pitch_joint", WaistPitch},

    // Left arm
    {"arm_l1_joint", LeftShoulderPitch},
    {"arm_l2_joint", LeftShoulderRoll},
    {"arm_l3_joint", LeftShoulderYaw},
    {"arm_l4_joint", LeftElbow},
    {"arm_l5_joint", LeftWristRoll},
    {"arm_l6_joint", LeftWristPitch},
    {"arm_l7_joint", LeftWristYaw},
    {"left_shoulder_pitch_joint", LeftShoulderPitch},
    {"left_shoulder_roll_joint", LeftShoulderRoll},
    {"left_shoulder_yaw_joint", LeftShoulderYaw},
    {"left_elbow_joint", LeftElbow},
    {"left_wrist_roll_joint", LeftWristRoll},
    {"left_wrist_pitch_joint", LeftWristPitch},
    {"left_wrist_yaw_joint", LeftWristYaw},

    // Right arm
    {"arm_r1_joint", RightShoulderPitch},
    {"arm_r2_joint", RightShoulderRoll},
    {"arm_r3_joint", RightShoulderYaw},
    {"arm_r4_joint", RightElbow},
    {"arm_r5_joint", RightWristRoll},
    {"arm_r6_joint", RightWristPitch},
    {"arm_r7_joint", RightWristYaw},
    {"right_shoulder_pitch_joint", RightShoulderPitch},
    {"right_shoulder_roll_joint", RightShoulderRoll},
    {"right_shoulder_yaw_joint", RightShoulderYaw},
    {"right_elbow_joint", RightElbow},
    {"right_wrist_roll_joint", RightWristRoll},
    {"right_wrist_pitch_joint", RightWristPitch},
    {"right_wrist_yaw_joint", RightWristYaw},
};

// Internal structs for buffers
struct UnitreeMotorCommand
{
    std::array<float, G1_NUM_MOTOR> q_target = {};
    std::array<float, G1_NUM_MOTOR> dq_target = {};
    std::array<float, G1_NUM_MOTOR> kp = {};
    std::array<float, G1_NUM_MOTOR> kd = {};
    std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct UnitreeMotorState
{
    std::array<float, G1_NUM_MOTOR> q = {};
    std::array<float, G1_NUM_MOTOR> dq = {};
    std::array<float, G1_NUM_MOTOR> tau_est = {};
};

struct UnitreeImuState
{
    std::array<float, 4> quat = {1, 0, 0, 0};
    std::array<float, 3> omega = {};
    std::array<float, 3> acc = {};
    std::array<float, 3> rpy = {};
};

// -----------------------------------------------------------------------------
// G1RealInterface
// -----------------------------------------------------------------------------
class G1RealInterface : public RobotInterface
{
public:
    G1RealInterface(rclcpp::Node::SharedPtr node, const std::string &network_interface)
        : node_(node), network_interface_(network_interface),
          is_ready_(false), mode_machine_(0), imu_source_(1)
    {
        // Initialize Unitree Channel
        ChannelFactory::Instance()->Init(0, network_interface_);

        // Create publisher/subscriber
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<LowCmd_>>(HG_CMD_TOPIC);
        lowcmd_publisher_->InitChannel();

        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<LowState_>>(HG_STATE_TOPIC);
        lowstate_subscriber_->InitChannel(
            std::bind(&G1RealInterface::LowStateHandler, this, std::placeholders::_1), 1);

        // Initialize Motion Switcher to release high-level control
        msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        msc_->SetTimeout(5.0f);
        msc_->Init();
    }

    ~G1RealInterface() override = default;

    void initialize(const RobotConfig &config) override
    {
        config_ = config;

        // Build config index <-> SDK index mappings
        config_to_sdk_.resize(config_.joint_names.size(), -1);
        sdk_to_config_.fill(-1);

        for (size_t cfg_idx = 0; cfg_idx < config_.joint_names.size(); ++cfg_idx)
        {
            const std::string &joint_name = config_.joint_names[cfg_idx];
            auto it = JOINT_NAME_TO_SDK_INDEX.find(joint_name);
            if (it != JOINT_NAME_TO_SDK_INDEX.end())
            {
                int sdk_idx = it->second;
                config_to_sdk_[cfg_idx] = sdk_idx;
                sdk_to_config_[sdk_idx] = static_cast<int>(cfg_idx);
                RCLCPP_DEBUG(node_->get_logger(), "Mapped joint '%s': config[%zu] <-> SDK[%d]",
                             joint_name.c_str(), cfg_idx, sdk_idx);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Unknown joint name '%s' at config index %zu",
                            joint_name.c_str(), cfg_idx);
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Built joint mapping for %zu joints", config_.joint_names.size());

        // Read IMU source from config: 0 = torso, 1 = pelvis (default)
        if (config.yaml && config.yaml["imu_source"]) {
            imu_source_ = config.yaml["imu_source"].as<int>();
        }
        
        if (imu_source_ == 0) {
            // Subscribe to torso IMU (secondary IMU)
            RCLCPP_INFO(node_->get_logger(), "Using Torso IMU (rt/secondary_imu)");
            imutorso_subscriber_ = std::make_shared<ChannelSubscriber<IMUState_>>(HG_IMU_TORSO_TOPIC);
            imutorso_subscriber_->InitChannel(
                std::bind(&G1RealInterface::TorsoImuHandler, this, std::placeholders::_1), 1);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Using Pelvis IMU (from LowState)");
        }

        // Try to switch to release mode
        RCLCPP_INFO(node_->get_logger(), "Checking motion switcher...");
        std::string form, name;
        int retry = 0;
        while (msc_->CheckMode(form, name), !name.empty() && retry < 3)
        {
            RCLCPP_INFO(node_->get_logger(), "Releasing high-level motion...");
            if (msc_->ReleaseMode())
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to switch to Release Mode");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            retry++;
        }

        // Start command writer thread (500Hz)
        command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000,
                                                      &G1RealInterface::LowCommandWriter, this);

        is_ready_ = true;
        RCLCPP_INFO(node_->get_logger(), "G1 Real Interface initialized");
    }

    RobotState getState() override
    {
        RobotState state;
        state.resize(config_.joint_names.size());
        state.motor.names = config_.joint_names;

        auto ms_ptr = motor_state_buffer_.GetData();
        auto imu_ptr = imu_state_buffer_.GetData();

        if (ms_ptr)
        {
            // Copy motor state using SDK -> Config mapping
            for (int sdk_idx = 0; sdk_idx < G1_NUM_MOTOR; ++sdk_idx)
            {
                int cfg_idx = sdk_to_config_[sdk_idx];
                if (cfg_idx >= 0 && cfg_idx < static_cast<int>(state.motor.q.size()))
                {
                    state.motor.q[cfg_idx] = ms_ptr->q[sdk_idx];
                    state.motor.dq[cfg_idx] = ms_ptr->dq[sdk_idx];
                    state.motor.tau_est[cfg_idx] = ms_ptr->tau_est[sdk_idx];
                }
            }
        }

        if (imu_ptr)
        {
            state.imu.quaternion = imu_ptr->quat;
            state.imu.gyroscope = imu_ptr->omega;
            state.imu.accelerometer = imu_ptr->acc;
            state.imu.euler = imu_ptr->rpy;
        }

        return state;
    }

    void sendCommand(const RobotCommand &command) override
    {
        UnitreeMotorCommand mc;

        // Convert bridge_core::RobotCommand to internal UnitreeMotorCommand
        // Using command.motor.names to map to SDK order
        for (size_t i = 0; i < command.motor.names.size(); ++i)
        {
            const std::string &joint_name = command.motor.names[i];
            auto it = JOINT_NAME_TO_SDK_INDEX.find(joint_name);
            if (it == JOINT_NAME_TO_SDK_INDEX.end())
            {
                continue; // Unknown joint, skip
            }
            int sdk_idx = it->second;

            if (i < command.motor.q.size())
            {
                mc.q_target[sdk_idx] = command.motor.q[i];
            }
            if (i < command.motor.dq.size())
            {
                mc.dq_target[sdk_idx] = command.motor.dq[i];
            }
            if (i < command.motor.kp.size())
            {
                mc.kp[sdk_idx] = command.motor.kp[i];
            }
            if (i < command.motor.kd.size())
            {
                mc.kd[sdk_idx] = command.motor.kd[i];
            }
            if (i < command.motor.tau.size())
            {
                mc.tau_ff[sdk_idx] = command.motor.tau[i];
            }
        }

        motor_command_buffer_.SetData(mc);
    }

    bool isReady() const override
    {
        return is_ready_;
    }

    std::string getRobotName() const override
    {
        return "G1_Real";
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string network_interface_;
    bool is_ready_;
    uint8_t mode_machine_;
    int imu_source_;  // 0 = torso, 1 = pelvis

    // Configuration and joint mapping
    RobotConfig config_;
    std::vector<int> config_to_sdk_;                   // config index -> SDK index
    std::array<int, G1_NUM_MOTOR> sdk_to_config_ = {}; // SDK index -> config index

    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;  // Torso IMU (secondary)
    ThreadPtr command_writer_ptr_;
    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    DataBuffer<UnitreeMotorState> motor_state_buffer_;
    DataBuffer<UnitreeMotorCommand> motor_command_buffer_;
    DataBuffer<UnitreeImuState> imu_state_buffer_;

    void LowStateHandler(const void *message)
    {
        LowState_ low_state = *(const LowState_ *)message;

        // Check CRC
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
        {
            // CRC Error
            return;
        }

        // Parse Motor State
        UnitreeMotorState ms;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            ms.q[i] = low_state.motor_state()[i].q();
            ms.dq[i] = low_state.motor_state()[i].dq();
            ms.tau_est[i] = low_state.motor_state()[i].tau_est();
        }
        motor_state_buffer_.SetData(ms);

        // Parse IMU State from pelvis (only if using pelvis IMU)
        if (imu_source_ == 1)
        {
            UnitreeImuState imu;
            imu.quat = low_state.imu_state().quaternion();
            imu.omega = low_state.imu_state().gyroscope();
            imu.acc = low_state.imu_state().accelerometer();
            imu.rpy = low_state.imu_state().rpy();
            imu_state_buffer_.SetData(imu);
        }

        mode_machine_ = low_state.mode_machine();
    }

    void TorsoImuHandler(const void *message)
    {
        // Parse IMU State from torso (secondary IMU)
        IMUState_ imu_torso = *(const IMUState_ *)message;
        UnitreeImuState imu;
        imu.quat = imu_torso.quaternion();
        imu.omega = imu_torso.gyroscope();
        imu.acc = imu_torso.accelerometer();
        imu.rpy = imu_torso.rpy();
        imu_state_buffer_.SetData(imu);
    }

    void LowCommandWriter()
    {
        LowCmd_ dds_low_command;
        // 0 = PR mode (Pitch/Roll) which seems to be default for normal control?
        // The examples switch between PR and AB. PR is kinematics based, AB is actuator based.
        // For RL controlling all joints, we likely want direct motor control.
        // G1 joints 4/5 (Ankle) are coupled.
        // If RL outputs Pitch/Roll for ankle, we use PR.
        // If RL outputs raw motor positions (A/B), we use AB.
        // Usually RL trains on "joint positions" which implies Pitch/Roll in kinematic model.
        // So we should use PR mode for Ankle.
        dds_low_command.mode_pr() = 0; // PR mode
        dds_low_command.mode_machine() = mode_machine_;

        auto mc_ptr = motor_command_buffer_.GetData();
        if (mc_ptr)
        {
            for (int i = 0; i < G1_NUM_MOTOR; i++)
            {
                dds_low_command.motor_cmd().at(i).mode() = 1; // Enable
                dds_low_command.motor_cmd().at(i).q() = mc_ptr->q_target[i];
                dds_low_command.motor_cmd().at(i).dq() = mc_ptr->dq_target[i];
                dds_low_command.motor_cmd().at(i).kp() = mc_ptr->kp[i];
                dds_low_command.motor_cmd().at(i).kd() = mc_ptr->kd[i];
                dds_low_command.motor_cmd().at(i).tau() = mc_ptr->tau_ff[i];
            }

            dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
        }
    }

};

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_real_node");

    // Parameters
    node->declare_parameter<std::string>("model_name", "");
    node->declare_parameter<std::string>("network_interface", "eth0");

    std::string model_name = node->get_parameter("model_name").as_string();
    std::string network_interface = node->get_parameter("network_interface").as_string();

    if (model_name.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Required parameter 'model_name' not provided");
        return 1;
    }

    std::string package_share = ament_index_cpp::get_package_share_directory("bridge_g1");
    std::string config_path = package_share + "/models/" + model_name + "/config.yaml";

    try
    {
        // Load Config
        Config config = ConfigManager::loadConfig(config_path, node);

        // Create Real Robot Interface
        auto robot_interface = std::make_shared<G1RealInterface>(node, network_interface);
        robot_interface->initialize(config.robot);

        // Create Algorithm using factory
        auto algorithm = AlgorithmFactory::create(config.algorithm.name, node->get_logger());
        if (!algorithm)
        {
            throw std::runtime_error("Unknown algorithm: " + config.algorithm.name);
        }
        algorithm->initialize(node, config.algorithm, config.robot, config.control);

        // Create Controller
        auto controller = std::make_shared<RLController>(node, robot_interface, algorithm, config);

        // Start
        controller->start();
        rclcpp::spin(node);
        controller->stop();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
