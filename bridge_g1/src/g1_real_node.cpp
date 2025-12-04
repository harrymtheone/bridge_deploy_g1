/**
 * @file g1_real_node.cpp
 * @brief Unitree G1 robot deployment node for real robot hardware
 *
 * This node connects bridge_core (RL controller) with the real G1 robot
 * via Unitree SDK 2.
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <bridge_core/core/types.hpp>
#include <bridge_core/core/config_manager.hpp>
#include <bridge_core/core/rl_controller.hpp>
#include <bridge_core/interfaces/robot_interface.hpp>
#include <bridge_core/algorithms/mod.hpp>
#include <bridge_core/algorithms/dreamwaq.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <array>
#include <thread>

// Unitree SDK headers
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/common/thread/thread.hpp>

// Topic names
static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;
using namespace bridge_core;

// -----------------------------------------------------------------------------
// Helpers from examples
// -----------------------------------------------------------------------------

template <typename T>
class DataBuffer {
public:
    void SetData(const T &newData) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }

    std::shared_ptr<const T> GetData() {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data ? data : nullptr;
    }

    void Clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = nullptr;
    }

private:
    std::shared_ptr<T> data;
    std::shared_mutex mutex;
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// -----------------------------------------------------------------------------
// Gamepad Definitions (from gamepad.hpp)
// -----------------------------------------------------------------------------
namespace unitree_gamepad {
    // bytecode mapping for raw joystick data
    typedef union {
        struct {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
        uint16_t value;
    } xKeySwitchUnion;

    typedef struct {
        uint8_t head[2];
        xKeySwitchUnion btn;
        float lx;
        float rx;
        float ry;
        float L2;
        float ly;
        uint8_t idle[16];
    } xRockerBtnDataStruct;

    typedef union {
        xRockerBtnDataStruct RF_RX;
        uint8_t buff[40];
    } REMOTE_DATA_RX;
}

// -----------------------------------------------------------------------------
// G1 Constants & Structs
// -----------------------------------------------------------------------------
const int G1_NUM_MOTOR = 29;

// Indices for Unitree G1 (matching enum from example)
enum G1JointIndex {
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

// Internal structs for buffers
struct UnitreeMotorCommand {
    std::array<float, G1_NUM_MOTOR> q_target = {};
    std::array<float, G1_NUM_MOTOR> dq_target = {};
    std::array<float, G1_NUM_MOTOR> kp = {};
    std::array<float, G1_NUM_MOTOR> kd = {};
    std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct UnitreeMotorState {
    std::array<float, G1_NUM_MOTOR> q = {};
    std::array<float, G1_NUM_MOTOR> dq = {};
    std::array<float, G1_NUM_MOTOR> tau_est = {};
};

struct UnitreeImuState {
    std::array<float, 4> quat = {1, 0, 0, 0};
    std::array<float, 3> omega = {};
    std::array<float, 3> acc = {};
    std::array<float, 3> rpy = {};
};

// -----------------------------------------------------------------------------
// G1RealInterface
// -----------------------------------------------------------------------------
class G1RealInterface : public RobotInterface {
public:
    G1RealInterface(rclcpp::Node::SharedPtr node, const std::string& network_interface)
        : node_(node), network_interface_(network_interface), 
          is_ready_(false), mode_machine_(0) 
    {
        // Initialize Unitree Channel
        ChannelFactory::Instance()->Init(0, network_interface_);

        // Create publisher/subscriber
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<LowCmd_>>(HG_CMD_TOPIC);
        lowcmd_publisher_->InitChannel();

        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<LowState_>>(HG_STATE_TOPIC);
        lowstate_subscriber_->InitChannel(
            std::bind(&G1RealInterface::LowStateHandler, this, std::placeholders::_1), 1);

        // Initialize gamepad publisher
        joy_pub_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

        // Initialize Motion Switcher to release high-level control
        msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        msc_->SetTimeout(5.0f);
        msc_->Init();
    }

    ~G1RealInterface() override {
        // Stop threads if needed? They are smart pointers, but maybe need join?
        // Unitree threads detach or manage themselves usually.
    }

    void initialize(const RobotConfig& config) override {
        // Try to switch to release mode
        RCLCPP_INFO(node_->get_logger(), "Checking motion switcher...");
        std::string form, name;
        int retry = 0;
        while (msc_->CheckMode(form, name), !name.empty() && retry < 3) {
            RCLCPP_INFO(node_->get_logger(), "Releasing high-level motion...");
            if (msc_->ReleaseMode()) {
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

    RobotState getState() override {
        RobotState state;
        state.resize(G1_NUM_MOTOR);

        auto ms_ptr = motor_state_buffer_.GetData();
        auto imu_ptr = imu_state_buffer_.GetData();

        if (ms_ptr) {
            // Copy motor state
            for(int i=0; i<G1_NUM_MOTOR; ++i) {
                state.motor.q[i] = ms_ptr->q[i];
                state.motor.dq[i] = ms_ptr->dq[i];
                state.motor.tau_est[i] = ms_ptr->tau_est[i];
            }
        }

        if (imu_ptr) {
            state.imu.quaternion = imu_ptr->quat;
            state.imu.gyroscope = imu_ptr->omega;
            state.imu.accelerometer = imu_ptr->acc;
            state.imu.euler = imu_ptr->rpy;
        }

        return state;
    }

    void sendCommand(const RobotCommand& command) override {
        UnitreeMotorCommand mc;
        
        // Convert bridge_core::RobotCommand to internal UnitreeMotorCommand
        // Assumption: command.motor vectors are ordered 0..28 matching G1JointIndex
        for(int i=0; i<G1_NUM_MOTOR; ++i) {
            if (i < (int)command.motor.q.size()) {
                mc.q_target[i] = command.motor.q[i];
                mc.dq_target[i] = command.motor.dq[i];
                mc.kp[i] = command.motor.kp[i];
                mc.kd[i] = command.motor.kd[i];
                mc.tau_ff[i] = command.motor.tau[i];
            }
        }
        
        motor_command_buffer_.SetData(mc);
    }

    bool isReady() const override {
        return is_ready_;
    }

    std::string getRobotName() const override {
        return "G1_Real";
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string network_interface_;
    bool is_ready_;
    uint8_t mode_machine_;
    
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
    ThreadPtr command_writer_ptr_;
    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

    DataBuffer<UnitreeMotorState> motor_state_buffer_;
    DataBuffer<UnitreeMotorCommand> motor_command_buffer_;
    DataBuffer<UnitreeImuState> imu_state_buffer_;

    // Gamepad state
    unitree_gamepad::REMOTE_DATA_RX rx_;

    void LowStateHandler(const void *message) {
        LowState_ low_state = *(const LowState_ *)message;
        
        // Check CRC
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
             // CRC Error
             return;
        }

        // Parse Motor State
        UnitreeMotorState ms;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            ms.q[i] = low_state.motor_state()[i].q();
            ms.dq[i] = low_state.motor_state()[i].dq();
            ms.tau_est[i] = low_state.motor_state()[i].tau_est();
        }
        motor_state_buffer_.SetData(ms);

        // Parse IMU State
        UnitreeImuState imu;
        imu.quat = low_state.imu_state().quaternion();
        imu.omega = low_state.imu_state().gyroscope();
        imu.acc = low_state.imu_state().accelerometer();
        imu.rpy = low_state.imu_state().rpy();
        imu_state_buffer_.SetData(imu);

        // Parse Gamepad & Publish ROS Joy
        memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
        publishJoy(rx_.RF_RX);

        mode_machine_ = low_state.mode_machine();
    }

    void publishJoy(const unitree_gamepad::xRockerBtnDataStruct& key_data) {
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = node_->now();
        joy_msg.header.frame_id = "unitree_remote";

        // Axis mapping (matched to RLController expectations)
        // RLController:
        // axes[1] (fwd) -> ly
        // axes[0] (lat) -> lx
        // axes[3] (yaw) -> rx (or ry?) usually yaw is x-axis of right stick?
        // Let's map:
        // 0: Left Stick X (Lateral) -> lx
        // 1: Left Stick Y (Forward) -> ly
        // 2: Right Stick Y
        // 3: Right Stick X (Yaw) -> rx
        
        joy_msg.axes.resize(4);
        joy_msg.axes[0] = key_data.lx; 
        joy_msg.axes[1] = key_data.ly;
        joy_msg.axes[2] = key_data.ry;
        joy_msg.axes[3] = key_data.rx; 

        // Button mapping
        // RLController:
        // 0: A (Start RL)
        // 1: B (Sit)
        // 4: L1 (Stand/Stop)
        // 5: R1 (Enable Start)
        // 6: Select (Reset)
        
        joy_msg.buttons.resize(16, 0);
        joy_msg.buttons[0] = key_data.btn.components.A;
        joy_msg.buttons[1] = key_data.btn.components.B;
        joy_msg.buttons[2] = key_data.btn.components.X;
        joy_msg.buttons[3] = key_data.btn.components.Y;
        joy_msg.buttons[4] = key_data.btn.components.L1;
        joy_msg.buttons[5] = key_data.btn.components.R1;
        joy_msg.buttons[6] = key_data.btn.components.select;
        joy_msg.buttons[7] = key_data.btn.components.start;
        
        joy_pub_->publish(joy_msg);
    }

    void LowCommandWriter() {
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
        if (mc_ptr) {
            for (int i = 0; i < G1_NUM_MOTOR; i++) {
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
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_real_node");

    // Parameters
    node->declare_parameter<std::string>("model_name", "");
    node->declare_parameter<std::string>("network_interface", "eth0");
    
    std::string model_name = node->get_parameter("model_name").as_string();
    std::string network_interface = node->get_parameter("network_interface").as_string();

    if (model_name.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Required parameter 'model_name' not provided");
        return 1;
    }

    std::string package_share = ament_index_cpp::get_package_share_directory("bridge_g1");
    std::string config_path = package_share + "/models/" + model_name + "/config.yaml";
    
    try {
        // Load Config
        Config config = ConfigManager::loadConfig(config_path, node);

        // Create Real Robot Interface
        auto robot_interface = std::make_shared<G1RealInterface>(node, network_interface);
        robot_interface->initialize(config.robot);

        // Create Algorithm
        std::shared_ptr<AlgorithmInterface> algorithm;
        if (config.algorithm.name == "DreamWAQ") {
            algorithm = std::make_shared<DreamWAQ>();
        } else if (config.algorithm.name == "Mod") {
            algorithm = std::make_shared<Mod>();
        } else {
            throw std::runtime_error("Unknown algorithm: " + config.algorithm.name);
        }
        algorithm->initialize(node, config.algorithm, config.robot, config.control);

        // Create Controller
        auto controller = std::make_shared<RLController>(node, robot_interface, algorithm, config);
        
        // Start
        controller->start();
        rclcpp::spin(node);
        controller->stop();

    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

