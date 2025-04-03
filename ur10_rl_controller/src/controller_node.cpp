#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <signal.h>
#include <memory>
#include "controllers/robot_config.hpp"
#include "controllers/robot_controller.hpp"
#include "controllers/robot_event.hpp"

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode(): Node("rl_controller"), _timer_counter(0), _is_on_timer(false) 
    {
        // Load the robot configuration and initialize the robot controller
        this->declare_parameter<std::string>("config_file", "configs/leg_robot.yaml");
        std::string config_file = this->get_parameter("config_file").as_string();
        auto config = RobotConfig::from_yaml(config_file);
        _config = config;

        // Initialize the robot controller
        _robot_controller = std::make_shared<RobotController>(config, this);

        // Create a timer to call the control loop
        _timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / config.control_rate),
            std::bind(&ControllerNode::timer_callback, this));

        // Create a subscriber to the joint states
        _joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_feedback", 10,
            std::bind(&Robot::joint_states_callback, _robot_controller->robot.get(), std::placeholders::_1));

        // Create a subscriber to the imu data
        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            std::bind(&Robot::imu_callback, _robot_controller->robot.get(), std::placeholders::_1));

        // Create a subscriber to the joystick
        _joystick_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControllerNode::joystick_callback, this, std::placeholders::_1));

        // Create a subscriber to the key_info
        _key_info_sub = this->create_subscription<interfaces::msg::KeyInfo>(
            "key_info", 10, std::bind(&ControllerNode::key_info_callback, this, std::placeholders::_1));

        // Create a publisher to the joint commands
        _joint_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_cmd", 10);
    }

private:
    void key_info_callback(const interfaces::msg::KeyInfo::SharedPtr msg) {
        std::string key_value = msg->key_value;
        std::string key_event = msg->key_event;

        if (key_value == "START" && key_event == "KEY_HOLDING_3S") {
            _robot_controller->push_event(RobotEvent::START_BUTTON_3S);
        } else if (key_value == "BACK" && key_event == "KEY_PRESSED") {
            _robot_controller->push_event(RobotEvent::BACK_BUTTON_PRESSED);
        }
    }

    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto axes = msg->axes;

        double left_x = -axes[0];
        double left_y = axes[1];
        double right_x = axes[3];
        double v_y = left_x * _robot_controller->config.max_linear_velocity_y;
        double v_x = left_y * _robot_controller->config.max_linear_velocity_x;
        double w = right_x * _robot_controller->config.max_angular_velocity;

        _robot_controller->set_command({v_x, v_y, w});
    }

    void timer_callback() {
        _robot_controller->push_event(RobotEvent::TIMER_EVENT);
        if (!_is_on_timer) {
            return;
        }

        _timer_counter++;
        if (_timer_counter > (2.0 / _robot_controller->current_controller.control_dt)) {
            _robot_controller->push_event(RobotEvent::TIME_OUT_2S);
        }
    }

    void reset_timer() {
        _is_on_timer = false;
        _timer_counter = 0;
    }

    void start_timer() {
        _is_on_timer = true;
        _timer_counter = 0;
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystick_sub;
    rclcpp::Subscription<interfaces::msg::KeyInfo>::SharedPtr _key_info_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_cmd_pub;

    std::shared_ptr<RobotController> _robot_controller;
    RobotConfig _config;
    int _timer_counter;
    bool _is_on_timer;
};

void signal_handler(int sig) {
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ControllerNode>();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try 
    {
        while (rclcpp::ok()) 
        {
            node->_robot_controller->run();
            rclcpp::spin_some(node);
        }
    } 
    catch (const std::exception &e) 
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
