#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace robot
{
    class RobotConfig
    {
        public:
            int action_dim;
            int observation_dim;
            std::string device;
            int control_rate;
            std::string policy_file_path;
            float action_scale;
            std::vector<float> default_joint_positions;
            std::vector<float> standing_joint_positions;
            std::vector<float> kneeling_joint_positions;
            std::vector<float> kps;
            std::vector<float> kds;
            float max_linear_velocity_x;
            float max_linear_velocity_y;
            float max_angular_velocity;

            /* Default constructor */
            RobotConfig() = default;

            /* Static method to load configuration from a YAML file */
            static RobotConfig fromYaml(const std::string &config_file_path)
            {
                RobotConfig config;

                try
                {
                    YAML::Node yaml_config = YAML::LoadFile(config_file_path);

                    config.action_dim = yaml_config["action_dim"].as<int>();
                    config.observation_dim = yaml_config["observation_dim"].as<int>();
                    config.device = yaml_config["device"].as<std::string>();
                    config.control_rate = yaml_config["control_rate"].as<int>();
                    config.policy_file_path = yaml_config["policy_file_path"].as<std::string>();
                    config.action_scale = yaml_config["action_scale"].as<float>();
                    config.default_joint_positions = yaml_config["default_joint_positions"].as<std::vector<float>>();
                    config.standing_joint_positions = yaml_config["standing_joint_positions"].as<std::vector<float>>();
                    config.kneeling_joint_positions = yaml_config["kneeling_joint_positions"].as<std::vector<float>>();
                    config.kps = yaml_config["kps"].as<std::vector<float>>();
                    config.kds = yaml_config["kds"].as<std::vector<float>>();
                    config.max_linear_velocity_x = yaml_config["max_linear_velocity_x"].as<float>();
                    config.max_linear_velocity_y = yaml_config["max_linear_velocity_y"].as<float>();
                    config.max_angular_velocity = yaml_config["max_angular_velocity"].as<float>();
                }
                catch (const std::exception &e)
                {
                    throw std::runtime_error("Failed to load YAML configuration: " + std::string(e.what()));
                }

                return config;
            }
    };

    class LegState
    {
        public:
            std::vector<float> position;
            std::vector<float> velocity;

            /* Constructor */
            LegState(const RobotConfig &config): position(config.action_dim, 0.0f), velocity(config.action_dim, 0.0f) {}
    };

    class BaseState
    {
        public:
            std::vector<float> vB;
            std::vector<float> wB;
            std::vector<float> orientation;
            std::vector<float> position;

            /* Constructor */
            BaseState(const RobotConfig &config): vB(3, 0.0f), wB(3, 0.0f), orientation(4, 0.0f), position(3, 0.0f) {}
    };

    class Robot
    {
        public:
            /* Constructor */
            Robot(const RobotConfig &config) : _leg_states(config), _base_state(config)
            {
                // Initialize callback flags
                _callback_flags["joint_states_callback"] = false;
                _callback_flags["base_state_callback"] = false;
                // _callback_flags["imu_callback"] = false;

            }

            /* Check if the robot is ready */
            bool is_ready() const
            {
                for (const auto &flag : _callback_flags)
                {
                    if (!flag.second) return false;
                }
                return true;
            }

            /* Retrieve the velocity of the robot's base */
            const std::vector<float> &vB() const
            {
                return _base_state.vB;
            }

            /* Retrieve the angular velocity of the robot's base */
            const std::vector<float> &wB() const
            {
                return _base_state.wB;
            }

            /* Retrieve the orientation of the robot's base */
            const std::vector<float> &orientation() const
            {
                return _base_state.orientation;
            }

            /* Retrieve the projected gravity vector (placeholder implementation) */
            const std::vector<float> &projected_g() const
            {
                return orientation();
            }

            /* Retrieve the joint positions of the robot */
            const std::vector<float> &joint_positions() const
            {
                return _leg_states.position;
            }

            /* Retrieve the joint velocities of the robot */
            const std::vector<float> &joint_velocities() const
            {
                return _leg_states.velocity;
            }

            /* Check if the robot is in a safe state */
            bool is_safe() const
            {
                bool is_safe = true;

                /*Check roll and pitch angles*/
                is_safe = is_safe && (std::abs(projected_g()[0]) < 0.7f);
                is_safe = is_safe && (std::abs(projected_g()[1]) < 0.7f);

                return is_safe;
            }

            /* Callback for joint states */
            void update_joint_states(const std::vector<float> &positions, const std::vector<float> &velocities)
            {
                /*Check if values are finite and update joint states*/
                if (std::all_of(positions.begin(), positions.end(), [](float v) { return std::isfinite(v); }))
                {
                    _leg_states.position = positions;
                }

                if (std::all_of(velocities.begin(), velocities.end(), [](float v) { return std::isfinite(v); }))
                {
                    _leg_states.velocity = velocities;
                }

                // Set the callback flag to true
                _callback_flags["joint_states_callback"] = true;
            }

            /* Callback for IMU data */
            void update_imu(const std::vector<float> &orientation_data, const std::vector<float> &angular_velocity_data)
            {
                // Check if values are finite and update base states
                if (std::all_of(orientation_data.begin(), orientation_data.end(), [](float v) { return std::isfinite(v); }))
                {
                    _base_state.orientation = orientation_data;
                }

                if (std::all_of(angular_velocity_data.begin(), angular_velocity_data.end(), [](float v) { return std::isfinite(v); }))
                {
                    _base_state.wB = angular_velocity_data;
                }

                // Set the callback flag to true
                _callback_flags["imu_callback"] = true;
            }

            /* Callback for base state */
            void update_base_state(const std::vector<float> &position, const std::vector<float> &orientation)
            {
                // Check if values are finite and update base states
                if (std::all_of(position.begin(), position.end(), [](float v) { return std::isfinite(v) && std::isnan(v); }))
                {
                    _base_state.position = position;
                }

                if (std::all_of(orientation.begin(), orientation.end(), [](float v) { return std::isfinite(v) && std::isnan(v); }))
                {
                    _base_state.orientation = orientation;
                }

                // Set the callback flag to true
                _callback_flags["base_state_callback"] = true;
            }

        private:
            LegState _leg_states;
            BaseState _base_state;
            std::unordered_map<std::string, bool> _callback_flags;
    };
}

#endif // ROBOT_HPP
