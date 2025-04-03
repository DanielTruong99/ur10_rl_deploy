import torch
from torch.jit import ScriptModule
import numpy as np
import rclpy

from .robot import Robot
from .robot_config import RobotConfig


class PolicyController(object):
    def __init__(self, config: RobotConfig, robot: Robot) -> None:
        """
        Import the policy, initialize process variables.

        Args:
            config (Config): the configuration object
            robot (Robot): the robot object interface with the state of the robot
        """
        self.config = config
        self.robot = robot
        self.load_policy(self.config.policy_file_path)

        # Initialize process variables
        self._previous_action = np.zeros(self.config.action_dim)
        self._policy_counter = 0
        self._control_dt = 1.0 / self.config.control_rate

    @property
    def control_dt(self) -> float:
        """
        Get the control time step.

        Returns:
            float: the control time step
        """
        return self._control_dt

    def forward(self, command: np.ndarray) -> np.ndarray:
        """
        Forward the policy to get the action.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the joint cmds
        """
        observation = self._compute_observation(command)
        action = self._compute_action(observation)
        joint_cmds = self._process_action(action)
        return joint_cmds

    def load_policy(self, policy_file_path: str) -> None:
        """
        Load the policy from the policy file.

        Args:
            policy_file_path (str): the policy file path

        """
        try:
            self.policy: ScriptModule = torch.jit.load(policy_file_path)
            self.policy.eval()
            self.policy.to(self.config.device)
            rclpy.logging._root_logger.info(f"Policy loaded from {policy_file_path}")

        except Exception as e:
            rclpy.logging._root_logger.error(
                f"Failed to load policy from {policy_file_path}: {e}"
            )

    def reset(self) -> None:
        """
        Reset the controller.
        """
        self._previous_action = np.zeros(self.config.action_dim)
        self._policy_counter = 0

    def _process_action(self, action: np.ndarray) -> np.ndarray:
        """
        Process the action.

        Args:
            action (np.ndarray): the action 

        Returns:
            np.ndarray: the processed action
        """
        self._previous_action = action
        self._policy_counter += 1
        return action * self.config.action_scale + self.config.default_joint_positions

    def _compute_action(self, observation: np.ndarray) -> np.ndarray:
        """
        Compute the action from the policy.

        Args:
            observation (np.ndarray): the observation

        Returns:
            np.ndarray: the action
        """
        with torch.no_grad():
            observation = (
                torch.from_numpy(observation).view(1, -1).float().to(self.config.device)
            )
            action = self.policy(observation).detach().view(-1).numpy()

        return action

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation from the robot state and the previous action.

        Args:
            command (np.ndarray): the action command (x, y, z, qw, qx, qy, qz)

        Returns:
            np.ndarray: the observation [q, qd, command, action]
        """
        observation = np.zeros(self.config.observation_dim)
        num_actions = self.config.action_dim
        observation[:num_actions] = self.robot.leg_states.position - self.config.default_joint_positions
        observation[num_actions : 2 * num_actions] = self.robot.leg_states.velocity
        observation[2 * num_actions : 2 * num_actions + 7] = command
        observation[2 * num_actions + 7 : 3 * num_actions + 7] = self._previous_action

        return observation
