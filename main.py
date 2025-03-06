import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class OpenManipulatorXControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_control')

        # Publisher to send joint trajectory commands
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(1)  # Wait for publisher to register

        self.get_logger().info("OpenManipulator-X ready (without MoveIt).")

    def move_to_joint_positions(self, positions, duration=2.0):
        """
        Moves the arm to the specified joint positions.
        :param positions: List of joint angles in radians [joint1, joint2, joint3, joint4]
        :param duration: Time in seconds to reach the position
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "joint1", "joint2", "joint3", "joint4"
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj_msg.points.append(point)

        self.get_logger().info(f"Moving to {positions} over {duration} sec")
        self.arm_pub.publish(traj_msg)

    def go_home(self):
        """Moves the arm to the 'home' position."""
        home_position = [0.0, 0.0, 0.0, 0.0]  # Adjust based on actual home pose
        self.move_to_joint_positions(home_position, duration=2.0)


def main():
    rclpy.init()
    node = OpenManipulatorXControl()

    # Move to a predefined position (adjust values based on your needs)
    target_position = [0.2, -0.5, 0.3, 0.0]  # Example joint angles in radians
    node.move_to_joint_positions(target_position, duration=3.0)

    # Move back to home
    node.go_home()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
