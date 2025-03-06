import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class OpenManipulatorXControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_control')

        # Publish to the arm_controller instead of MoveIt!
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.get_logger().info("Direct controller control initialized.")

    def move_arm(self, joint_positions, duration=2.0):
        """ Move the arm to the given joint positions """
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)

        msg.points.append(point)
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Sent arm trajectory: {joint_positions}")

    def move_gripper(self, position, duration=1.0):
        """ Move the gripper """
        msg = JointTrajectory()
        msg.joint_names = ["gripper"]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)

        msg.points.append(point)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Sent gripper command: {position}")


def main():
    rclpy.init()
    node = OpenManipulatorXControl()

    # Move arm to a test position
    time.sleep(1)
    node.move_arm([0.0, -0.5, 0.5, 0.0])  # Example joint positions
    time.sleep(2)
    node.move_arm([0.0, 0.5, -0.5, 0.0])
    time.sleep(2)
    node.move_arm([0.5, 0.0, 0.0, 0.5])

    # Open the gripper
    time.sleep(2)
    node.move_gripper(0.2)  # Adjust based on your gripper limits

    # Close the gripper
    time.sleep(2)
    node.move_gripper(-0.2)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
