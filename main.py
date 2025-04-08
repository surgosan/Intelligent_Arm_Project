import rclpy
from mercurial import node
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

# Import the GripperCommand action and ActionClient support
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


class OpenManipulatorXControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_control')

        # Arm controller remains unchanged
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Replace the gripper publisher with an action client on the proper topic
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info("Direct controller control initialized.")

    def move_arm(self, joint_positions, duration=2.0):
        """Move the arm to the given joint positions."""
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        msg.points.append(point)

        self.arm_pub.publish(msg)
        self.get_logger().info(f"Sent arm trajectory: {joint_positions}")

    def move_gripper(self, position, max_effort=10.0):
        """Move the gripper using the GripperActionController via its action interface."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info("Waiting for gripper action server...")
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return

        self.get_logger().info(f"Sending gripper command: position {position}, max_effort {max_effort}")
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper command rejected!")
            return

        self.get_logger().info("Gripper command accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Gripper action result: {result}")


def main():
    rclpy.init()
    node = OpenManipulatorXControl()

    ### ARM CONTROLS ###
    # ARM: node.move_arm([
    # X-Axis (Base Joint) <- Left/Right
    # Y-Axis (Base Joint) <- Up/Down -- Do not go past 0.5, or it will slam into the ground if rest is 0
    # Y-Axis (Mid-Joint)  <- Up/Down -- Do not go past 0.8 or the arm will bend at wrist
    # Y-Axis (Top-Joint)  <- Up/Down
    # ]) <-- Radians

    ### GRIPPER CONTROLS ###
    # node.move_gripper(
    # Grip Position,      <- -0.1 to 0.2 --- Higher Value = more open
    # max_effort=2.0     <- What it sounds like. Max amount of force allowed to protect the system
    # )

    # Move arm up to starting position
    # node.move_gripper(0.1, max_effort=0.1)
    node.move_arm([0.0, 0.0, -2.0, 0.0])
    time.sleep(1)

    # Turn Left 90 degrees
    node.move_arm([2.0, 0.0, -2.0, 0.0])
    time.sleep(1)

    # Lean forward
    node.move_arm([2.0, 1.0, -2.0, 0.0])
    time.sleep(1)

    # Lean backward
    node.move_arm([2.0, -1.0, -2.0, 0.0])
    time.sleep(1)

    # Close the gripper
    # node.move_gripper(0.0, max_effort=0.1)
    time.sleep(2)

    # Straight up & open gripper
    node.move_arm([0.0, 0.0, -2.0, 0.0])
    # node.move_gripper(0.1, max_effort=0.1)
    time.sleep(1)

    # Forward lean. Low to ground and parallel
    node.move_arm([0.0, 1.0, 0.0, -1.0])
    time.sleep(4)

    # Close gripper
    # node.move_gripper(0.0, max_effort=0.1)
    time.sleep(4)

    # Raise up
    node.move_arm([0.0, 0.2, 0.0, -0.2])
    time.sleep(1)

    # Turn Right 90 degrees
    node.move_arm([-2.0, 0.2, 0.0, -0.2])
    time.sleep(1)

    # Go down from there
    node.move_arm([-2.0, 1.0, 0.0, -1.0])
    time.sleep(4)

    # Open gripper
    # node.move_gripper(0.1, max_effort=0.1)
    time.sleep(2)

    # Move back up
    node.move_arm([-2.0, 0.2, 0.0, -0.2])
    time.sleep(1)

    # Go to sleep position
    node.move_arm([0.0, 0.45, -0.1, 1.15])
    time.sleep(1)




    # Reset to home
    # node.move_arm([0.0, 0.0, 0.0, 0.0])
    rclpy.shutdown()


if __name__ == '__main__':
    main()
