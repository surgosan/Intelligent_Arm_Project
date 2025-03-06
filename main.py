import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


class OpenManipulatorXControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_control')

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander('arm')
        self.gripper_group = MoveGroupCommander('gripper')

        self.get_logger().info("MoveIt! and OpenManipulator-X ready.")

    def move_to_pose(self, pose):
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return plan

    def go_home(self):
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)
        self.arm_group.stop()


def main():
    rclpy.init()
    manipulator_control = OpenManipulatorXControl()

    # Move to a predefined pose
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.2
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.15
    pose.pose.orientation.w = 1.0

    manipulator_control.move_to_pose(pose)
    manipulator_control.go_home()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
