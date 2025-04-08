import math

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
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info("Robot joins initialized")

    def send_arm_cmd(self, joint_radians, duration):
        robot_control = JointTrajectory()
        robot_control.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        point = JointTrajectoryPoint()
        point.positions = joint_radians
        point.time_from_start.sec = int(duration)
        robot_control.points.append(point)

        self.arm_pub.publish(robot_control)
        self.get_logger().info(f"Sent arm trajectory: {joint_radians}")

    def process_arm_movement(self, joint_degrees):
        """Move the arm to the given joint degrees. Ensure that the positions are safe to move to."""
        # TODO: Adjust duration based on new position distance
        # TODO: Make this function return something (success, fail, limited success)

        ### ARM CONTROLS ###
        # ARM: node.move_arm([
        # 11: X-Axis (Base Joint) <- Left/Right
        # 12: Y-Axis (Base Joint) <- Up/Down -- Do not go past 0.5, or it will slam into the ground if rest is 0
        # 13: Y-Axis (Mid  Joint) <- Up/Down -- Do not go past 0.8 or the arm will bend at wrist
        # 14: Y-Axis (Top  Joint) <- Up/Down
        # ]) <-- Radians

        # Ensure the number of parameters is valid.
        if len(joint_degrees) != 4:
            print(f"Movement Parameters Required: 4 -- Supplied: {len(joint_degrees)}")
            return

        ### ----------------------------- Check absolute limitations ----------------------------- ###
        ### Absolute Limitations as Radian Inputs to Degrees Equivalent ###
        # 11: (-3.14 to 3.14) => -180 degrees to 180 degrees
        # 12: (-1.57 to 1.57) =>  -90 degrees to  90 degrees
        # 13: (-1.57 to 1.00) =>  -90 degrees to  57 degrees
        # 14: (-1.57 to 1.57) =>  -90 degrees to  90 degrees

        # Set to lower limit if user wants to go past
        if joint_degrees[0] < -180:
            print("Cannot rotate less than -180 degrees")
            joint_degrees[0] = -180

        # Set to upper limit if user wants to go past
        if joint_degrees[0] > 180:
            print("Cannot rotate more than 180 degrees")
            joint_degrees[0] = 180

        #  Iterate over joints 2,3,4 and set to min if user wants a lower value
        for index, angle in enumerate(joint_degrees):
            if index == 0: continue # Joint 0 (11) has a range of -180 to 180 degrees
            if angle < -90:
                print(f"Cannot rotate joint {index+1} less than -90 degrees")
                joint_degrees[index] = -90

        #  Iterate over joints 1,2,4 and set to max if user wants a higher value
        for index, angle in enumerate(joint_degrees):
            if index == 0: continue
            if index == 2: # Handle joint 3 max
                if angle > 57:
                    print(f"Cannot rotate joint 3 more than 57 degrees")
                    joint_degrees[index] = 57
                    continue

            if angle > 90: # Handle joints 2,4 max
                print(f"Cannot rotate joint {index+1} more than 90 degrees")
                joint_degrees[index] = 90


        ### ----------------------------- Check combination limitations ----------------------------- ###
        # 12 >= 0.60  &&  13 >= 0.00  |  34.30 degrees  &&  00.00 degrees
        # 12 >= 0.35  &&  13 >= 0.00  &&  14 > 0.00   |  20.05 degrees  &&  00.00 degrees  && 00.00 degrees
        # 12 >= 0.00  &&  13 >= 0.80  |  00.00 degrees  &&  45.83 degrees
        # 12 >= 0.00  &&  13 >= 0.50  &&  14 > 0.60   |  00.00 degrees  &&  45.83 degrees  &&  34.30
        # 13 >= -1.0  &&  13 >= 1.20  |  -57.29 degrees  &&  68.75 degrees
            # If 14 >= 0.5 do as well  |  28.60 degrees

        # Prevent forward ground slam TODO
        if joint_degrees[1] >= 34.30 < joint_degrees[1] - joint_degrees[2]:
            print("Preventing Forward Ground Slam")
            joint_degrees[1] = joint_degrees[1] - (joint_degrees[1] - 34.30)

        # if (joint_degrees[1] >= 34.30 and joint_degrees[2] <= 00.00
        #         and joint_degrees[1] - joint_degrees[2] > 34.30):
        #     print("Preventing Forward Ground Slam II")
        #     joint_degrees[2] = joint_degrees[2] - (joint_degrees[1] - 34.3)

        # Prevent forward leaning wrist slam. May overwrite "Prevent forward ground slam" to allow wrist movement
        if joint_degrees[1] >= 15.00 and joint_degrees[3] > 00.00:
            print("Preventing Forward Leaning Wrist Slam")
            joint_degrees[1] = 20.05
            joint_degrees[2] = 00.00

        # # Prevent forward hip folding into ground
        # if joint_degrees[1] >= 0.00 and joint_degrees[2] >= 45.83:
        #     print("Preventing Forward Hip Folding")
        #     joint_degrees[1] = 0.00
        #     joint_degrees[2] = 45.83

        # Preventing Forward Hip Folding Wrist Slam
        if joint_degrees[1] >= 0.00 and joint_degrees[2] >= 45.83 and joint_degrees[3] >= 34.30:
            print("Preventing Forward Hip Folding Wrist Slam")
            joint_degrees[1] = 0.00
            joint_degrees[2] = 45.83
            joint_degrees[3] = 34.30

        # Prevent curling into self
        if joint_degrees[1] <= -57.29 and joint_degrees[2] >= 68.75:
            print("Preventing Curling Into Self")
            joint_degrees[1] = -57.29
            joint_degrees[2] = 68.75

            if joint_degrees[3] >= 0.5: # If arm is also tilted forward. Set max
                joint_degrees[3] = 0.5

        ### ----------------------------- Convert Degrees to Radians ----------------------------- ###
        joint_radians = [0.0] * 4
        for index, angle in enumerate(joint_degrees):
            radians = (angle * math.pi) / 180
            radians = round(radians, 3)
            joint_radians[index] = radians

        print(f"---------------------  Sending Degrees: {joint_degrees}")
        print(f"---------------------  Sending Radians: {joint_radians}")
        self.send_arm_cmd(joint_radians, 2.0) # Make duration dynamic


def main(args=None):

    def to_home():
        node.process_arm_movement([0, 0, 0, 0])

    rclpy.init(args=args)
    node = OpenManipulatorXControl()

    to_home()
    time.sleep(2)

    ### -------------- Absolute Limitations as Radian Inputs to Degrees Equivalent -------------- ###
    # 11: (-3.14 to 3.14) => -180 degrees to 180 degrees
    # 12: (-1.57 to 1.57) =>  -90 degrees to  90 degrees
    # 13: (-1.57 to 1.00) =>  -90 degrees to  57 degrees
    # 14: (-1.57 to 1.57) =>  -90 degrees to  90 degrees
    ### ----------------------------- Check combination limitations ----------------------------- ###
    # 12 >= 0.60  &&  13 >= 0.00  |  34.30 degrees  &&  00.00 degrees
    # 12 >= 0.35  &&  13 >= 0.00  &&  14 > 0.00   |  20.05 degrees  &&  00.00 degrees  && 00.00 degrees
    # 12 >= 0.00  &&  13 >= 0.80  |  00.00 degrees  &&  45.83 degrees
    # 12 >= 0.00  &&  13 >= 0.50  &&  14 > 0.60   |  00.00 degrees  &&  45.83 degrees  &&  34.30
    # 13 >= -1.0  &&  13 >= 1.20  |  -57.29 degrees  &&  68.75 degrees
    # If 14 >= 0.5 do as well  |  28.60 degrees

    node.process_arm_movement([90, 35, 20, 0])
    time.sleep(5)

    to_home()


    print("\n")


if __name__ == '__main__':
    main()