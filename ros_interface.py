import math
import numpy as np
import time

# Ros related imports
import rclpy
from mercurial import node
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

### DEFINES (Meters) ###
# base_offset = 0.007 # Not used. We don't want to go under the base for safety.
base_length = 0.128
elbow_offset = 0.024
elbow_length = 0.124


def map_range_clamped(value, in_min, in_max, out_min, out_max):
    """ Clamp input value to the given range """
    value = max(min(value, in_max), in_min)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def kinemator(joint_angles_deg):
    """ Get the x,y position of the arm joints """
    ### Absolute Limitations as Radian Inputs to Degrees Equivalent ###
    # 11: (-3.14 to 3.14) => -180 degrees to 180 degrees (Base Rotation)
    # 12: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Base Tilt)
    # 13: (-1.57 to 1.00) =>  -90 degrees to  60 degrees (Elbow Tilt - Rotated 90 deg to right)
    # 14: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Wrist Tilt - Inline with Elbow Joint)

    # ---------------------- Correct degrees based on joint perspective && clamp to limits ----------------------
    joint_angles_deg[0] = map_range_clamped(joint_angles_deg[0], -180, 180, -180, 180) # Reverse if needed
    joint_angles_deg[1] = map_range_clamped(joint_angles_deg[1], 0, 180, -90, 90) + 90
    joint_angles_deg[2] = map_range_clamped(joint_angles_deg[2], -90, 60, 90, -60.0)
    joint_angles_deg[3] = map_range_clamped(joint_angles_deg[3], -180, 180, -180, 180)

    print(f"\nUpdated Angles DEG: {joint_angles_deg[0]}, {joint_angles_deg[1]}, {joint_angles_deg[2]}, {joint_angles_deg[3]}")

    # Convert Degrees to Radians
    joint_angles_deg[0] = round(math.radians(joint_angles_deg[0]), 3)
    joint_angles_deg[1] = round(math.radians(joint_angles_deg[1] - 90), 3)
    joint_angles_deg[2] = round(math.radians(joint_angles_deg[2]), 3)
    joint_angles_deg[3] = round(math.radians(joint_angles_deg[3]), 3)

    print(f"Updated Angles RAD: {joint_angles_deg[0]}, {joint_angles_deg[1]}, {joint_angles_deg[2]}, {joint_angles_deg[3]}")

    # --------------------------------- Calculate (x,y) position for each joint) ---------------------------------
    # 11 & 12 are static. The angles are used to calculate the position of the NEXT joint
    print(f"\nBase Rotation: {joint_angles_deg[0]}")

    elbow_x = round(base_length * math.cos(joint_angles_deg[1]), 3)
    elbow_y = round(base_length * math.sin(joint_angles_deg[1]), 3) * -1

    print(f"Elbow Position: {elbow_x}, {elbow_y}")

    wrist_x = round(elbow_length * math.cos(joint_angles_deg[2]) + elbow_x, 3)
    wrist_y = round(elbow_length * math.sin(joint_angles_deg[2]) + elbow_y, 3)

    print(f"Wrist Position: {wrist_x}, {wrist_y}")

    node.send_arm_cmd([
        joint_angles_deg[0],
        joint_angles_deg[1],
        joint_angles_deg[2],
        joint_angles_deg[3]
    ])


def forward_kinematics_dh(joint_angles_deg):
    # Extract joint angles and convert to radians
    θ0 = math.radians(11)  # offset angle
    θ1 = math.radians(joint_angles_deg[0])
    θ2 = math.radians(joint_angles_deg[1]) - θ0
    θ3 = math.radians(joint_angles_deg[2]) + θ0
    θ4 = math.radians(joint_angles_deg[3])

    # Link parameters
    d1 = 0.077
    a2 = 0.130
    a3 = 0.135
    a4 = 0.126

    def dh_transform(θ, α, a, d):
        α = math.radians(α)
        return np.array([
            [math.cos(θ), -math.sin(θ)*math.cos(α),  math.sin(θ)*math.sin(α), a*math.cos(θ)],
            [math.sin(θ),  math.cos(θ)*math.cos(α), -math.cos(θ)*math.sin(α), a*math.sin(θ)],
            [0,            math.sin(α),              math.cos(α),             d],
            [0,            0,                        0,                      1]
        ])

    T1 = dh_transform(θ1, 90, 0, d1)
    T2 = dh_transform(θ2, 0, a2, 0)
    T3 = dh_transform(θ3, 0, a3, 0)
    T4 = dh_transform(θ4, 0, a4, 0)

    T_final = T1 @ T2 @ T3 @ T4
    position = T_final[:3, 3]

    print(f"END-EFFECTOR POSITION (x, y, z): {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}")
    return position


class OpenManipulatorXControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_x_control')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.current_joint_radians = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Robot joins initialized")

    def compute_duration(self, target_radians):
        """Compute movement duration based on joint travel distance."""
        total_delta = sum(abs(a - b) for a, b in zip(self.current_joint_radians, target_radians))
        base_speed = 2.0  # default speed for average distance
        return max(1.0, min(5.0, base_speed * total_delta / math.pi))  # Clamp between 1s and 5s

    def send_arm_cmd(self, joint_radians, duration):
        robot_control = JointTrajectory()
        robot_control.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        point = JointTrajectoryPoint()
        point.positions = joint_radians
        point.time_from_start.sec = int(duration)
        robot_control.points.append(point)

        self.arm_pub.publish(robot_control)
        self.current_joint_radians = joint_radians.copy() # Store new state
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
        ### ----------------------------- Check combination limitations ----------------------------- ###
        # 12 >= 0.60  &&  13 >= 0.00  |  34.30 degrees  &&  00.00 degrees
        # 12 >= 0.35  &&  13 >= 0.00  &&  14 > 0.00   |  20.05 degrees  &&  00.00 degrees  && 00.00 degrees
        # 12 >= 0.00  &&  13 >= 0.80  |  00.00 degrees  &&  45.83 degrees
        # 12 >= 0.00  &&  13 >= 0.50  &&  14 > 0.60   |  00.00 degrees  &&  45.83 degrees  &&  34.30
        # 13 >= -1.0  &&  13 >= 1.20  |  -57.29 degrees  &&  68.75 degrees
        # If 14 >= 0.5 do as well  |  28.60 degrees

        # Clamp to absolute limits
        joint_degrees[0] = max(min(joint_degrees[0], 180), -180)
        joint_degrees[1] = max(min(joint_degrees[1], 90), -90)
        joint_degrees[2] = max(min(joint_degrees[2], 57), -90)
        joint_degrees[3] = max(min(joint_degrees[3], 90), -90)

        ### ----------------------------- Convert Degrees to Radians ----------------------------- ###
        target_radians = [round(math.radians(d), 3) for d in joint_degrees]
        duration = round(self.compute_duration(target_radians), 2)

        print(f"---------------------  Sending Degrees: {joint_degrees} over {duration} second(s)")
        # print(f"---------------------  Sending Radians: {joint_radians} over {duration} second(s)")
        self.send_arm_cmd(target_radians, duration) # Make duration dynamic
        print(f"Sleeping for {duration + 1}s")
        time.sleep(duration + 1)


def main(args=None):
    rclpy.init(args=args)
    node = OpenManipulatorXControl()

    def to_home():
        node.process_arm_movement([0, 0, 0, 0])

    # Need to start 1 movement first before actually moving
    print("To Home")
    to_home()
    print("\n\n")

    print("Movement 1")
    node.process_arm_movement([0, 90, -90, 0])
    print("\n\n")

    print("Movement 2")
    current_move = [0, -90, 0, 0]
    node.process_arm_movement(current_move)
    print("\n\n")

    print("To Home")
    to_home()
    print("\n\n")

if __name__ == '__main__':
    main()