import enum
import math

### DEFINES (Meters) ###
# base_offset = 0.007 # Not used. We don't want to go under the base for safety.
base_length = 0.128
elbow_offset = 0.024
elbow_length = 0.124


# class Joint(enum.Enum):
#     """ Define the different joints and their lengths """
#     ankle    = 2
#     hip      = 2
#     shoulder = 2
#
#     @property
#     def length(self):
#         """ Return the length of the arm after the joint """
#         return self.value
#
#
# def get_next_location(joint: Joint, angle_deg, x_offset, y_offset):
#     """Get the Next Location based on angle and length"""
#     local_x = joint.length * math.cos(math.radians(angle_deg))
#     local_y = joint.length * math.sin(math.radians(angle_deg))
#
#     updated_x = round(local_x + x_offset, 3)
#     updated_y = round(local_y + y_offset, 3)
#
#     print(f"Current (X, Y): {updated_x}, {updated_y}")
#     return updated_x, updated_y

def map_range_clamped(value, in_min, in_max, out_min, out_max):
    """ Clamp input value to the given range """
    value = max(min(value, in_max), in_min)
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def kinemator(joint_angles_deg):
    """ Get the x,y position of the arm joints """
    print(f"\nInput Angles DEG: {joint_angles_deg[0]}, {joint_angles_deg[1]}, {joint_angles_deg[2]}, {joint_angles_deg[3]}")
    ### Absolute Limitations as Radian Inputs to Degrees Equivalent ###
    # 11: (-3.14 to 3.14) => -180 degrees to 180 degrees (Base Rotation)
    # 12: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Base Tilt)
    # 13: (-1.57 to 1.00) =>  -90 degrees to  60 degrees (Elbow Tilt - Rotated 90 deg to right)
    # 14: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Wrist Tilt - Inline with Elbow Joint)

    # ---------------------- Clamp degrees to joint limits && Find degrees in natural POV for ROS ----------------------
    joint_angles_deg[0] = map_range_clamped(joint_angles_deg[0], -180, 180, -180, 180) # Reverse if needed
    joint_angles_deg[1] = map_range_clamped(joint_angles_deg[1], 0, 180, 90, -90)
    joint_angles_deg[2] = map_range_clamped(joint_angles_deg[2], 60, 180, 40, -90)
    joint_angles_deg[3] = map_range_clamped(joint_angles_deg[3], -90, 90, -90, 90)

    print(f"Natural Angles DEG: {joint_angles_deg[0]}, {joint_angles_deg[1]}, {joint_angles_deg[2]}, {joint_angles_deg[3]}")

    # Convert Degrees to Radians
    joint_angles_deg[0] = round(math.radians(joint_angles_deg[0]), 3)
    joint_angles_deg[1] = round(math.radians(joint_angles_deg[1] - 90), 3)
    joint_angles_deg[2] = round(math.radians(joint_angles_deg[2]), 3)
    joint_angles_deg[3] = round(math.radians(joint_angles_deg[3]), 3)

    print(f"Natural Angles RAD: {joint_angles_deg[0]}, {joint_angles_deg[1]}, {joint_angles_deg[2]}, {joint_angles_deg[3]}")

    # --------------------------------- Updated Angles Relative to other joints ---------------------------------
    # 11 stays the same
    # 12 stays the same (natural POV)
    joint_angles_deg[2]

    # --------------------------------- Calculate (x,y) position for each joint) ---------------------------------
    # 11 & 12 are static. The angles are used to calculate the position of the NEXT joint
    print(f"\nBase Rotation: {joint_angles_deg[0]}")

    elbow_x = round(base_length * math.cos(joint_angles_deg[1]), 3)
    elbow_y = round(base_length * math.sin(joint_angles_deg[1]), 3) * -1

    print(f"Elbow Position: {elbow_x}, {elbow_y}")

    wrist_x = round(elbow_length * math.cos(joint_angles_deg[2]) + elbow_x, 3)
    wrist_y = round(elbow_length * math.sin(joint_angles_deg[2]) + elbow_y, 3)

    print(f"Wrist Position: {wrist_x}, {wrist_y}")



if __name__ == "__main__":
    ### Absolute Limitations as Radian Inputs to Degrees Equivalent ###
    # 11: (-3.14 to 3.14) => -180 degrees to 180 degrees (Base Rotation)
    # 12: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Base Tilt)
    # 13: (-1.57 to 1.00) =>  -90 degrees to  60 degrees (Elbow Tilt - Rotated 90 deg to right)
    # 14: (-1.57 to 1.57) =>  -90 degrees to  90 degrees (Wrist Tilt - Inline with Elbow Joint)

    # print(map_range_clamped(0, -90, 90, 180, 0))
    # print(map_range_clamped(0, -90, 60, 90, -60))
    # print(map_range_clamped(0, -90, 90, 180, 0))

    kinemator([0, 0, 90, 0])

    # x2, y2 = get_next_location(Joint.ankle, 45, 0, 0)
    # x3, y3 = get_next_location(Joint.hip, 60, x2, y2)
    # x4, y4 = get_next_location(Joint.shoulder, 0, x3, y3)