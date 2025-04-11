import enum
import math


class Joint(enum.Enum):
    """ Define the different joints and their lengths """
    ankle    = 2
    hip      = 2
    shoulder = 2

    @property
    def length(self):
        """ Return the length of the arm after the joint """
        return self.value


def get_next_location(joint: Joint, angle_deg, x_offset, y_offset):
    """Get the Next Location based on angle and length"""
    local_x = joint.length * math.cos(math.radians(angle_deg))
    local_y = joint.length * math.sin(math.radians(angle_deg))

    updated_x = round(local_x + x_offset, 3)
    updated_y = round(local_y + y_offset, 3)

    print(f"Current (X, Y): {updated_x}, {updated_y}")
    return updated_x, updated_y


if __name__ == "__main__":
    x2, y2 = get_next_location(Joint.ankle, 45, 0, 0)
    x3, y3 = get_next_location(Joint.hip, 60, x2, y2)
    x4, y4 = get_next_location(Joint.shoulder, 0, x3, y3)