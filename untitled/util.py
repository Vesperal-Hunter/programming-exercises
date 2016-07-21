from math import sqrt, degrees, atan2

def quaternion_to_radians(q):
    yaw = atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw

def quaternion_to_degrees(quat):
    return degrees(quaternion_to_radians(quat))