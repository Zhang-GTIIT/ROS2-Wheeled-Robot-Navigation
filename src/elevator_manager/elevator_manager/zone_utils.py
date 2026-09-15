import math

def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def in_waiting_zone(pose, zone):
    """
    pose: (x, y, yaw) in map frame
    zone: dict with keys:
          cx, cy, yaw, length, width, yaw_tol
    """
    x, y, yaw = pose

    dx = x - zone['cx']
    dy = y - zone['cy']

    cos_y = math.cos(zone['yaw'])
    sin_y = math.sin(zone['yaw'])

    # transform to zone local frame
    local_x =  cos_y * dx + sin_y * dy
    local_y = -sin_y * dx + cos_y * dy

    if abs(local_x) < zone['length'] / 2.0 and \
       abs(local_y) < zone['width'] / 2.0:

        if abs(angle_diff(yaw, zone['yaw'])) < zone['yaw_tol']:
            return True

    return False