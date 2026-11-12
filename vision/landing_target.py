"""Detection -> MAVLink LANDING_TARGET, matching ArduPilot's AC_PrecLand_Companion decoding.

ArduPilot (angle mode, position_valid=0) rebuilds the body-frame line of sight as
    los_body = normalize(-tan(angle_y), tan(angle_x), 1)
i.e. angle_x is the image-right offset and angle_y the image-down offset, for a downward camera
with image-up = vehicle nose. `distance` (>0) is used as range to the target.
"""
import math

import numpy as np
from pymavlink import mavutil


def target_angles(tvec_cam):
    """Landing point in camera frame -> (angle_x, angle_y, distance)."""
    x, y, z = tvec_cam
    return math.atan2(x, z), math.atan2(y, z), float(np.linalg.norm(tvec_cam))


def ardupilot_los_body(angle_x, angle_y):
    """Mirror of AC_PrecLand_Companion::handle_msg (angle mode); used by tests."""
    v = np.array([-math.tan(angle_y), math.tan(angle_x), 1.0])
    return v / np.linalg.norm(v)


def send_landing_target(conn, det, time_usec):
    angle_x, angle_y, dist = target_angles(det.tvec)
    conn.mav.landing_target_send(
        time_usec, 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
        angle_x, angle_y, dist,
        0.0, 0.0,   # size_x/size_y: unused by ArduPilot
        type=mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        position_valid=0)
