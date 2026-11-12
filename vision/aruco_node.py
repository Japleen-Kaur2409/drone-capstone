"""Companion-computer vision node: real camera -> ArUco pad -> LANDING_TARGET.

In simulation the same detector runs inside scripts/land_mission.py against the simulated camera;
this script is for the real drone (or bench testing with a webcam).

    python -m vision.aruco_node --conn /dev/serial0 --baud 921600 --camera 0
"""
import argparse
import time

import cv2
from pymavlink import mavutil

from vision.detector import PadDetector
from vision.landing_target import send_landing_target, target_angles
from vision.pad import Camera, Pad


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="tcp:127.0.0.1:14551")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--camera", default="0", help="cv2.VideoCapture index or path")
    p.add_argument("--cam-config", default="config/cam.json")
    p.add_argument("--pad", default="config/pad.json")
    p.add_argument("--rate", type=float, default=15.0)
    p.add_argument("--show", action="store_true")
    args = p.parse_args()

    cam = Camera.load(args.cam_config)
    detector = PadDetector(cam, Pad.load(args.pad))
    conn = mavutil.mavlink_connection(args.conn, baud=args.baud, source_component=191)
    conn.wait_heartbeat(timeout=30)
    print(f"Connected to sys {conn.target_system}")

    src = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.height)

    period = 1.0 / args.rate
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            time.sleep(0.5)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        det = detector.detect(gray)
        if det:
            send_landing_target(conn, det, int(t0 * 1e6))
            ax, ay, d = target_angles(det.tvec)
            print(f"ids={det.ids} dist={d:5.2f} m ax={ax:+.3f} ay={ay:+.3f} err={det.reproj_px:.2f}px")
        if args.show:
            if det:
                cv2.aruco.drawDetectedMarkers(frame, det.corners)
            cv2.imshow("aruco", frame)
            if cv2.waitKey(1) == 27:
                break
        time.sleep(max(0.0, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
