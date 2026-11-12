"""Closed-loop precision-landing run against SITL, scored and logged.

takeoff -> fly to the deck with GPS-level error -> LAND (ArduPilot precland uses our
LANDING_TARGET) -> score touchdown vs pad centre -> append a row to logs/landings.csv.

    python -m scripts.land_mission --scenario sim/scenarios/moderate.yaml --stabilised
"""
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from pymavlink import mavutil

from sim.camera import DeckCamera
from sim.deck import DeckMotion
from vision.detector import PadDetector
from vision.landing_target import send_landing_target
from vision.pad import Camera, Pad

MSG_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 50,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 50,
    mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5,
}


class Vehicle:
    def __init__(self, conn_str):
        self.m = mavutil.mavlink_connection(conn_str, source_component=191)
        self.m.wait_heartbeat(timeout=30)
        self.att = self.pos = None
        self.boot_ms = 0
        self.landed_state = 0
        self.armed = False
        self.statustext = []
        for msg_id, hz in MSG_RATES_HZ.items():
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def pump(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "ATTITUDE":
                self.att = msg
                self.boot_ms = msg.time_boot_ms
            elif t == "LOCAL_POSITION_NED":
                self.pos = msg
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system \
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "STATUSTEXT":
                self.statustext.append(msg.text)
                print(f"  [AP] {msg.text}")

    @property
    def t(self):
        return self.boot_ms / 1000.0

    @property
    def alt(self):
        return -self.pos.z if self.pos else 0.0

    def goto(self, n, e, alt):
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            n, e, -alt, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="tcp:127.0.0.1:14551")
    p.add_argument("--scenario", default="sim/scenarios/calm.yaml")
    p.add_argument("--stabilised", action="store_true", help="2-DOF platform on")
    p.add_argument("--stab-gain", type=float, default=0.85,
                   help="fraction of deck roll/pitch the platform cancels")
    p.add_argument("--alt", type=float, default=8.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--rate", type=float, default=15.0, help="camera frame rate (sim time)")
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--show", action="store_true")
    p.add_argument("--log", default="logs/landings.csv")
    args = p.parse_args()

    cam = Camera.load("config/cam.json")
    pad = Pad.load("config/pad.json")
    deck = DeckMotion.load(args.scenario, args.stab_gain if args.stabilised else 0.0)
    camera = DeckCamera(cam, pad, seed=args.seed)
    detector = PadDetector(cam, pad)
    rng = np.random.default_rng(args.seed)

    v = Vehicle(args.conn)
    print(f"Connected. Scenario={deck.name} stabilised={args.stabilised}")
    while v.pos is None or v.att is None:
        v.pump()
        time.sleep(0.01)

    # --- arm + takeoff
    t_wall = time.time()
    while not v.armed:
        if time.time() - t_wall > 90:
            sys.exit("Failed to arm")
        v.m.set_mode("GUIDED")
        v.m.arducopter_arm()
        end = time.time() + 2
        while time.time() < end and not v.armed:
            v.pump()
            time.sleep(0.02)
    print("Armed")
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt)

    phase = "takeoff"
    t_start = v.t
    next_frame = 0.0
    approach_n = approach_e = 0.0
    frames = dets = 0
    first_det_alt = None
    vz_touch = 0.0
    result = "timeout"
    touch = None

    while True:
        v.pump()
        if v.t - t_start > args.timeout:
            break

        # --- camera + vision at fixed sim-time rate
        if v.t >= next_frame and v.pos and v.att:
            next_frame = v.t + 1.0 / args.rate
            deck_pos, deck_rpy = deck.pose(v.t)
            img = camera.render([v.pos.x, v.pos.y, v.pos.z],
                                (v.att.roll, v.att.pitch, v.att.yaw), deck_pos, deck_rpy)
            det = detector.detect(img)
            frames += 1
            if det:
                dets += 1
                if first_det_alt is None:
                    first_det_alt = v.alt
                    print(f"  first detection at {v.alt:.1f} m")
                send_landing_target(v.m, det, v.boot_ms * 1000)
            if args.show:
                vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                if det:
                    cv2.aruco.drawDetectedMarkers(vis, det.corners)
                cv2.putText(vis, f"{phase} alt {v.alt:.1f}m", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("deck camera", vis)
                cv2.waitKey(1)

        # --- mission state machine
        if phase == "takeoff" and v.alt > 0.95 * args.alt:
            ang = rng.uniform(0, 2 * math.pi)
            err = deck.s.get("approach_error_m", 1.5)
            approach_n = deck.n0 + err * math.cos(ang)
            approach_e = deck.e0 + err * math.sin(ang)
            print(f"Approaching deck (aim off by {err:.1f} m)")
            phase = "approach"
        elif phase == "approach":
            v.goto(approach_n, approach_e, args.alt)
            if math.hypot(v.pos.x - approach_n, v.pos.y - approach_e) < 0.4 \
                    and math.hypot(v.pos.vx, v.pos.vy) < 0.3:
                print("Over deck, switching to LAND")
                v.m.set_mode("LAND")
                phase = "land"
        elif phase == "land":
            if v.alt < 0.3:
                vz_touch = max(vz_touch, v.pos.vz)
            if v.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND or not v.armed:
                deck_pos, _ = deck.pose(v.t)
                touch = (v.pos.x - deck_pos[0], v.pos.y - deck_pos[1], deck.tilt_deg(v.t))
                result = "landed"
                break
        time.sleep(0.002)

    if args.show:
        cv2.destroyAllWindows()

    err_n, err_e, tilt = touch if touch else (float("nan"),) * 3
    err = math.hypot(err_n, err_e)
    plnd_msgs = [s for s in v.statustext if "PrecLand" in s or "Plnd" in s]
    row = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "scenario": deck.name,
        "stabilised": int(args.stabilised),
        "seed": args.seed,
        "result": result,
        "error_m": round(err, 3),
        "err_n": round(err_n, 3),
        "err_e": round(err_e, 3),
        "vz_touch": round(vz_touch, 2),
        "deck_tilt_deg": round(tilt, 1),
        "first_det_alt_m": round(first_det_alt, 1) if first_det_alt else "",
        "det_rate": round(dets / frames, 2) if frames else 0,
        "duration_s": round(v.t - t_start, 1),
        "precland_msgs": " | ".join(plnd_msgs),
    }
    print("\nResult:")
    for k, val in row.items():
        print(f"  {k:16s} {val}")

    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    new = not os.path.exists(args.log)
    with open(args.log, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    sys.exit(0 if result == "landed" else 1)


if __name__ == "__main__":
    main()
