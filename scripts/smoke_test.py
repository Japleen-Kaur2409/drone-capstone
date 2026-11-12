"""SITL smoke test: connect, arm in GUIDED, take off, hover, land."""
import argparse
import sys
import time

from pymavlink import mavutil


def wait_alt(m, target, tol=0.5, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f"  alt {alt:5.1f} m")
            if abs(alt - target) < tol:
                return True
    return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="tcp:127.0.0.1:14551")
    p.add_argument("--alt", type=float, default=10.0)
    args = p.parse_args()

    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat(timeout=30)
    print(f"Heartbeat from sys {m.target_system} comp {m.target_component}")

    # Wait for EKF / GPS so arming checks pass
    print("Waiting for vehicle to be armable...")
    t0 = time.time()
    while time.time() - t0 < 90:
        m.set_mode("GUIDED")
        m.arducopter_arm()
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            break
        time.sleep(2)
    else:
        sys.exit("Failed to arm")
    print("Armed in GUIDED")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, args.alt)
    if not wait_alt(m, args.alt):
        sys.exit("Takeoff did not reach target altitude")
    print("Reached altitude, hovering 3 s")
    time.sleep(3)

    m.set_mode("LAND")
    print("Landing...")
    m.motors_disarmed_wait()
    print("Landed and disarmed. SITL OK.")


if __name__ == "__main__":
    main()
