"""Refill the simulated battery (MAV_CMD_BATTERY_RESET).

    python -m scripts.battery_reset          # 100 %
    python -m scripts.battery_reset 50       # set to 50 %, e.g. to test low-battery behaviour
"""
import argparse
import sys

from pymavlink import mavutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument("percent", nargs="?", type=float, default=100.0)
    p.add_argument("--conn", default="tcp:127.0.0.1:14551")
    args = p.parse_args()

    m = mavutil.mavlink_connection(args.conn)
    if not m.wait_heartbeat(timeout=20):
        sys.exit("No heartbeat: is SITL running? (./sim.sh up)")
    # param1: battery mask (255 = all), param2: remaining percentage
    m.mav.command_long_send(m.target_system, 1, mavutil.mavlink.MAV_CMD_BATTERY_RESET, 0,
                            255, args.percent, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", condition="COMMAND_ACK.command==42651",
                       blocking=True, timeout=5)
    if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        sys.exit(f"Battery reset not accepted ({ack.result if ack else 'no reply'})")
    batt = m.recv_match(type="BATTERY_STATUS", blocking=True, timeout=5)
    print(f"Battery reset: {batt.battery_remaining if batt else args.percent:.0f} %")


if __name__ == "__main__":
    main()
