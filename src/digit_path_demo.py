#!/usr/bin/env python3
"""digit_path_demo.py — Multi-drone waypoint control demo."""

import time
import threading
from pymavlink import mavutil

CONNECTIONS = {
    "1": "udpin:127.0.0.1:14551",
    "2": "udpin:127.0.0.1:14561",
    "3": "udpin:127.0.0.1:14571",
}

TAKEOFF_ALT = 10.0
TOLERANCE = 2.0

PATHS = {
    "1": [(0, 3), (10, 3)],
    "2": [(10, 0), (10, 6), (6, 6), (4, 0), (0, 0), (0, 6)],
    "3": [(10, 0), (10, 6), (5, 6), (5, 3), (5, 6), (0, 6), (0, 0)],
}


def connect(conn_str, label):
    print(f"[{label}] Connecting via {conn_str}")
    v = mavutil.mavlink_connection(conn_str, source_system=255)
    if v.wait_heartbeat(timeout=30) is None:
        raise RuntimeError(f"[{label}] No heartbeat")
    print(f"[{label}] Heartbeat sys={v.target_system}")
    return v


def set_mode(v, name):
    v.set_mode(v.mode_mapping()[name])
    time.sleep(1)


def arm(v):
    v.mav.command_long_send(v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(2)


def takeoff(v, alt):
    v.mav.command_long_send(v.target_system, v.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    time.sleep(8)


def goto(v, n, e, d):
    v.mav.set_position_target_local_ned_send(0,
        v.target_system, v.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)


def pos(v):
    m = v.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    return (m.x, m.y, m.z) if m else None


def fly_to(v, n, e, d):
    goto(v, n, e, d)
    t0 = time.time()
    while time.time() - t0 < 15:
        p = pos(v)
        if p and ((p[0]-n)**2 + (p[1]-e)**2) ** 0.5 < TOLERANCE:
            return True
        time.sleep(0.2)
    return False


def fly_digit(digit, conn_str):
    label = f"UAV-{digit}"
    try:
        v = connect(conn_str, label)
        v.mav.param_set_send(v.target_system, v.target_component,
            b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        set_mode(v, "GUIDED")
        arm(v)
        takeoff(v, TAKEOFF_ALT)
        d = -TAKEOFF_ALT
        for n, e in PATHS[digit]:
            fly_to(v, n, e, d)
            print(f"[{label}] waypoint ({n},{e}) reached")
        print(f"[{label}] path complete.")
    except Exception as exc:
        print(f"[{label}] FAILED: {exc}")


def main():
    threads = [threading.Thread(target=fly_digit, args=(d, c), daemon=True)
               for d, c in CONNECTIONS.items()]
    for t in threads:
        t.start()
        time.sleep(1)
    for t in threads:
        t.join()
    print("All drones finished.")


if __name__ == "__main__":
    main()
