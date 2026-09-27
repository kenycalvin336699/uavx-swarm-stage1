#!/usr/bin/env python3
"""
disaster_response_demo.py

Stage 1 proof-of-concept for the UAV-X Resilient BVLOS Swarm Challenge.

Demonstrates:
  1. Priority-weighted PoI survey (P1 life-critical, P2 infrastructure, P3 situational)
  2. Dedicated relay UAV maintaining communication to GCS
  3. Quality-driven relay repositioning when link degrades
  4. Autonomous return-to-home when battery drops below threshold
  5. Structured decision logging for reproducibility

SITL setup (three terminals):
  sim_vehicle.py -v ArduCopter -I0 --sysid 1 --console \
      --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
  sim_vehicle.py -v ArduCopter -I1 --sysid 2 --console \
      --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14561
  sim_vehicle.py -v ArduCopter -I2 --sysid 3 --console \
      --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14571
"""

import math
import time
import threading
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CONNECTIONS = {
    1: "udpin:127.0.0.1:14551",   # relay drone
    2: "udpin:127.0.0.1:14561",   # survey drone A
    3: "udpin:127.0.0.1:14571",   # survey drone B
}

RELAY_DRONE_ID = 1
SURVEY_DRONE_IDS = [2, 3]

TAKEOFF_ALT = 25.0
CRUISE_SPEED = 10.0              # m/s
WAYPOINT_TOLERANCE = 8.0         # metres
WAYPOINT_TIMEOUT = 180           # seconds
HEARTBEAT_TIMEOUT = 30
TAKEOFF_TIMEOUT = 30
BATTERY_RTH_THRESHOLD = 25       # percent

# GCS is at local origin (0, 0). All drones start near origin.
GCS_POSITION = (0.0, 0.0)

# Link-quality model
LINK_DIRECT_RANGE = 800.0        # metres; beyond this, PDR drops
LINK_QUALITY_THRESHOLD = 0.85    # below this triggers relay repositioning

# PoIs: (id, priority, north_m, east_m, description)
POIS = [
    ("P1-A", "P1",  600.0,  400.0, "Reported survivor location"),
    ("P1-B", "P1",  900.0,  300.0, "Active landslide zone"),
    ("P2-A", "P2",  500.0,  900.0, "Blocked evacuation route"),
    ("P2-B", "P2",  700.0,  700.0, "Damaged bridge"),
    ("P3-A", "P3", 1200.0,  200.0, "General area mapping"),
    ("P3-B", "P3",  400.0, 1100.0, "Crop damage assessment"),
]

# Lock for clean interleaved logging from multiple threads
LOG_LOCK = threading.Lock()


def log(tag, message):
    """Thread-safe timestamped log."""
    ts = time.strftime("%H:%M:%S")
    with LOG_LOCK:
        print(f"[{ts}] [{tag}] {message}", flush=True)


# ---------------------------------------------------------------------------
# Link-quality model (simplified — Stage 2 will use ns-3 / FlyNetSim)
# ---------------------------------------------------------------------------

def estimate_link_quality(position_ned):
    """Return simulated PDR between a position and the GCS.
    PDR = 1.0 within LINK_DIRECT_RANGE, decays beyond it, plus mild noise.
    """
    n, e = position_ned[0], position_ned[1]
    dist = math.hypot(n - GCS_POSITION[0], e - GCS_POSITION[1])
    if dist <= LINK_DIRECT_RANGE:
        pdr = 1.0
    else:
        # linear decay from 1.0 to 0.3 over the next 800 m
        excess = dist - LINK_DIRECT_RANGE
        pdr = max(0.30, 1.0 - (excess / 800.0))
    return pdr


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def connect_vehicle(conn_str, label):
    log(label, f"Connecting via {conn_str} ...")
    vehicle = mavutil.mavlink_connection(conn_str, source_system=255)
    hb = vehicle.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        raise RuntimeError(f"[{label}] No heartbeat within {HEARTBEAT_TIMEOUT}s")
    log(label, f"Heartbeat. sys={vehicle.target_system}")
    return vehicle


def set_param(vehicle, label, name, value):
    vehicle.mav.param_set_send(
        vehicle.target_system,
        vehicle.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    time.sleep(0.2)


def set_mode(vehicle, label, mode_name, timeout=5):
    mode_id = vehicle.mode_mapping()[mode_name]
    vehicle.set_mode(mode_id)
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == mode_id:
            log(label, f"Mode -> {mode_name}")
            return True
    log(label, f"WARNING: mode change to {mode_name} not confirmed")
    return False


def arm(vehicle, label, timeout=15):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log(label, "Armed.")
            return True
    log(label, "ERROR: arming not confirmed")
    return False


def takeoff(vehicle, label, altitude, timeout=TAKEOFF_TIMEOUT):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude,
    )
    log(label, f"Takeoff -> {altitude} m")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = vehicle.recv_match(type="GLOBAL_POSITION_INT",
                                 blocking=True, timeout=1)
        if msg is None:
            continue
        rel_alt = msg.relative_alt / 1000.0
        if rel_alt >= altitude * 0.9:
            log(label, f"Reached {rel_alt:.1f} m")
            return True
    log(label, "ERROR: takeoff altitude not reached")
    return False


def goto_local_ned(vehicle, north, east, down):
    type_mask = 0b0000111111111000  # position only
    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north, east, down,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )


def latest_local_position(vehicle):
    pos = None
    for _ in range(10):
        msg = vehicle.recv_match(type="LOCAL_POSITION_NED",
                                 blocking=True, timeout=0.2)
        if msg is None:
            break
        pos = (msg.x, msg.y, msg.z)
    return pos


def latest_battery(vehicle):
    msg = vehicle.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
    if msg is None:
        return None
    return msg.battery_remaining


def fly_to_waypoint(vehicle, label, north, east, down,
                    timeout=WAYPOINT_TIMEOUT, tolerance=WAYPOINT_TOLERANCE):
    goto_local_ned(vehicle, north, east, down)
    start = time.time()
    last_report = start
    while time.time() - start < timeout:
        pos = latest_local_position(vehicle)
        if pos is not None:
            n, e, _ = pos
            dist = math.hypot(n - north, e - east)
            now = time.time()
            if now - last_report > 5:
                log(label, f"{dist:6.1f} m to go")
                last_report = now
            if dist <= tolerance:
                log(label, f"Reached ({north:.0f}, {east:.0f})")
                return True
        time.sleep(0.1)
    log(label, f"Timeout approaching ({north:.0f}, {east:.0f})")
    return False


# ---------------------------------------------------------------------------
# Relay and survey drone behaviors
# ---------------------------------------------------------------------------

def relay_worker(conn_str):
    """Relay drone: flies to midpoint between GCS and survey centroid,
    then repositions when link quality drops.
    """
    label = "RELAY"
    try:
        vehicle = connect_vehicle(conn_str, label)
        set_param(vehicle, label, "ARMING_CHECK", 0)

        if not set_mode(vehicle, label, "GUIDED"):
            return
        if not arm(vehicle, label):
            return
        if not takeoff(vehicle, label, TAKEOFF_ALT):
            return

        down = -TAKEOFF_ALT

        # Initial relay position: midpoint between GCS and survey centroid
        survey_n = sum(p[2] for p in POIS) / len(POIS)
        survey_e = sum(p[3] for p in POIS) / len(POIS)
        relay_n = (GCS_POSITION[0] + survey_n) / 2.0
        relay_e = (GCS_POSITION[1] + survey_e) / 2.0

        log(label, f"Initial position: ({relay_n:.0f}, {relay_e:.0f})")
        fly_to_waypoint(vehicle, label, relay_n, relay_e, down)

        # Monitor loop: reposition if link quality degrades
        while True:
            pos = latest_local_position(vehicle)
            if pos is None:
                time.sleep(1)
                continue

            pdr = estimate_link_quality(pos)
            log(label, f"Link PDR to GCS = {pdr:.2f}")

            if pdr < LINK_QUALITY_THRESHOLD:
                log(label, "PDR below threshold — repositioning")
                # In Stage 2 this would search candidate positions.
                # For the demo, we simply nudge toward the survey centroid.
                new_n = pos[0] + (survey_n - pos[0]) * 0.15
                new_e = pos[1] + (survey_e - pos[1]) * 0.15
                fly_to_waypoint(vehicle, label, new_n, new_e, down)

            battery = latest_battery(vehicle)
            if battery is not None and battery < BATTERY_RTH_THRESHOLD:
                log(label, f"Battery {battery}% below threshold — RTL")
                set_mode(vehicle, label, "RTL")
                return

            time.sleep(3)

    except Exception as exc:
        log(label, f"FAILED: {exc}")


def survey_worker(conn_str, drone_id, poi_subset):
    """Survey drone: visits assigned PoIs in priority order."""
    label = f"SURVEY-{drone_id}"
    try:
        vehicle = connect_vehicle(conn_str, label)
        set_param(vehicle, label, "ARMING_CHECK", 0)

        if not set_mode(vehicle, label, "GUIDED"):
            return
        if not arm(vehicle, label):
            return
        if not takeoff(vehicle, label, TAKEOFF_ALT):
            return

        down = -TAKEOFF_ALT
        surveyed = []

        for poi_id, priority, north, east, desc in poi_subset:
            log(label, f"Next task: {poi_id} [{priority}] — {desc}")
            reached = fly_to_waypoint(vehicle, label, north, east, down)

            if reached:
                # Simulate sensor capture
                log(label, f"Captured imagery at {poi_id} (hover 3 s)")
                time.sleep(3)
                surveyed.append(poi_id)

                # Check link quality at this position
                pdr = estimate_link_quality((north, east))
                log(label, f"Link PDR at {poi_id} = {pdr:.2f}")
                if pdr < LINK_QUALITY_THRESHOLD:
                    log(label, f"Warning: link degraded at {poi_id}")
            else:
                log(label, f"Skipping {poi_id} after timeout")

            battery = latest_battery(vehicle)
            if battery is not None and battery < BATTERY_RTH_THRESHOLD:
                log(label, f"Battery {battery}% — returning home")
                set_mode(vehicle, label, "RTL")
                break

        log(label, f"Survey complete. Visited: {surveyed}")

    except Exception as exc:
        log(label, f"FAILED: {exc}")


# ---------------------------------------------------------------------------
# PoI distribution across survey drones
# ---------------------------------------------------------------------------

def split_pois_by_priority(pois, num_drones):
    """Distribute PoIs across drones, keeping priority order.
    P1s go first, then P2s, then P3s — round-robin across drones.
    """
    priority_order = {"P1": 0, "P2": 1, "P3": 2}
    sorted_pois = sorted(pois, key=lambda p: (priority_order[p[1]], p[0]))

    assignments = [[] for _ in range(num_drones)]
    for idx, poi in enumerate(sorted_pois):
        assignments[idx % num_drones].append(poi)
    return assignments


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    log("MAIN", "UAV-X Disaster Response Demo — Stage 1 proof-of-concept")
    log("MAIN", f"PoIs: {len(POIS)} "
                f"(P1={sum(1 for p in POIS if p[1]=='P1')}, "
                f"P2={sum(1 for p in POIS if p[1]=='P2')}, "
                f"P3={sum(1 for p in POIS if p[1]=='P3')})")

    assignments = split_pois_by_priority(POIS, len(SURVEY_DRONE_IDS))
    for drone_id, pois in zip(SURVEY_DRONE_IDS, assignments):
        log("MAIN", f"Drone {drone_id} assigned: "
                    f"{[p[0] for p in pois]}")

    threads = []

    # Relay drone
    t_relay = threading.Thread(
        target=relay_worker,
        args=(CONNECTIONS[RELAY_DRONE_ID],),
        name="RELAY",
        daemon=True,
    )
    threads.append(t_relay)

    # Survey drones
    for drone_id, pois in zip(SURVEY_DRONE_IDS, assignments):
        t = threading.Thread(
            target=survey_worker,
            args=(CONNECTIONS[drone_id], drone_id, pois),
            name=f"SURVEY-{drone_id}",
            daemon=True,
        )
        threads.append(t)

    for t in threads:
        t.start()
        time.sleep(1)

    # Relay runs forever (monitoring loop), so only join survey threads
    for t in threads:
        if t.name.startswith("SURVEY"):
            t.join()

    log("MAIN", "All survey drones complete. Relay holding position.")
    log("MAIN", "Press Ctrl+C to stop the relay monitoring loop.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
