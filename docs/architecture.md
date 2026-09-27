# System Architecture

## Three-Tier Design

### Tier 1 — GCS Layer (outside disaster zone)

Assigns mission region and PoI priority list (P1/P2/P3).
Monitors end-to-end Packet Delivery Ratio for each Survey UAV.
Makes role decisions when the link to the swarm is healthy.

### Tier 2 — Relay Layer (midpoint region)

Maintains the communication chain between the Survey Layer and the GCS.
Repositions autonomously when link quality degrades.
Acts as Relay Coordinator if the GCS link is lost.

### Tier 3 — Survey Layer (disaster zone)

Executes PoI inspection. Each UAV is assigned a sub-region and
autonomously plans its path based on battery state, sensor footprint,
and PoI priorities.

## Implementation Status (Stage 1)

- Tier 3: implemented in `src/disaster_response_demo.py`
- Tier 2: implemented as single relay with greedy repositioning
- Tier 1: static role assignment; dynamic role management planned for Stage 2

## Role State Machine

Survey  → Relay    when link quality drops below 85% PDR
Relay   → Survey   when link quality exceeds 95% PDR for 10 seconds
Relay   → Standby  when mission completes
Standby → Relay    when current Relay fails or battery drops below 25%

## Data Flow

- GCS → Relay → Survey: commands, priority lists, role swaps
- Survey → Relay → GCS: telemetry, imagery, link quality reports
- Survey ↔ Survey: position broadcasts for collision avoidance

## Link Quality Measurement

Measured at every hop:
- GCS ↔ Relay: end-to-end PDR, RSSI
- Relay ↔ Survey: per-UAV PDR, RSSI
- Survey ↔ Survey: inter-UAV distance and collision risk

All measurements broadcast to the swarm for a shared link-health picture.

## Stage 2 Extensions

- Replace linear link-quality model with FlyNetSim (ArduPilot SITL + ns-3)
- Implement quality-driven candidate search for relay placement
- Add dynamic role management from Relay Coordinator when GCS link lost
- Add failure injection for fault-recovery testing
