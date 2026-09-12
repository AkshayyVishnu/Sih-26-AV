# Stateflow Decision Logic Chart Specification (Phase 2.4)

## Overview
This document specifies the **DecisionLogic** Stateflow chart that handles decision-making in the Plan A MATLAB/Simulink pipeline. The chart implements the architecture.md design for adaptive path planning on unstructured Indian roads.

## Chart Structure

### Top Level: Two Parallel (AND) Regions
The chart has two parallel independent regions that execute simultaneously:

```
DecisionLogic (chart)
├── Region 1: DRIVE_MODE (path adaptation logic)
└── Region 2: SAFETY_SUPERVISOR (fault monitoring watchdog)
```

---

## Region 1: DRIVE_MODE (Path Adaptation)

### States
| State | Purpose | Initial | Entry Action |
|-------|---------|---------|--------------|
| **NORMAL_DRIVE** | Baseline operation, pure pursuit control | ✓ Yes | `emergency_brake_active = false; replan_requested = false;` |
| **OBSTACLE_DETECTED** | Obstacle confirmed, decide action | No | (See obstacle action table below) |

### Obstacle Decision Logic (inside OBSTACLE_DETECTED entry)
When entering OBSTACLE_DETECTED, execute this truth table:

| Class ID | Confidence | TTC | Action |
|----------|------------|-----|--------|
| 1 (animal) | any | > 1.5s | **Animal caution**: `animal_caution_active=true; replan_requested=true; emergency_brake_active=false;` |
| any | any | < 1.0s | **Emergency**: `emergency_brake_active=true; replan_requested=true;` |
| any | any | 1.0-1.5s | **Replan**: `replan_requested=true; emergency_brake_active=false;` |
| -1 (none) | 0 | inf | (No action) |

### Transitions

#### NORMAL_DRIVE → OBSTACLE_DETECTED
- **Guard**: `nearest_dist < 15 && nearest_ttc < 3 && nearest_conf > 0.6`
- **Debounce**: Fire this guard for 0.3 seconds before actually transitioning (prevents chattering on sensor noise)
- **Implementation**: Use a duration condition or a counter in a local state variable

#### OBSTACLE_DETECTED → NORMAL_DRIVE
- **Guard**: `(nearest_dist > 20 || nearest_ttc > 5) && time_in_state > 2.0`
- **Rationale**: Obstacle must be gone AND we must stay clear for 2 seconds (hysteresis to prevent resume chattering)
- **Submode memory**: On resume, return to the last sub-mode (LANE_KEEP vs OVERTAKE_IN_PROGRESS) via history junction
  - Track via state variable: `last_submode` (0=LANE_KEEP, 1=OVERTAKE)
  - Set `active_submode` output to last_submode on resume

---

## Region 2: SAFETY_SUPERVISOR (Fault Monitoring)

### States
| State | Purpose | Initial | Entry Action | Exit Action |
|-------|---------|---------|--------------|------------|
| **WATCHDOG_OK** | Pipeline nominal | ✓ Yes | `safety_override_active = false;` | - |
| **WATCHDOG_FAULT** | Latency or pipeline failure | No | `safety_override_active = true; emergency_brake_active = true;` | - |

### Transitions

#### WATCHDOG_OK → WATCHDOG_FAULT
- **Guard**: `!pipeline_ok || total_ms > 150`
- **Triggers on**:
  - Pipeline exception (pipeline_ok = false)
  - Total latency exceeds 150ms (closed-loop collapse threshold from architecture.md)
- **Action**: Force emergency brake immediately

#### WATCHDOG_FAULT → WATCHDOG_OK
- **Guard**: `pipeline_ok && total_ms <= 150`
- **Rationale**: Both conditions must be met to declare recovery
- **Hysteresis**: None needed; symmetric recovery

---

## Chart Inputs (from pipeline_wrapper)

| Name | Type | Description | Units |
|------|------|-------------|-------|
| `is_valid` | boolean | Path planning succeeded | - |
| `replanned` | boolean | Fresh plan computed this tick | - |
| `pipeline_ok` | boolean | No Python exception | - |
| `total_ms` | double | Full pipeline latency | milliseconds |
| `nearest_class_id` | int32 | Nearest obstacle class (0=ped, 1=animal, -1=none) | - |
| `nearest_conf` | double | Detection confidence | [0, 1] |
| `nearest_dist` | double | Distance to nearest obstacle | meters |
| `nearest_ttc` | double | Time-to-collision estimate | seconds |
| `ego_speed` | double | Vehicle forward speed | m/s |

---

## Chart Outputs (to control block)

| Name | Type | Description | Notes |
|------|------|-------------|-------|
| `emergency_brake_active` | boolean | Hard stop (brake=1, throttle=0) | Driven by both OBSTACLE_DETECTED and WATCHDOG_FAULT |
| `replan_requested` | boolean | Request new path from planner | For next tick |
| `animal_caution_active` | boolean | Special handling for livestock | Separate from emergency brake |
| `safety_override_active` | boolean | Watchdog has detected fault | Set by SAFETY_SUPERVISOR region |
| `active_submode` | int32 | Lane-keep vs overtake (memory) | 0=LANE_KEEP, 1=OVERTAKE |

---

## Implementation Notes

### Debounce on Entry to OBSTACLE_DETECTED
The entry guard `nearest_dist < 15 && nearest_ttc < 3 && nearest_conf > 0.6` should only transition if sustained for ~0.3 seconds. Options:
1. **Stateflow duration()**: Use `duration(obstacleConfirmed, 0.3)` in guard
2. **External counter**: Maintain a tick counter in NORMAL_DRIVE; increment if condition true, reset otherwise; transition on count >= 6 (at 20Hz, 6 ticks = 0.3s)
3. **Local state variable**: Add a local variable `ticks_obstacle_detected`

**Recommended**: Option 1 (Stateflow duration) if available; otherwise Option 2 (counter).

### Hysteresis on Resume (OBSTACLE_DETECTED → NORMAL_DRIVE)
The exit guard requires BOTH:
- Obstacle cleared: `nearest_dist > 20 || nearest_ttc > 5`
- Time elapsed: `time_in_state > 2.0` (stay in OBSTACLE_DETECTED for at least 2 seconds after clear)

**Implementation**: 
```
time_in_state = elapsed_time since entering OBSTACLE_DETECTED
```
Use Stateflow's `after(2, sec)` operator or maintain a local timer.

### History Junction for Submode Memory
When resuming to NORMAL_DRIVE, we want to remember whether the vehicle was in LANE_KEEP or OVERTAKE_IN_PROGRESS mode.

**Simple implementation** (without formal sub-states):
- Add output `active_submode` (int32: 0=LANE_KEEP, 1=OVERTAKE)
- On exit from OBSTACLE_DETECTED: `last_submode = active_submode`
- On entry to NORMAL_DRIVE: `active_submode = last_submode`

---

## Guard Condition Thresholds (Tunable)

These values can be adjusted based on testing:

| Parameter | Current Value | Range | Rationale |
|-----------|---------------|-------|-----------|
| Entry distance threshold | 15m | 10-20m | Comfortable braking distance |
| Entry TTC threshold | 3s | 2-4s | Closure velocity dependent |
| Entry confidence threshold | 0.6 | 0.5-0.8 | Balance false positives vs misses |
| Debounce time | 0.3s | 0.2-0.5s | Sensor noise typical frequency |
| Exit distance threshold | 20m | 15-25m | Hysteresis vs entry |
| Exit TTC threshold | 5s | 4-6s | Hysteresis vs entry |
| Resume hold time | 2.0s | 1-3s | Prevent chattering at boundary |
| Latency threshold | 150ms | 150-200ms | Closed-loop collapse point |

---

## Timing Diagram

```
Time →

NORMAL_DRIVE:        [========]←obstacle entry guard met for 0.3s→[EXIT]
OBSTACLE_DETECTED:          [ENTRY]←stay 2s while clear→[OBSTACLE_DETECTED]
WATCHDOG_OK:         [========]←latency spike→[EXIT]
WATCHDOG_FAULT:              [ENTRY]←latency/pipeline recovered→[WATCHDOG_OK]

emergency_brake:     F        [T on fault/animal]→[F after recovery]
replan_requested:    F        [T in obstacle]→[F on resume]
animal_caution:      F        [T if class=animal]→[F on resume]
```

---

## Simulink Integration Checklist

- [ ] Create new Simulink model: `models/plan_a_full_model.slx`
- [ ] Add pipeline_wrapper MATLAB Function block
- [ ] Create DecisionLogic Stateflow chart (this spec)
- [ ] Add control block (pure pursuit + MPC)
- [ ] Wire wrapper outputs → chart inputs
- [ ] Wire chart outputs → control block inputs
- [ ] Add From Workspace blocks for synthetic/CARLA inputs
- [ ] Add Scope/Display blocks for logging
- [ ] Set simulation solver: FixedStepDiscrete, dt=0.05s
- [ ] Test with 20 synthetic ticks
- [ ] Verify latency, replans, obstacle handling
- [ ] Phase 3: Connect to real CARLA

---

## References

- **architecture.md**: Overall 8-stage pipeline architecture; Decision logic Stage 6
- **pipeline-decision-log.md**: Tracker/planner implementation details
- **Phase 2.2 contract**: Input/output data marshaling between MATLAB and Python
- **Phase 1 test results**: Confirmed wrapper latency baseline (2.86ms mean)
