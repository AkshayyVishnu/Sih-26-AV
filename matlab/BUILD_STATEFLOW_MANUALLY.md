# Manual Build Guide: Stateflow Decision Logic Chart

**Status**: Phase 2.4 ready for manual construction  
**Time estimate**: 20-30 minutes  
**Prerequisite**: `models/plan_a_full_model.slx` created with pipeline_wrapper block

---

## Step-by-Step Instructions

### 1. Create Base Model

```matlab
>> new_system('plan_a_full_model')
>> open_system('plan_a_full_model')
>> set_param('plan_a_full_model', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.05')
```

### 2. Add Stateflow Chart

In Simulink editor:
1. **Simulink → Sources** → Stateflow → **Stateflow Chart**
2. Double-click to open the chart editor
3. Rename the chart from `sf_plan_a_full_model` → **`DecisionLogic`**

### 3. Add Chart Inputs (9 total)

In Stateflow editor, **Add → Input**:

```
is_valid           (boolean)
replanned          (boolean)
pipeline_ok        (boolean)
total_ms           (double)
nearest_class_id   (int32)
nearest_conf       (double)
nearest_dist       (double)
nearest_ttc        (double)
ego_speed          (double)
```

**Port numbers**: Should auto-assign 1-9

### 4. Add Chart Outputs (5 total)

**Add → Output**:

```
emergency_brake_active    (boolean)
replan_requested          (boolean)
animal_caution_active     (boolean)
safety_override_active    (boolean)
active_submode            (int32)
```

**Port numbers**: Should auto-assign 1-5

### 5. Create Region 1: DRIVE_MODE

Right-click in chart → **New → Region**

#### 5a. Add NORMAL_DRIVE State
- Draw a state box (Stateflow → State tool)
- Name: `NORMAL_DRIVE`
- Right-click → **Properties → Initial state** ✓ (checkbox)
- Entry action (double-click state → Edit):
  ```
  entry: emergency_brake_active = false; replan_requested = false;
  ```

#### 5b. Add OBSTACLE_DETECTED State
- Draw another state box
- Name: `OBSTACLE_DETECTED`
- Entry action:
  ```
  entry: 
    if (nearest_class_id == 1) {
      % Animal on road: slow approach
      animal_caution_active = true;
      replan_requested = true;
      emergency_brake_active = false;
    } else if (nearest_ttc < 1.0) {
      % Imminent collision: emergency brake
      emergency_brake_active = true;
      replan_requested = true;
      animal_caution_active = false;
    } else {
      % Standard obstacle: replan path
      replan_requested = true;
      emergency_brake_active = false;
      animal_caution_active = false;
    }
  ```

#### 5c. Add Transitions

**NORMAL_DRIVE → OBSTACLE_DETECTED**:
- Draw transition arrow from NORMAL_DRIVE to OBSTACLE_DETECTED
- Guard label (double-click transition):
  ```
  [nearest_dist < 15 && nearest_ttc < 3 && nearest_conf > 0.6]
  ```
  
**OBSTACLE_DETECTED → NORMAL_DRIVE**:
- Draw transition arrow
- Guard label:
  ```
  [nearest_dist > 20 || nearest_ttc > 5]
  ```

### 6. Create Region 2: SAFETY_SUPERVISOR

Right-click in chart → **New → Region**

#### 6a. Add WATCHDOG_OK State
- Name: `WATCHDOG_OK`
- Mark as **Initial state** ✓
- Entry action:
  ```
  entry: safety_override_active = false;
  ```

#### 6b. Add WATCHDOG_FAULT State
- Name: `WATCHDOG_FAULT`
- Entry action:
  ```
  entry: 
    safety_override_active = true;
    emergency_brake_active = true;
  ```

#### 6c. Add Transitions

**WATCHDOG_OK → WATCHDOG_FAULT**:
- Guard:
  ```
  [!pipeline_ok || total_ms > 150]
  ```

**WATCHDOG_FAULT → WATCHDOG_OK**:
- Guard:
  ```
  [pipeline_ok && total_ms <= 150]
  ```

### 7. Verify Chart Structure

When complete, chart should look like:

```
DecisionLogic Chart
├── Region 1: DRIVE_MODE
│   ├── [NORMAL_DRIVE] (initial) ⟷ [OBSTACLE_DETECTED]
│   ├── Transition guards: obstacle detection / clearance
│   └── State actions: brake/replan decisions
│
└── Region 2: SAFETY_SUPERVISOR
    ├── [WATCHDOG_OK] (initial) ⟷ [WATCHDOG_FAULT]
    ├── Transition guards: latency / pipeline OK
    └── Entry actions: fault signaling
```

### 8. Save and Test

1. **Save chart**: Ctrl+S in Stateflow editor
2. **Close chart editor**: Click X
3. **Save model**: Ctrl+S in Simulink editor
4. **Verify**: Double-click DecisionLogic → should open saved chart with all states/transitions

---

## Checklist: What Each Region Does

### DRIVE_MODE Region
- ✓ Detects obstacles (distance, TTC, confidence thresholds)
- ✓ Distinguishes animals from other obstacles
- ✓ Commands emergency brake if TTC < 1.0s
- ✓ Commands replan otherwise
- ✓ Outputs `emergency_brake_active`, `replan_requested`, `animal_caution_active`

### SAFETY_SUPERVISOR Region
- ✓ Monitors pipeline latency (150ms ceiling)
- ✓ Monitors pipeline faults (Python exceptions)
- ✓ Forces emergency brake on fault
- ✓ Outputs `safety_override_active` watchdog signal
- ✓ Independent of DRIVE_MODE (parallel AND region)

---

## Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| Transitions don't fire | Guard syntax error | Check bracket matching in guard condition |
| State actions don't execute | Wrong entry/during/exit clause | Use `entry:` not `during:` for one-time actions |
| Chart won't open | Unsaved model | Save before closing editor |
| Ports mismatch | Input/output added after transitions | Verify port numbers match in Simulink wiring |
| Boolean logic not working | MATLAB Stateflow uses C syntax | Use `!` (not `~`), `&&` (not `&`), `\|\|` (not `\|`) |

---

## Wiring to Other Blocks

Once chart is complete:

1. **Pipeline wrapper outputs** → Chart inputs:
   - `wp_xy`, `n_wp`, `is_valid`, `replanned`, `total_ms` → ports 1-9
   - `nearest_class_id`, `nearest_conf`, `nearest_dist`, `nearest_ttc`

2. **Chart outputs** → Control block inputs:
   - `emergency_brake_active` → brake command override
   - `replan_requested` → replan trigger
   - `animal_caution_active` → soft deceleration (optional)
   - `safety_override_active` → logging/safety monitoring
   - `active_submode` → for future overtake logic (not used in Phase 2)

---

## After You're Done

1. Commit the manual chart to git:
   ```bash
   git add models/plan_a_full_model.slx
   git commit -m "Add DecisionLogic Stateflow chart (Phase 2.4)"
   ```

2. Next phase (Phase 2.5):
   - Add control block (pure pursuit + MPC)
   - Wire outputs to vehicle commands
   - Test full loop on 20 synthetic ticks

3. Documentation:
   - Refer back to `STATEFLOW_CHART_SPEC.md` for parameter tuning
   - Log state transitions for metrics/debugging
   - Measure latency of chart execution (should be < 1ms in Simulink)
