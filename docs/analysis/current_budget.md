# Robot Current Budget

Living document — update with practice data from AdvantageScope / hoot logs.  
All values in **Amps (supply current from battery)** unless noted.  
Last updated: 2026-07-29

## Changes applied

| Date | Change | File | Notes |
|---|---|---|---|
| 2026-07-29 | Added 40 A supply current limit to drive motors | `TunerConstants.java` | Starting point — raise to 45 A if acceleration feels sluggish |
| 2026-07-29 | Added slew rate limiter (3.0/sec) to gamepad turn input | `DriveInputSmoother.java` | 0 → full stick in ~0.33 s; tune if turning feels sluggish |

---

## Battery and breaker ceiling

| Parameter | Value | Notes |
|---|---|---|
| Main breaker | 120 A | Thermal — brief spikes above 120 A tolerated; sustained trips it |
| Battery internal resistance (healthy) | ~15–25 mΩ | 100 A draw → ~2 V sag at terminals |
| Battery internal resistance (aged) | 50+ mΩ | Same 100 A → 5 V sag — check batteries regularly |
| RoboRIO brownout threshold | ~6.8 V | PDH voltage; includes wiring losses |
| Practical safe ceiling (sustained) | ~100–110 A | Keeps voltage well above brownout on a healthy battery |

---

## Motor inventory

| Subsystem | Motor | Count | Controller |
|---|---|---|---|
| Drive | Kraken X60 (TalonFX) | 4 | CANivore "Swerve Bus" |
| Steer (azimuth) | Kraken X60 (TalonFX) | 4 | CANivore "Swerve Bus" |
| Shooter | Kraken X60 (TalonFX) | 3 | RIO CAN |
| Feeder | Kraken X60 (TalonFX) | 1 | RIO CAN |
| Intake Rollers | Kraken X60 (TalonFX) | 1 | RIO CAN |
| Intake Pivot | Neo 2.0 (SPARK Max) | 1 | RIO CAN |
| Floor | Neo / Neo 2 (SPARK Max) | 1 | RIO CAN |
| Hanger | — | — | Not present (2026 offseason) |

> **Note:** `Constants.java` line 85 has a stale comment "REV Neo 2.0 (used on Feeder)" — the Feeder is actually a TalonFX Kraken.

---

## Configured current limits

| Subsystem | Stator limit | Supply limit | Source | Status |
|---|---|---|---|---|
| Drive (×4) | 120 A via `kSlipCurrent` (traction control) | 40 A | TunerConstants.java | Set 2026-07-29 — validate on practice field |
| Steer (×4) | 60 A | None | TunerConstants.java | OK for azimuth load |
| Shooter (×3) | 120 A | 70 A | Shooter.java | Set |
| Feeder (×1) | 120 A | 50 A | Feeder.java | Set |
| Intake Rollers (×1) | 120 A | 50 A | IntakeRollers.java | Set |
| Intake Pivot (×1) | — | 40 A (REV smart) | IntakePivot.java | Set |
| Floor (×1) | — | 40 A (REV smart) | Floor.java | Set |

> REV `smartCurrentLimit` is a combined stator+supply protection — not a pure supply cap like TalonFX.

---

## Per-subsystem current estimates

Estimates below are starting points from motor specs and typical FRC experience.  
**Replace with measured values from AdvantageScope logs as practice data is collected.**

### Drive motors (4× Kraken, supply limit TBD)

| Condition | Estimated draw per motor | Total (×4) | Measured |
|---|---|---|---|
| Idle / coasting | ~1–2 A | ~4–8 A | — |
| Moderate driving (50% stick) | ~10–15 A | ~40–60 A | — |
| Hard driving (full stick, straight) | ~20–25 A | ~80–100 A | — |
| Hard turn (full rotation stick) | ~30–50 A unconstrained | **120–200 A** ← brownout zone | — |
| Hard turn with 40 A supply limit | ≤40 A | ≤160 A | — |

**Proposed supply limit:** 40 A per motor (starting point). Raise to 45 A if acceleration feels sluggish during pure driving. Validate with shooter + drive simultaneous test.

### Steer motors (4× Kraken, 60 A stator only)

| Condition | Estimated draw per motor | Total (×4) | Measured |
|---|---|---|---|
| Holding position | ~1–3 A | ~4–12 A | — |
| Active steering | ~5–15 A | ~20–60 A | — |

Steer supply current isn't limited today but motors are small-load — low brownout risk. Monitor in logs.

### Shooter (3× Kraken, 70 A supply each)

| Condition | Estimated draw per motor | Total (×3) | Measured |
|---|---|---|---|
| Idle / stopped | ~1 A | ~3 A | — |
| Spinning up from rest | ~40–70 A | ~120–210 A | **70.7 / 73.9 / 72.9 A supply peak (hitting limit); 101.4 / 101.8 / 98.9 A stator peak** |
| At speed (steady state, ~3000 RPM) | ~8–15 A | ~24–45 A | — |

> Measured 2026-07-29. All three motors hitting the 70 A supply limit during spinup. 3 × 70 A = 210 A from shooter alone. Limit is working but value is too high for the overall power budget.

### Feeder (1× Kraken, 50 A supply)

| Condition | Estimated draw | Measured |
|---|---|---|
| Idle | ~1 A | — |
| Feeding at 5000 RPM | ~10–20 A | **51.1 A peak (at limit)** |
| Peak / stall | ≤50 A (capped) | — |

### Intake Rollers (1× Kraken, 50 A supply)

| Condition | Estimated draw | Measured |
|---|---|---|
| Idle / stopped | ~1 A | — |
| Intaking at 80% output | ~15–30 A | — |
| Stall (ball jam) | ≤50 A (capped) | — |

### Intake Pivot (1× Neo 2, 40 A smart limit)

| Condition | Estimated draw | Measured |
|---|---|---|
| Holding stowed | ~2–5 A | — |
| Deploying to intake | ~10–20 A | — |
| Homing (5% output) | ~1–3 A | — |

### Floor (1× Neo/Neo 2, 40 A smart limit)

| Condition | Estimated draw | Measured |
|---|---|---|
| Idle | ~1 A | — |
| Feeding at 55% output | ~8–15 A | **50.4 A peak — exceeds configured 40 A smart limit** |

> REV smartCurrentLimit is a stator-based limit with a brief burst allowance — explains why measured briefly exceeds the configured value. Effective supply cap is looser than TalonFX supply limits.

### Electronics (RoboRIO, radio, Limelight, sensors)

| Component | Estimated draw | Measured |
|---|---|---|
| RoboRIO 2 | ~3–5 A | — |
| OpenMesh radio | ~1–2 A | — |
| Limelight 4 | ~1–2 A | — |
| Pigeon 2 + CANcoders + misc | ~1 A | — |
| **Electronics total** | **~6–10 A** | — |

---

## Worst-case scenario budgets

These are the key concurrent-load scenarios from the intake → drive → aim → shoot cycle.

### Scenario A: Hard turn (pure driving)

| Subsystem | Peak draw |
|---|---|
| Drive (4×, at 40 A limit) | 160 A |
| Steer (4×, estimated) | ~30 A |
| All mechanisms idle | ~5 A |
| Electronics | ~8 A |
| **Total** | **~203 A** |

Main breaker will absorb brief spikes. Battery voltage sag at 200 A on a healthy 20 mΩ battery = ~4 V → 8 V at terminals. **Marginal — adding supply limit is the fix.**

### Scenario B: Driving while intaking (hard turn + intake running)

| Subsystem | Peak draw |
|---|---|
| Drive (4×, at 40 A limit) | 160 A |
| Intake Rollers | ~30 A |
| Floor | ~15 A |
| Intake Pivot (moving) | ~15 A |
| Electronics | ~8 A |
| **Total** | **~228 A** |

More realistic: drive won't be at full acceleration the whole time during intake. Expect 150–180 A sustained. **Monitor in practice.**

### Scenario C: Hard driving + shooter spinning up (worst case)

| Subsystem | Peak draw |
|---|---|
| Drive (4×, at 40 A limit) | 160 A |
| Shooter spinup (3× at 70 A limit each) | 210 A |
| Feeder | ~51 A |
| Floor | ~50 A |
| Electronics | ~8 A |
| **Total** | **~479 A** |

> **CONFIRMED by 2026-07-29 practice run.** 20 brownout events in 104 seconds of teleop. Voltage crashed to 5.25 V (hard cutoff is 6.3 V). Shooter hitting 70 A × 3 + feeder 51 A + floor 50 A = 311 A from mechanisms alone, stacking simultaneously. **Mitigation: pre-spin shooter before approaching hub** (already flagged as #1 priority in intake analysis memory). Also need to reduce shooter supply limit from 70 A.

### Scenario D: Shooter at speed + moderate driving (most common match scenario)

| Subsystem | Peak draw |
|---|---|
| Drive (4×, moderate) | ~80 A |
| Shooter at speed (3× at ~12 A each) | ~36 A |
| Feeder (during feed) | ~15 A |
| Electronics | ~8 A |
| **Total** | **~139 A** |

This is the expected normal-play scenario. Well within breaker limits.

---

## Data collection checklist

When running practice with AdvantageScope / hoot files, log and record:

- [ ] Drive supply current per motor during hard turn (all 4 channels)
- [ ] Drive supply current during straight-line sprint
- [ ] Shooter supply current: spinup peak and at-speed steady state  
- [ ] Feeder supply current during feed
- [ ] Intake roller supply current during intake
- [ ] Battery voltage minimum during Scenario B (intake while driving)
- [ ] Battery voltage minimum during Scenario C (turn while shooter spins up)
- [ ] Steer supply current during hard turn (is it a concern?)

Update the "Measured" columns above as data comes in.
