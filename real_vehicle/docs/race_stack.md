# The race stack on the real car

Status: **deployed and under validation. The car has not moved under its own
control.** The no-movement ladder below is the gate; the motion test is staged
and announced.

What this is: the autonomy the simulation validated, running on the car.
Learned odometry (the trained LiDAR-inertial network) feeds slam_toolbox, which
builds the map; the race driver explores lap 1 reactively from the LiDAR alone,
plans a race line on its own map, and races it; and underneath all of it the
adaptive layer identifies the car's own plant -- where its axles are, how much
its servo really steers, where its motor starts -- from the odometry, and
inverts it. Nothing in the car is told the track.

The reference is `simulation/data/real2sim/final_s13`: three laps, 13.7 cm of
minimum clearance, plant identified with confidence 0.966.

---

## 1. The graph

```
WT61PC @100 Hz ──/apex/imu/data_raw──┐
A2M8 Express @10 Hz ──/lidar/scan_raw┤
                                     ├─► apex_learned_odometry   (core 1, /odom_rt)
                                     │     ONNX network + polar ICP, 1 thread
                                     │     ├─ /apex/odometry/learned            10 Hz
                                     │     ├─ /apex/odometry/learned_predicted 100 Hz
                                     │     ├─ /apex/learned_odometry/scan_deskewed
                                     │     ├─ /apex/learned_odometry/status
                                     │     └─ TF odom_learned → base_link
                                     │
                  slam_toolbox (core 2) ◄┘  /map, graph, TF map → odom_learned
                                     │
   /apex/tracking/arm ───────────────┤
                       apex_race_driver (core 3)  50 Hz control, 10 Hz reactive
                                     ├─ /apex/cmd_vel_track        ← single owner
                                     ├─ /apex/race_driver/status
                                     └─ planner worker, cores 2-3, nice 10
                                     │
              cmd_vel_to_apex_actuation_node (core 0, profile `affine`)
                                     └─ sysfs PWM ch0 ESC / ch1 servo
```

Everything legacy (`recognition_tour`, `curve_*`, `fixed_map`, the session
manager) is off in race mode, because three of those nodes also publish
`/apex/cmd_vel_track` and the car may have exactly one driver. `apex_race_up.sh`
asserts that before arming is possible.

## 2. Where the code lives, and why it is in three places

| Part | Where | Why there |
|---|---|---|
| Driver, lap-1 planner, race planner, plant identification, adaptive layer | `real_vehicle/ros2_ws/src/apex_race/apex_race/core/` | **byte-identical copies** of the simulation's modules. `test/test_core_parity.py` hashes all thirteen against the simulation tree and fails if one drifts: what Gazebo validated has to be what the car runs. |
| The estimator and the trained model | `~/odom_rt` on the Pi, mounted read-only at `/odom_rt` | It carries ~1 GB of torch, onnxruntime and numba, which will not fit in the image, and it is coupled to the weights: the de-skewing, the metric embedding and the ICP are the network's input pipeline. `sync_odom_rt_to_pi.sh` ships an explicit file list with a sha256 manifest and the git commit. |
| `Path2D`, `path_from_points`, `PurePursuit`, `_forward_backward` | `apex_race/apex_race/vendor/pose_dataset/` | The only four objects of the simulation's pose-dataset package the driving logic may import. The firewall test asserts that the vendored surface is exactly those four. |

The knowledge firewall (`test/test_car_knowledge_firewall.py`) scans every module of
the package for the track definitions, the speed calibration, the ground truth
and the simulation tree. The car has to find the track with its own LiDAR.

## 3. The car's own numbers

`config/vehicle_real.yaml` and `config/race_driver.yaml` hold every value that
differs from the simulation, each with the reason. The four that matter most:

**Steering: ±12.5°, not ±18°.** The servo's duty window is [5.0, 10.0] % about a
mechanical centre of 8.2 %, so it reaches −23.0° one way and +12.96° the other.
Every planner here assumes a symmetric authority, so the smaller of the two is
the real limit. `SteeringServo` now clamps the *requested* angle to it, and
`applied_deg` can no longer report an angle the wheels did not take -- the car
used to log 46-48° requests that the duty bounds silently saturated.

**Speed: the bridge is affine now.** The legacy profile stepped to 17 % at any
non-zero command, then applied an exponential ramp, a launch boost and a
ramp-down, so what the car delivered depended on the command *history* -- and
the plant identifier would have measured the bridge's ramp instead of the car.
The `affine` profile is one straight line per direction:

```
pct(u) = 15 % + 33 %/(m/s) · u,   ceiling 45 %   →  0.91 m/s at full scale
```

The 15 % offset sits deliberately *below* the believed 17 % start threshold:
the leftover 0.061 m/s is a residual deadband the identifier brackets from the
launches, and an error on that side leaves the car still rather than lurching.

**Speeds: explore 0.70, race 0.90 m/s.** Below this the adaptive controller
cannot identify anything -- the curvature map needs 0.45 m/s, the lever arm
0.30 m/s -- and the odometry network drops below its 0.3 m/s training floor.
Above it, the stopping distance stops fitting in a one-metre lane.

**`reactive.w_min_m` is a measurement, not a tuning knob.** It is how wide the
lane really is. Set it to the corridor you are driving in.

## 4. What the 2D harness says about the track

The lap-1 reactive driver had never been validated in a corridor of about one
metre. It was measured there before the car was allowed to move
(`race_harness_2d.py`, 7 seeds, three laps each, the measured 17.4 % missing
returns, the plant drawn as it may be):

| corridor | corner radius | steering | result |
|---|---|---|---|
| 1.3 m | 1.5 m | ±12.5° | **6/7 complete, clearance 15-20 cm** |
| 1.1 m | 1.5 m | ±12.5° | 5/7 |
| 1.1 m | 1.2 m | ±12.5° | marginal (4 cm) |
| 1.1 m | 1.2 m | ±18° | 2/2 complete |
| 1.1 m | 1.0 m | either | 0/2 |

The arithmetic behind it, worth keeping: the usable centre band is
`corridor − 0.514 m` (two footprint circles of 0.177 m plus `c_hard` 0.08); the
largest path radius through a corner of centreline radius `Rc` is `Rc + band/2`;
and the steering that needs is `atan(0.30 / that)`.

**Every seed that fails has confidence 0.00 at the moment it fails, and every
seed that completes reaches at least 0.28.** The failures are the plants whose
servo delivers 11-13 % less than commanded: they run wide in a corner before the
identifier has measured them. This is why the motion test is staged -- the first
steps measure the plant on a straight line, not in a corner.

Two configuration defects were found this way and fixed:

* `reactive.lane_margin_m` was 0.5 m, sized for 1.5 m/s. At 0.70 m/s the proven-lane
  rule then demanded 0.81 m of visible lane to cruise, and a corner only ever
  proves about 0.6 m ahead, so **the car stopped in every corner**. Now 0.25 m
  (0.2 s of latency plus one revolution).
* `adaptive.recover_clearance_m` was 0.25 m, which **silently disabled the reverse
  recovery** in a narrow corridor: there are always accumulated points inside the
  0.31 m half-strip behind the car, so the car stood still through 13 s of being
  stuck without ever trying to back out. Now 0.15 m; what makes the reverse safe
  is that it is bounded in distance, time and attempts per lap.

## 4b. The one thing lap 1 cannot do yet

Every harness failure happens with the identifier still at confidence 0, so the
obvious fix is to let the car start a run already knowing itself. It does not,
and neither does the simulation: `PlantIdentifier` begins every run at the
priors and nothing is persisted between runs.

Two thirds of a warm start are available through configuration today, and the
staged motion test is what fills them in: after the straight-line and
figure-of-eight steps, read the identified values off
`/apex/race_driver/status` and write them into `config/vehicle_real.yaml`:

| measured | goes to | what it is the prior of |
|---|---|---|
| `gain_right / gain_left` | `servo.right_ratio` | the right-turn gain |
| `dead_mps` | `esc.min_effective_speed_mps` | the start threshold |

The left-turn gain and the steering trim have no configurable prior, so they
are still learned from scratch on every run. Giving `PlantIdentifier` a
`warm_start()` -- a measured plant loaded as a *prior with its own sigma*, not
as a certainty, so the filter still refines it -- is the obvious next step and
belongs on the simulation side first, where the parity test can police the copy.

## 4c. The blocker: the 5 V supply

Measured on 2026-10-11, and it stops the ladder where it is:

| state | `vcgencmd get_throttled` |
|---|---|
| LiDAR stopped, governor `ondemand` | `0x50000` — historical bits only, the rail is fine |
| LiDAR motor turning | `0x50005` — **under-voltage now, throttled now** |
| LiDAR motor + governor `performance` | the motor never reaches speed; no scan is ever published |

The LiDAR node measures it itself during the soft start. With no other load the
ramp takes 11.1 s and sees under-voltage in 144 polls; with the race stack
starting beside it, 37.0 s and 598 polls. And on the run after that, with the
motor turning and the estimator warming up, **the Pi stopped responding to SSH
and to ping** — the reset at motor start that
`docs/sensor_timing_requirements.md` already describes, reproduced with the
full stack.

What changed in the code because of this:

* the race stack **waits until `/lidar/scan_raw` is actually publishing** before
  it raises the governor or starts the estimator, so the CPU is idle while the
  motor draws its worst current;
* the `performance` governor is **no longer forced**. Pinning four cores at
  2.4 GHz was enough on its own to keep the motor from reaching speed. The Pi 5
  boosts under load by itself; `APEX_SET_GOVERNOR=performance` is available for
  when the supply can take it, and it is restored on exit;
* with the actuation bridge enabled, the stack **refuses to start** on live
  under-voltage. A Pi that browns out mid-run takes the estimator, the SLAM and
  the driver with it, and the car coasts until the bridge's 0.35 s watchdog
  fires.

None of that is a fix. The fix is a 5 V supply that can carry a Pi 5 under load
and the LiDAR motor at the same time, and it is a hard gate for the motion
test. Until then the car is safe to leave alone: the pipeline container is
`restart: "no"`, the LiDAR's boot mode is off and `apex-real-ready.service` is
disabled, so nothing starts the motor by itself after a reset.

## 5. Running it

```bash
# on the workstation
./real_vehicle/tools/core/sync_apex_code_to_pi.sh      # code  -> ~/AiAtonomousRc/APEX
./real_vehicle/tools/core/sync_odom_rt_to_pi.sh        # model -> ~/odom_rt, with a manifest

# on the car
cd ~/AiAtonomousRc/APEX
./tools/core/apex_race_up.sh sensors-only              # C: capture + timing audit
./tools/core/apex_race_up.sh dry                       # D1: everything runs, nothing moves
./tools/core/apex_race_up.sh check --duration 180      #     PASS/FAIL of the whole graph
./tools/core/apex_race_up.sh replay /work/data/replay/final_s13_sensors.npz
                                                       # D2: the full state machine, car off
./tools/core/apex_race_up.sh armed                     # the real thing; announce first
./tools/core/apex_race_up.sh arm                       # in another terminal; Ctrl-C disarms
./tools/core/apex_race_up.sh down
```

`dry` points the driver's commands at `/apex/race_driver/cmd_vel_dry` and does
not start the actuation bridge at all, so the ESC cannot be reached even by
mistake. `replay` additionally replaces the two sensor drivers with a recording,
which is the only way the whole state machine -- explore, closing, planning,
race -- runs on the Pi with the car switched off: a stationary car never leaves
`wait`, because the lap counter needs a closed lap.

## 5b. Losing the car mid-run

The car is on Wi-Fi to a laptop hotspot and it has gone off the network in the
middle of a run. Three things keep that from costing a run:

* **the stack is detached.** `apex_race_up.sh` starts the container with
  `docker compose up -d`, so an SSH session that dies takes nothing with it.
  Every run writes into its own `ros2_ws/apex_race/<timestamp>/`, and
  `apex_race_up.sh status` prints what is running and what that run has written
  so far. It is the first thing to run when the link comes back; it touches
  nothing.
* **`APEX_RACE_PERSIST=1`** gives the container Docker's `unless-stopped`
  policy, so the session comes back by itself after a reboot or a crash,
  spaced out by Docker's own backoff. Off by default, because a stack that
  cannot start must not loop.
* **`apex-net-watchdog`** (`tools/core/install_apex_net_watchdog.sh`) reconnects
  the Wi-Fi and writes one line per drop into `data/net_watchdog.csv`, together
  with the rail state at that moment and whether the stack was still running.

Its link check is deliberately *not* a ping of the gateway: this hotspot does
not answer ICMP, and the first version of the watchdog read that as a dead link
and bounced the Wi-Fi every thirty seconds. What it checks is NetworkManager's
own device state plus an address and a default route.

That log is also how the next disappearance gets diagnosed: a radio that drops
and returns leaves `miss` and `recovered` rows; a car that went down entirely
leaves none, because nothing was running to write them.

## 5c. What the car measured (static replay, 2026-10-11)

Run `20261011_101220`: the recorded sensors of `final_s13` replayed into the
real stack on the Pi, with no motor and nothing able to move.

| | |
|---|---|
| topics | all eight up, including `/map` from slam_toolbox |
| command gap | **48.8 ms** worst (gate: under 100 ms) |
| stale scans dropped | 0 |
| **reactive step** | **145 ms mean, 395 ms worst, over 371 revolutions** |
| everything else holding the core | under 1 ms (odometry 0.5, map 0.1, graph 0.2, control 0.2) |
| consequence | the control tick lost its slot 1860 times; the driver never left `explore` |

The lap-1 planner is the whole budget and it does not fit in the 100 ms a
revolution allows. The candidate grid went from 21x11 to 15x7 because of it --
the 2D harness gives the same six-of-seven seeds and the same 15-20 cm of
clearance either way, so it costs nothing that shows up in how the car drives --
and run `20261011_131021` measured the result:

| grid | reactive step | worst | command gap | control ticks lost |
|---|---|---|---|---|
| 21x11 (231 arcs) | 145.0 ms | 395 ms | 48.8 ms | 1860 |
| 15x7 (105 arcs) | **107.7 ms** | 414 ms | 70.4 ms | 1368 |

A 26 % saving, not the 2.2x the arc count suggests, because the candidate
evaluation is not the whole cost: accumulating the cloud, clustering it,
tracking the walls and building the distance field are paid once per
revolution whatever the grid. **It is still at the budget, not under it**, and
16 of that run's 138 seconds were spent below 2.4 GHz with the rail sagging, so
the true figure is somewhere under 108 ms and has to be re-measured on a healthy
supply before anything else is cut.

If it still does not fit then, the levers left are the fixed costs -- a coarser
voxel for the accumulated cloud, a shorter horizon -- or planning every other
revolution, which at 0.70 m/s means replanning every 14 cm instead of every 7.
That last one changes how the car drives and belongs in the 2D harness first.

The same run also measured the rail: **60 of 112 seconds in live under-voltage
with no motor turning**, on CPU load alone. And `in0_lcrit_alarm` latches -- it
read 0 at idle minutes earlier and 1 in all 112 samples afterwards -- which is
worth knowing because the LiDAR driver's `SupplyGuard` treats that file as a
live signal, so once it latches the motor ramp always waits its maximum. That
is why the ramp went from 11.1 s to 37.0 s.

## 5d. What the dual supply changed, and the drift it left behind (2026-10-11, later)

The supply was the compute problem. With the second feed in, measured at rest
and on load: rail `0x0`, zero under-voltage polls, motor ramp 37.0 s -> 7.0 s,
LiDAR jitter 13-46 ms -> 0.1 ms, 256 valid bins instead of 244. **The reactive
step went from 145 ms to 37 ms mean (75 ms worst)**, so the grid cut in 5c was
answering a brown-out, not a budget: the 21x11 grid of the simulation is
restored. Live sensors then gave 24 of 25 on the bring-up check (estimator
latency p95 160 ms, compute p95 15 ms, command gap p99.9 35 ms, TF complete,
SLAM publishing); the one miss was this document's own governor check, now
informative. What appeared instead is *thermal* throttling (`0xe0000`, no
under-voltage bit), which a fan would answer.

**Bounded motion, authorised, under 30 cm.** The affine map is exact: 0.20 m/s
commanded drove the ESC to 21.6 %, which is 15 + 33 x 0.20 to the digit. The
odometry reported 4.1 cm and the car moved about 4 cm -- confirmed by eye -- so
**the learned odometry follows real motion**. The delivered speed peaked at
0.125 m/s against the 0.138 the believed curve predicted, and two repeats gave
7.4 and 7.7 cm.

**The drift that was left: the car walked away while parked.** 0.42 m/min with
the estimator freshly started, 0.97 m/min after a single 7 cm pulse, 4.78 m/min
after a series -- it grows with motion, not with time standing. The sensors were
clean and the classical ICP called the standstill on 1125 of 1142 revolutions,
so the invented motion was the learned part of the fusion, with `bias_ay` on its
clamp at -1.00000.

That is **fixed twice over, in the simulation tree, and validated there**.

1. **At inference** (`simulation/learning/lidar_imu_pose/standstill.py`, README
   section 15): while the classical ICP confirms a standstill, its increment is
   used and the network is not stepped. Three parked minutes went from 23.8 m of
   drift to 3 mm.
2. **In the weights** (`real2sim_v7e_standstill`, README section 16): the
   network no longer believes the car is moving. Standing still was simply not
   in the training set -- the dataset's recorded standstill is 1.5 s -- and
   neither was the WT61PC's habit of reporting exactly 0.000 rad/s at rest.
   With both in, and with the inference hold **off**, three parked minutes cost
   **0.13 m** instead of 23.8, and driving is unchanged on 108 held-out runs.

The two are independent and both ship. The architecture is v5's, so the Pi
budget is unchanged: 18 ms per scan.

**Neither is on the car yet.** Deploying means syncing `standstill.py`, the v7
checkpoint and its `.onnx` to `~/odom_rt` (`tools/core/sync_odom_rt_to_pi.sh`
already lists the first), then repeating `apex_race_up.sh check --stationary`,
which is the gate that measures exactly this. Until then the operating rule
stands: start the estimator shortly before driving, never leave it idling.

## 6. The no-movement ladder

| # | Step | Pass criterion |
|---|---|---|
| U | `pytest real_vehicle/ros2_ws/src/apex_race/test real_vehicle/ros2_ws/src/apex_telemetry/test` | all green: 13/13 parity hashes, the firewall, the 48°→12.5° regression, the curvature round-trip to 1e-9, and identifiability (the affine envelope identifies, the legacy one provably cannot) |
| B | `pytest simulation/ros2_ws/src/apex_fusion_research/test -m "not slow"` | all green |
| H1 | `race_harness_2d.py --vehicle-yaml … --driver-params …` | §4 |
| C | `apex_race_up.sh sensors-only` + `record_real2sim_sensors.py` + `sensor_timing_audit.py --profile APEX_real_wt61` | LiDAR 10/10 as in `lidar_check_static_02.json`; IMU 100 Hz ±5 %; 10 ± 20 % IMU samples per revolution |
| R1 | `replay_slam.py prepare` on that capture | the offline estimator matches the Pi's `estimator.csv` within 1 mm |
| R2 | `replay_slam.py run` on that capture | ≥90 % of scans processed; with the car at rest the total map→odom correction stays under 5 cm over 120 s |
| S | static servo sweep, wheels off the ground, **ESC unplugged** | symmetric authority ≥12.0°; centre within 0.5°; `applied_deg` never exceeds the limit; `right_ratio` measured into `vehicle_real.yaml` |
| P | `vcgencmd get_throttled` / `measure_temp` / governor | no live under-voltage, <75 °C, `performance` |
| I | IMU axis signs, static | at rest `az ≈ +9.8`; nose up 30° → `ax ≈ −4.9`; **yawing left → `gz > 0`**. The WT61PC firmware passes the raw sensor axes through, and a flipped `gz` latches `sign_fault` and kills the adaptation silently |
| D1 | `apex_race_up.sh dry` + `check --duration 180 --expect-phase explore` | TF chain complete; estimator p95 latency <250 ms and compute <40 ms; **command gap p99.9 <100 ms**; no core above 85 %; 0 scan-age stops |
| D2 | `apex_race_up.sh replay` + `check` | `race_events.json` reaches `planning`, `handover`, `t_race`, `t_done`; `scan_age_stops == 0`; the command gap holds **through the planner burst** |

Step **H** -- pushing the car by hand with the ESC unplugged -- is the only way
to measure the odometry's accuracy at the real speed before the car drives
itself. It is movement, and it is announced like any other.

## 7. Go / no-go for the first motion test

U, B, H1 · C 10/10 · R1 <1 mm · R2 <5 cm · S ≥12° symmetric · I all four signs ·
D1 and D2 with 0 scan-age stops, no core above 85 % and the command gap under
100 ms · P with no live under-voltage · `/apex/cmd_vel_track` with exactly one
publisher · ≥1 GB free · and a person holding the arm with physical access to
the battery.

Then, staged: 2 m straight → 5 m, reading the identified `v_gain` off the status
to re-set `speed_pct_per_mps` → a figure of eight for `gain_left/right` and
`trim` → lap 1 → three laps at 0.90 m/s.
