# Notes

## 2026-10-07 - Week 4 done (SLAM)
**Did**
- Built test_world.sdf (8 static models).
- Mapped it with slam_toolbox, saved maps/test_world_map.pgm + .yaml.
- Fixed lidar frame_id with gz_frame_id (lidar_link).
- Reload test passed with nav2_map_server.

**State**
- Map is installed via CMake and visible in the install folder.
- map_server must be configured and activated before it publishes /map.
- In RViz, the Map display needs Transient Local on the Topic row, not Update Topic.

**Next**
- Install navigation2 and nav2_bringup.
- Write nav2 params file. Run AMCL on the saved map.
- Send one goal from RViz.

**Not understood yet**
- How the costmaps are made

## 2026-10-08 - Week 5: Nav2 + AMCL working
**Did**
- Nav2 bringup with own params (config/nav2_params.yaml): base_footprint, robot_radius 0.2, initial pose (0,0,0).
- 3 goals succeeded (2 RViz, 1 terminal). Terminal goal ended ~1 cm from target.

**Findings**
- Sim odometry drifts: map->odom reached 0.37 m / 2.8 deg after a few goals with in-place turns.
- AMCL corrects it (scan dots align with walls). Particle cloud is wide near walls.
- Goal tolerance is 0.25 m / 0.25 rad, so "succeeded" allows ~14 deg heading error.

**Open for week 6**
- collision_monitor stops robot briefly (TF extrapolation errors at lidar_link).
- MPPI "Optimizer fail to compute path" warnings, control loop missed 20 Hz.
- Inflation radius 0.7 is large for the 0.9 m gap.
- 5-run test with unmapped obstacle.

## Week 6 (Oct 20, 2026): unmapped obstacle + 5-run gate

Setup: `unmapped_box` (0.4 x 0.4 x 1 m) at world (1.5, 0) was added to `worlds/test_world.sdf` itself (commit 5319c12). The saved map was made from the world before that commit and does not contain the box. The box is inside lidar range (2.5 m) from the start, so Nav2 sees it before the goal is sent.
Goal: map (3.0, 0.0), heading +x, from spawn (0, 0). Full restart of Gazebo and Nav2 before every run.
Pass rule (written before the runs): SUCCEEDED, no touch (by eye), nothing changed since the last run. Unknown counts as fail.

| Run | Result | Side | Time | Recoveries | Stops | Last feedback pose | Heading (rad) |
|---|---|---|---|---|---|---|---|
| 0 (not counted) | OK | -y | 14.3 s | 0 | 1 | (2.976, 0.053) | 0.244 |
| 1 | PASS | -y | 11.4 s | 0 | 0 | (2.995, -0.030) | 0.268 |
| 2 | PASS | -y | 11.3 s | 0 | 0 | (3.009, -0.030) | 0.247 |
| 3 | PASS | -y | 12.2 s | 0 | 0 | (3.017, -0.018) | 0.250 |
| 4 | PASS | -y | 17.9 s | 0 | 4 | (3.060, -0.054) | 0.222 |
| 5 | PASS | -y | 12.5 s | 0 | 1 | (3.035, 0.050) | 0.206 |

Result: 5 passes in a row. Phase 1 gate met.
Caveats:
- Run 1 numbers come from pasted terminal output; its goal log was overwritten by run 2 (tee filename mistake).
- Feedback pose is AMCL's belief in the map frame, not Gazebo's true pose. Run 1 feedback heading (0.268) is over 0.25 rad yet Nav2 reported SUCCEEDED; cause unknown.
- collision_monitor stopped the robot in 3 of 6 runs (runs 0, 4, 5). Cause: scan timestamp 1-20 ms ahead of latest odom->base_footprint TF ("extrapolation into the future"). Unfixed.
- Run 4: stray lidar points reported at start (~45 degree line, x/-y quadrant), run took 17.9 s. Unexplained, did not recur.
- distance_remaining repeats exact values across runs; not a live distance. Do not use.
- Robot passed on -y side in every run. Only one path was tested.

## 2026-10-26 - Week 1(new plan): collision_monitor stops (mitigated, cause unconfirmed)
**Did**
- Measured sim speed and /odom delivery (Oct 22, no goal sent).
- Ran two batches of 10 runs, same goal (3.0, 0.0), full restart each run (Oct 26).
- Added `headless:=true` launch option (no Gazebo window). Default is false, so the window stays on.

**Findings**
- Runs with stops: old setup (window on, RViz on) 3 of 6. Headless, no RViz: 0 of 10. Headless, RViz on: 1 of 10 (run 06 took 14 s with no stop, unexplained).
- All 20 new runs ended SUCCEEDED.
- Sim speed from raw timestamps: about 0.62x with Nav2 + RViz and window on, about 0.84x with window off.
- /odom with Nav2 on (idle) and no RViz, window off: 49.95 Hz, longest gap 0.024 s. With Nav2 + RViz on, window off: 36.8 Hz, longest gap 0.328 s.
- The real_time_factor field is not reliable (it matched nothing). Use raw sim/real timestamps.
- transform_tolerance is already 0.2 s, so a larger tolerance does not explain the stops.

**Not verified**
- Collisions were not checked in either batch (no window, RViz cannot show a scrape). Not gate evidence.
- Batch A changed two things at once (window off, RViz off).
- The old baseline is only 6 runs, from a different day.
- The link between /odom jitter and the stops is an inference.
- Early guess "RViz causes the stalls" is not supported by batch B.
- Cause of the scan/odom timing gap is still unknown. This is a workaround that lowers load.

**Next**
- DECISIONS.md, README, demo GIF (needs the window, so launch without headless).