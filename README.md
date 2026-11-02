# diffdrive_nav2_ros2

A differential-drive robot built from scratch (URDF/Xacro) and driven by Nav2 in Gazebo Harmonic with ROS2 Jazzy. It localizes on a saved map with AMCL and drives to a goal, going around an obstacle that is not in the map.

<!-- TODO: demo GIF here (robot avoiding the unmapped box) -->

## Status
- [x] Robot description (chassis, wheels, caster, 2D LiDAR)
- [x] Gazebo simulation with differential-drive plugin
- [x] Teleop control, `/odom` and `/scan` publishing
- [x] SLAM (slam_toolbox) and a saved map
- [x] Nav2 autonomous navigation with AMCL
- [x] Unmapped obstacle test: 5 passes in a row (git tag `phase1-gate`)
- [ ] Next: moving obstacles (C++ lidar detection, tracking, custom costmap plugin)

## World and robot
- World: `worlds/test_world.sdf` (4 outer walls, a partition, a side wall, a box, a cylinder, and `unmapped_box`).
- `unmapped_box` (0.4 x 0.4 x 1 m at world (1.5, 0)) is **not** in the saved map `maps/test_world_map` (198 x 158 cells, 0.05 m). Nav2 only sees it through the lidar.
- Nav2 params (`config/nav2_params.yaml`) start from Nav2's TurtleBot3 defaults: NavFn planner, MPPI controller, `collision_monitor`. Changes: `base_footprint`, `robot_radius` 0.2, initial pose (0, 0, 0), `inflation_radius` 0.7 -> 0.4 (local and global).

## Run it
Needs ROS2 Jazzy, Gazebo Harmonic, ros_gz, navigation2 and nav2_bringup. Nav2 and the launch files read the **installed** copy of this package, so rebuild after every edit:
```
cd ~/ros2_ws && colcon build --packages-select diffdrive_nav2_ros2 && source install/setup.bash
```
Terminal 1, Gazebo (add `headless:=true` for no window):
```
ros2 launch diffdrive_nav2_ros2 gazebo.launch.py
```
Terminal 2, Nav2 (wait for "Managed nodes are active" twice). Do not run slam_toolbox at the same time:
```
PARAMS=$(ros2 pkg prefix diffdrive_nav2_ros2)/share/diffdrive_nav2_ros2/config/nav2_params.yaml
MAP=$(ros2 pkg prefix diffdrive_nav2_ros2)/share/diffdrive_nav2_ros2/maps/test_world_map.yaml
ros2 launch nav2_bringup bringup_launch.py map:=$MAP params_file:=$PARAMS use_sim_time:=true
```
Terminal 3 (optional), RViz:
```
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```
Terminal 4, goal (map frame, (3.0, 0.0), heading +x):
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" --feedback
```
Batch runner (full restart per run, results table, logs in `~/run_logs`, refuses to overwrite a used prefix):
```
bash scripts/run_batch.sh PREFIX COUNT HEADLESS(true|false) RVIZ(true|false)
```

## Results
Same goal every time, full restart before every run. "Runs with stops" = runs where `collision_monitor` stopped the robot at least once.

**Gate (Oct 20, inflation 0.7, window on, RViz on):** 5 passes in a row, all on the -y side of the box. Pass rule, written before the runs: `SUCCEEDED`, no touch (by eye), nothing changed since the last run.

| Run | Time | Stops |
|---|---|---|
| 1 | 11.4 s | 0 |
| 2 | 11.3 s | 0 |
| 3 | 12.2 s | 0 |
| 4 | 17.9 s | 4 |
| 5 | 12.5 s | 1 |

**Batches since then (all runs SUCCEEDED):**

| Inflation | Setup | Runs | Time | Side | Runs with stops |
|---|---|---|---|---|---|
| 0.7 | headless, no RViz | 10 | 11-12 s | -y | 0 |
| 0.7 | headless, RViz on | 10 | 10-14 s | - | 1 |
| 0.4 | headless, no RViz | 10 | 9 s | +y | 0 |
| 0.4 | window on, no RViz | 5 | 8-9 s | +y | 0 |
| 0.4 | headless, no RViz (run by `scripts/run_batch.sh`) | 10 | 8-9 s | +y | 0 |
| 0.4 | window on, RViz on (run by `scripts/run_batch.sh`) | 10 | 8-10 s | +y | 1 |

- Lowering the inflation radius from 0.7 to 0.4 flipped the route from -y to +y and saved about 2-3 s. My calculation (not checked in a costmap): at 0.7 the inflated zones of the box and the partition leave about 0.04 m between them, at 0.4 about 0.64 m.
- Headless (`-s`) raised the simulation speed from about 0.62x to about 0.84x real time (measured from raw sim and real timestamps). It did not reliably reduce stops.

## Known issues and limits
- **collision_monitor stops (cause unconfirmed):** the lidar scan is stamped 1-36 ms newer than the latest `odom -> base_footprint` transform, so the monitor treats the scan as invalid and stops the robot for 0.05-0.54 s. `transform_tolerance` is already 0.2 s, so a bigger tolerance does not explain it. It is intermittent: 3 of 6 runs on Oct 20, 2 of 55 runs since, and the setup did not predict which runs had stops.
- **Collisions were judged by eye only in the gate runs and in 5 window runs at inflation 0.4.** Headless runs and scripted batches cannot show contact.
- **Odometry drifts in sim:** `map -> odom` reached 0.37 m / 2.8 deg after a few goals with in-place turns (cause not verified).
- **Goal tolerance is 0.25 m / 0.25 rad.** In one gate run the feedback heading was about 0.27 rad while Nav2 still reported `SUCCEEDED` (cause unknown). The pose in the feedback is AMCL's estimate, not Gazebo's true pose.
- **Narrow test:** one world, one goal, one static obstacle. The controller is MPPI from Nav2's defaults and has not been compared with other controllers.
- Window-mode scripted batches crashed three times before they worked (once with a WSL restart). Cause unknown.

## How it works
- **AMCL:** AMCL works out where the robot is on the saved map. Its inputs are the lidar scan, the saved map and the odometry. It keeps many guesses of the robot's pose (the particle cloud in RViz), moves them using odometry, and scores each one by how well the scan matches the map. The output is the `map -> odom` transform: the difference between where odometry says the robot is and where the map says it is. Odometry itself still drifts, and AMCL corrects for it through this transform. In Week 5, after a few goals with in-place turns, the offset grew to 0.37 m and 2.8 degrees. A known limit I did not test: in a room where every place looks alike, AMCL can lock onto the wrong spot.
- **Costmaps and inflation:** A costmap is a grid where each cell holds the cost of the robot being there, from free to blocked. The global costmap (static layer from the saved map, obstacle layer from the lidar, inflation layer) is what the planner uses to choose the route. The local costmap is a 3 m x 3 m window that moves with the robot (voxel layer from the lidar, plus inflation), and MPPI uses it while following the route. The inflation layer adds cost around obstacles, fading with distance until the inflation radius. I changed the radius from 0.7 to 0.4 in both costmaps. The robot then went around the other side of the box (+y instead of -y), and the run was about 2-3 s faster (11-12 s to 8-9 s). My explanation, not checked in a costmap: at 0.7 the inflated zones of the box and the partition left almost no cheap gap, and at 0.4 they left about 0.6 m.
- **MPPI:** MPPI is Nav2's controller. It follows the planner's path while avoiding obstacles, and it outputs a velocity command (forward speed and turn rate). Its inputs are the robot's state, the path, the local costmap and the goal. Every cycle (20 Hz) it adds random Gaussian noise to the previous control plan to make 2000 trajectories of 56 steps x 0.05 s, so 2.8 s ahead. Critics (path following, goal, costmap cost and others) give each trajectory a total cost, and each cost becomes a trust weight, highest for the cheapest. The temperature (0.3 here) sets how sharply cost differences change the weights: low means almost only the cheapest count, high means a plain average. The new plan is the old plan plus the weighted average of the noise. The first command goes to the robot, and the rest is reused next cycle. It is Nav2's default, and I have not compared it with other controllers.
- **Odometry drift:** Odometry is like closing your eyes and counting your steps across a room. Small errors in each step add up, so odometry drifts away from the true position and would mess up localization on its own. In my test, `map -> odom` reached 0.37 m and 2.8 degrees after a few goals with in-place turns. I did not test the cause. Wheel slip is a guess. AMCL compensates by changing `map -> odom`, so the pose on the map stays right while the odometry stays wrong.
- **TF timing and `collision_monitor`:** Each lidar scan has a timestamp, and to use it `collision_monitor` needs the robot's position at that moment, from the `odom -> base_footprint` transform. Sometimes the scan is stamped newer than the newest transform available (1-110 ms in my logs), and the lookup fails with 'extrapolation into the future'. `collision_monitor` is a safety check just before the wheels. When it can't place the scan, it stops the robot for a moment (0.05-0.85 s) and continues when a later scan works. These stops were not about a real obstacle. They happened in 3 of 6 runs on Oct 20, in 2 of 55 batch runs after, and in one launch-file run with at least 14 stops. The cause is unconfirmed. One candidate is machine load: the simulation ran at about 0.62x real time with Nav2 and RViz running, and `/odom` messages arrived unevenly in one test. But headless mode and RViz off did not reliably prevent the stops. `transform_tolerance` is already 0.2 s, so a larger tolerance does not explain it.

## Files
`config/` Nav2 params, `launch/` Gazebo launch, `maps/` saved map, `urdf/` robot, `worlds/` test world, `scripts/` batch runner, `NOTES.md` lab notes, `DECISIONS.md` choices and evidence, `CLAUDE.md` project rules for Claude Code.
