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
