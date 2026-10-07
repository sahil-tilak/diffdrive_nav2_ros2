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
