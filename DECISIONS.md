# Decisions

Format: what I chose, what else I could have chosen, why, and my own evidence. "Not tested" means exactly that.

## 2026-11-02 - Chose: `headless:=true` as an option (default off)
Alternatives: always show the Gazebo window; always run headless.
Why: the Gazebo window costs CPU. I want it off for batches, and on for watching the robot and recording the GIF.
Evidence:
- Sim speed from raw sim and real timestamps (the `real_time_factor` field matched nothing, so I ignored it): about 0.72x with Gazebo alone (window on), about 0.62x with Nav2 and RViz running (window on), about 0.84x with Nav2 and RViz running (window off).
- I first believed headless also reduced `collision_monitor` stops. Later data did not support that: since Oct 26, 2 of 55 batch runs had stops, in different setups (headless + RViz, and window + RViz), so the setup did not predict them.
Limits: headless runs cannot show a collision.
Not tested: why the stops happen.

## 2026-11-02 - Chose: `inflation_radius` 0.4 (was 0.7), local and global costmap
Alternatives: keep 0.7 (the TurtleBot3 default in my params file); other values (not run).
Why: at 0.7, my calculation says the inflated zones of the unmapped box and the partition leave about 0.04 m of cost-free gap (the physical gap is 1.44 m), so the planner took the long way around the -y side. At 0.4 that gap is about 0.64 m.
Evidence:
- 0.7: 25 counted runs (5 gate, 10 headless without RViz, 10 headless with RViz). All went -y. The clean batch took 11-12 s.
- 0.4: 35 runs (10 headless, 5 window, 10 scripted headless, 10 scripted window + RViz). All SUCCEEDED, all went +y, 8-10 s.
- Sideways excursion (AMCL poses from the feedback, map frame): about 1.01 m at 0.7, about 0.69 m at 0.4.
Risks: less margin around obstacles. Collisions were checked by eye only in the gate runs and in 5 window runs at 0.4. One world, one goal, one obstacle.
Not tested: other inflation values; narrow corridors; whether the planner chose +y because of the gap (I never looked at the costmap).

## 2026-11-02 - Chose: keep MPPI (the controller in my params file)
Alternatives: DWB; Regulated Pure Pursuit.
Why: honestly, I did not choose it. It came with Nav2's TurtleBot3 params. What I know: it samples 2000 noisy trajectories of 56 steps x 0.05 s (2.8 s ahead), scores them with critics, and averages them with weights set by a temperature (0.3). The Nav2 docs say it can use cost functions that are not convex or differentiable.
Evidence: none comparing controllers. It reached the goal in every counted run since Oct 20.
Not tested: DWB and Regulated Pure Pursuit. The Week 5 warnings ("Optimizer fail to compute path", "control loop missed its desired rate") are not explained.
Next: run the same test with Regulated Pure Pursuit, if time allows.

## 2026-11-02 - Chose: `launch/bringup.launch.py` instead of splitting the repo into packages
Alternatives: split into `diffdrive_description` and `diffdrive_bringup` (the plan's suggestion); keep the manual three-terminal start.
Why: a split moves files and changes the package name in every command, which could break the setup that passed the gate and the batches. A launch file gives a one-command start without moving anything.
Evidence: 3 test runs with the launch file reached the goal: window on (about 9 s), headless (about 9 s), window + RViz (17 s, at least 14 `collision_monitor` stops).
Limits: Nav2 starts after a fixed delay, it does not wait for Gazebo to be ready. Startup timing differs from the batch runner, so these runs are not in the results tables. `scripts/run_batch.sh` still uses the manual start.
Not done: the split. It may still make sense when the C++ packages arrive.
