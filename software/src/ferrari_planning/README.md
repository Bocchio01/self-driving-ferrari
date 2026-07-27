Global planner: get a map and compute a path from start to goal (RRT*, A*, Dijkstra, etc.)
Local planner: get the global path and the reading from the sensors and compute a trajectory to follow the path while avoiding obstacles that might have changed since the last time the global path was computed (DWA, MPC, etc.)
Trajectory generator: get the local path (poses only) and compute the speed profile at each pose based on geometry, curvature, and speed limits. The output is a trajectory (poses + speed profile + timestamp) that can be sent to the controller.

When driven in "line following" mode, the global planner is not used and is replaced directly by the line following algorithm that outputs a "global path" (poses only) to the local planner.
Local planner and trajectory generator are always used, regardless of the mode. The local planner is responsible for avoiding obstacles, while the trajectory generator is responsible for computing the speed profile.


For now, we can consider to use our mock global planner that loads the F1 track, and develop both the local planner and the trajectory generator.
Once these are done, we will get back to controllers and implement a look up in timeseries to get the current reference trajectory to follow.