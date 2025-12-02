
# UGV Pathfinder Node (ROS 2, Python)

This package provides a **UGV pathfinding node** for ROS 2 written in Python.  
The node listens to a local occupancy grid map and a goal pose, computes a path in the map frame, and publishes a list of waypoints for the rover control node to follow.
Right now, it only the only path it has is for the straight line in challenge 1, but once we get the algos for the other challenges, we can ammend the code to include those
---
## 1. Node Overview

**Node name:** `ugv_pathfinder`  
**Language:** Python (`rclpy`)  
**Main file:** `ugv_pathfinder_node.py`

### Responsibilities

- Subscribe to a **local occupancy grid map**.
- Subscribe to a **goal pose**.
- Convert between **world coordinates ↔ grid indices**.
- Run a path-planning routine in grid space.
- Publish the resulting **path** as a sequence of waypoints.

The current implementation uses a **simple straight-line path** in grid space as a placeholder. It’s structured so you can easily drop in a real A* or other planner later.
## 2. Topics
### Subscribed Topics
1. **`/ugv/local_map`**
   - Type: `nav_msgs/msg/OccupancyGrid`
   - Description: Local 2D occupancy grid map around the UGV.
   - Used for: Checking free vs occupied cells during planning.
2. **`/ugv/goal`**
   - Type: `geometry_msgs/msg/PoseStamped`
   - Description: Target goal pose (position in the map frame) the UGV should navigate to.
   - Used for: End point of the planned path.
*(Optional future extension: a `/ugv/pose` topic for the current robot pose. In the template, we assume the UGV starts at `(0, 0)` in the map frame.)*
### Published Topics
1. **`/ugv/path`**
   - Type: `nav_msgs/msg/Path`
   - Description: Sequence of `PoseStamped` waypoints in the map frame from start → goal.
   - Consumers:
     - Rover / ESC control node (to convert waypoints into motor commands)
     - Logging / visualization nodes (for debugging and analysis)
---
## 3. Parameters
All parameters are declared in the node and can be overridden via your launch file.
- **`robot_radius`**
  - Type: `double`
  - Default: `0.25` (meters)
  - Intended use: Inflate obstacles or check clearance around the robot (not fully used in the simple straight-line stub, but ready for a more advanced planner).

- **`occupancy_threshold`**
  - Type: `int`
  - Default: `50`
  - Meaning: Cells with occupancy values `< threshold` are considered **free**; anything ≥ threshold (or unknown) is treated as **blocked**.
---
## 4. Behavior and Data Flow
1. **Map callback**
   - Function: `map_callback(self, msg: OccupancyGrid)`
   - Stores the latest occupancy grid in `self.latest_map`.
   - Sets a flag `self.has_map = True`.
   - Triggers `try_plan()`.

2. **Goal callback**
   - Function: `goal_callback(self, msg: PoseStamped)`
   - Stores the latest goal in `self.goal_pose`.
   - Sets `self.has_goal = True`.
   - Triggers `try_plan()`.

3. **Planning trigger**
   - Function: `try_plan(self)`
   - Checks that both a map and a goal are available.
   - Creates a start pose (currently hardcoded to `(0, 0)` in the map frame).
   - Calls `plan_path(start_pose, goal_pose, path_msg)`.

4. **Core planning**
   - Function: `plan_path(self, start, goal, out_path)`
   - Converts start and goal from world coordinates → grid indices using:
     - `world_to_grid(wx, wy)`
   - Calls the placeholder planner:
     - `simple_straight_line(sx, sy, gx, gy, grid_path)`
   - If a valid sequence of grid cells is returned, converts them back to world coordinates using:
     - `grid_to_world(ix, iy)`
   - Fills a `nav_msgs/Path` and publishes it on `/ugv/path`.

5. **Collision / map checks**
   - `is_inside_map(ix, iy)` ensures indices are within map bounds.
   - `is_cell_free(ix, iy)` checks occupancy against `occupancy_threshold`.
   - Unknown cells (`-1`) are treated as blocked in this template.
---
## 5. Placeholder Planner (Straight Line)
The function:
```python
simple_straight_line(self, sx, sy, gx, gy, out_cells)
