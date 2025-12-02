import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class UgvPathfinderNode(Node):
    def __init__(self) -> None:
        super().__init__('ugv_pathfinder')

        # Parameters you can tune later
        self.declare_parameter('robot_radius', 0.25)           # meters
        self.declare_parameter('occupancy_threshold', 50)      # 0–100

        self.robot_radius = self.get_parameter(
            'robot_radius').get_parameter_value().double_value
        self.occupancy_threshold = self.get_parameter(
            'occupancy_threshold').get_parameter_value().integer_value

        # Internal state
        self.latest_map: OccupancyGrid | None = None
        self.has_map: bool = False
        self.goal_pose: PoseStamped | None = None
        self.has_goal: bool = False

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/ugv/local_map',
            self.map_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/ugv/goal',
            self.goal_callback,
            10
        )

        # Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/ugv/path',
            10
        )

        self.get_logger().info('UGV Pathfinder node started.')

    # Callback for the map
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.has_map = True

        self.get_logger().info(
            f'Received map: {msg.info.width} x {msg.info.height}'
        )

        self.try_plan()

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg
        self.has_goal = True

        self.get_logger().info(
            f'Received new goal: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        self.try_plan()

    # Try to plan whenever we have both map and goal
    def try_plan(self) -> None:
        if not (self.has_map and self.has_goal and self.latest_map and self.goal_pose):
            return

        # For now, assume UGV starts at (0,0) in map frame.
        # Later, you can subscribe to /ugv/pose instead.
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.latest_map.header.frame_id
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.orientation.w = 1.0

        path_msg = Path()
        success = self.plan_path(start_pose, self.goal_pose, path_msg)

        if not success:
            self.get_logger().warn('Path planning failed (no path found).')
            return

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published path with {len(path_msg.poses)} waypoints.'
        )

    #Core planning function 
    def plan_path(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        out_path: Path
    ) -> bool:
        if not (self.latest_map and self.has_map):
            return False

        # Convert world → grid
        start_ix, start_iy = self.world_to_grid(
            start.pose.position.x,
            start.pose.position.y
        )
        goal_ix, goal_iy = self.world_to_grid(
            goal.pose.position.x,
            goal.pose.position.y
        )

        if start_ix is None or goal_ix is None:
            self.get_logger().warn('Start or goal is outside of map bounds.')
            return False

        # Placeholder planner: straight line in the occupancy grid
        grid_path: List[Tuple[int, int]] = []
        ok = self.simple_straight_line(
            start_ix, start_iy, goal_ix, goal_iy, grid_path
        )

        if not ok:
            self.get_logger().warn(
                'simple_straight_line failed (blocked or out of bounds).'
            )
            return False

        # Fill nav_msgs/Path
        out_path.header.stamp = self.get_clock().now().to_msg()
        out_path.header.frame_id = self.latest_map.header.frame_id

        for ix, iy in grid_path:
            pose = PoseStamped()
            pose.header = out_path.header

            wx, wy = self.grid_to_world(ix, iy)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0

            out_path.poses.append(pose)

        return True

    # === Utility functions ===

    def world_to_grid(self, wx: float, wy: float) -> tuple[int | None, int | None]:
        """Convert world (meters) → grid indices (cells)."""
        if not self.latest_map:
            return None, None

        info = self.latest_map.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution

        ix = int((wx - origin_x) / res)
        iy = int((wy - origin_y) / res)

        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return None, None

        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        """Convert grid indices (cells) → world (meters)."""
        info = self.latest_map.info  # type: ignore
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution

        wx = origin_x + (ix + 0.5) * res
        wy = origin_y + (iy + 0.5) * res
        return wx, wy

    def is_inside_map(self, ix: int, iy: int) -> bool:
        info = self.latest_map.info  # type: ignore
        return 0 <= ix < info.width and 0 <= iy < info.height

    def is_cell_free(self, ix: int, iy: int) -> bool:
        """Return True if cell is considered free."""
        info = self.latest_map.info  # type: ignore
        index = iy * info.width + ix
        occ = self.latest_map.data[index]  # type: ignore

        # -1 = unknown, 0 = free, 100 = occupied (convention)
        if occ < 0:
            # you can choose to treat unknown as free or occupied
            return False

        return occ < self.occupancy_threshold

    def simple_straight_line(
        self,
        sx: int,
        sy: int,
        gx: int,
        gy: int,
        out_cells: List[Tuple[int, int]]
    ) -> bool:
        """
        Extremely simple "pathfinder":
        straight line between start and goal in grid space.
        This is just a placeholder for a real A* implementation.
        """
        out_cells.clear()

        dx = gx - sx
        dy = gy - sy
        steps = max(abs(dx), abs(dy))

        if steps == 0:
            if self.is_inside_map(sx, sy) and self.is_cell_free(sx, sy):
                out_cells.append((sx, sy))
                return True
            return False

        x = float(sx)
        y = float(sy)
        step_x = dx / float(steps)
        step_y = dy / float(steps)

        for _ in range(steps + 1):
            ix = int(round(x))
            iy = int(round(y))

            if not self.is_inside_map(ix, iy) or not self.is_cell_free(ix, iy):
                return False  # hit obstacle or left map

            out_cells.append((ix, iy))
            x += step_x
            y += step_y

        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvPathfinderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
