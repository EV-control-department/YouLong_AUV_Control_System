"""Pure A* path planning algorithm. No ROS dependency."""

import heapq
import math


class AStarPlanner:
    """Grid-based A* path planner with obstacle inflation."""

    def __init__(self, resolution: float = 0.5, safe_radius: float = 2.0):
        self.resolution = resolution
        self.safe_radius = safe_radius

    def plan(self, start_x: float, start_y: float,
             goal_x: float, goal_y: float,
             obstacles: list) -> tuple:
        """Plan path from start to goal avoiding obstacles.

        Args:
            start_x, start_y: Start position (world NED)
            goal_x, goal_y: Goal position (world NED)
            obstacles: List of (x, y) obstacle positions

        Returns:
            (next_x, next_y): Next waypoint at least 1.5m ahead.
            Returns (goal_x, goal_y) if path is clear or no path found.
        """
        # Check if direct path is clear
        if self._path_clear(start_x, start_y, goal_x, goal_y, obstacles):
            return goal_x, goal_y

        # Build grid bounds
        all_x = [start_x, goal_x] + [o[0] for o in obstacles]
        all_y = [start_y, goal_y] + [o[1] for o in obstacles]
        margin = self.safe_radius + 5.0
        min_x = min(all_x) - margin
        max_x = max(all_x) + margin
        min_y = min(all_y) - margin
        max_y = max(all_y) + margin

        # Grid dimensions
        nx = int((max_x - min_x) / self.resolution) + 1
        ny = int((max_y - min_y) / self.resolution) + 1

        # Build obstacle grid
        obstacle_grid = set()
        for ox, oy in obstacles:
            r = self.safe_radius / self.resolution
            cx = int((ox - min_x) / self.resolution)
            cy = int((oy - min_y) / self.resolution)
            for dx in range(-int(r) - 1, int(r) + 2):
                for dy in range(-int(r) - 1, int(r) + 2):
                    if dx * dx + dy * dy <= r * r:
                        gx, gy = cx + dx, cy + dy
                        if 0 <= gx < nx and 0 <= gy < ny:
                            obstacle_grid.add((gx, gy))

        # Start and goal in grid coordinates
        sx = int((start_x - min_x) / self.resolution)
        sy = int((start_y - min_y) / self.resolution)
        gx = int((goal_x - min_x) / self.resolution)
        gy = int((goal_y - min_y) / self.resolution)

        # A* search
        # 8 directions
        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if cx == gx and cy == gy:
                # Reconstruct path
                path = []
                node = (gx, gy)
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.reverse()

                # Find first waypoint > 1.5m from start
                min_dist = 1.5 / self.resolution
                for px, py in path:
                    dist = math.sqrt((px - sx) ** 2 + (py - sy) ** 2)
                    if dist >= min_dist:
                        world_x = px * self.resolution + min_x
                        world_y = py * self.resolution + min_y
                        return world_x, world_y

                # Path too short, return goal
                return goal_x, goal_y

            for dx, dy, cost in directions:
                nx_g, ny_g = cx + dx, cy + dy
                if nx_g < 0 or nx_g >= nx or ny_g < 0 or ny_g >= ny:
                    continue
                if (nx_g, ny_g) in obstacle_grid:
                    continue

                tentative = g_score[(cx, cy)] + cost
                if (nx_g, ny_g) not in g_score or tentative < g_score[(nx_g, ny_g)]:
                    g_score[(nx_g, ny_g)] = tentative
                    h = math.sqrt((nx_g - gx) ** 2 + (ny_g - gy) ** 2)
                    f = tentative + h
                    heapq.heappush(open_set, (f, nx_g, ny_g))
                    came_from[(nx_g, ny_g)] = (cx, cy)

        # No path found
        return goal_x, goal_y

    def _path_clear(self, x1: float, y1: float, x2: float, y2: float,
                    obstacles: list) -> bool:
        """Check if direct path between two points is clear of obstacles."""
        for ox, oy in obstacles:
            # Point-to-segment distance
            dx = x2 - x1
            dy = y2 - y1
            len_sq = dx * dx + dy * dy
            if len_sq < 1e-10:
                dist = math.sqrt((ox - x1) ** 2 + (oy - y1) ** 2)
            else:
                t = max(0, min(1, ((ox - x1) * dx + (oy - y1) * dy) / len_sq))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                dist = math.sqrt((ox - proj_x) ** 2 + (oy - proj_y) ** 2)

            if dist < self.safe_radius:
                return False
        return True
