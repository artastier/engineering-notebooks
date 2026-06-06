import numpy as np

class ClusterWorldGenerator:

    @staticmethod
    def generate(
            shape=(20, 20), coverage=0.25, num_seeds=4, start=(10, 2), goal=(10, 16)
    ):
        """Generates a grid map with clustered obstacles using a seed-growth approach.

        Args:
            shape (tuple): Dimensions of the grid (rows, cols).
            coverage (float): Percentage of the map to be covered by walls (0.0 to
              1.0).
            num_seeds (int): Fewer seeds create massive single chunks; more seeds
              create scattered boulders.
            start (tuple): (x, y) coordinates to protect from being blocked.
            goal (tuple): (x, y) coordinates to protect from being blocked.
        """
        grid = np.zeros(shape)
        total_cells = shape[0] * shape[1]
        target_obstacles = int(total_cells * coverage)

        # Protect immediate neighborhoods of start/goal to guarantee viability
        protected_zone = set()
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                protected_zone.add((start[0] + dx, start[1] + dy))
                protected_zone.add((goal[0] + dx, goal[1] + dy))

        # Initialize seeds
        obstacle_pool = []
        attempts = 0

        while len(obstacle_pool) < num_seeds and attempts < 100:
            rx = np.random.randint(0, shape[0])
            ry = np.random.randint(0, shape[1])
            if (rx, ry) not in protected_zone and grid[rx, ry] == 0:
                grid[rx, ry] = 1
                obstacle_pool.append((rx, ry))
            attempts += 1

        # Grow blobs until target obstacle density is satisfied
        while len(obstacle_pool) < target_obstacles:
            # Pick a random existing obstacle cell to expand from
            parent_idx = np.random.randint(len(obstacle_pool))
            cx, cy = obstacle_pool[parent_idx]

            # Select a random adjacent direction (orthogonal or diagonal)
            dx, dy = np.random.choice([-1, 0, 1]), np.random.choice([-1, 0, 1])
            nx, ny = cx + dx, cy + dy

            # Validation checks: boundary limits, protection zone, duplication check
            if 0 <= nx < shape[0] and 0 <= ny < shape[1]:
                if (nx, ny) not in protected_zone and grid[nx, ny] == 0:
                    grid[nx, ny] = 1
                    obstacle_pool.append((nx, ny))

        return grid