import numpy as np


class GridAntColonyOptimizer:
    def __init__(self, grid_map, alpha=1.0, beta=2.0, evaporation_rate=0.3):
        self.grid = grid_map
        self.rows, self.cols = grid_map.shape
        self.num_nodes = self.rows * self.cols
        self.alpha = alpha
        self.beta = beta
        self.evaporation_rate = evaporation_rate
        # Node-to-node pheromone matrix
        self.pheromones = np.ones((self.num_nodes, self.num_nodes)) * 0.1

    def _coord_to_idx(self, r, c):
        return r * self.cols + c

    def _idx_to_coord(self, idx):
        return divmod(idx, self.cols)

    def get_valid_neighbors(self, current_node):
        r, c = self._idx_to_coord(current_node)
        neighbors = []
        # 8-directional movement matrix
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.grid[nr, nc] == 0: # Free space
                    neighbors.append(self._coord_to_idx(nr, nc))
        return neighbors

    def simulate_ants(self, start, goal, num_ants=10):
        start_idx = self._coord_to_idx(*start)
        goal_idx = self._coord_to_idx(*goal)
        gr, gc = goal

        paths = []
        costs = []

        for _ in range(num_ants):
            current = start_idx
            path = [current]
            visited = {current}
            stuck = False

            while current != goal_idx:
                neighbors = self.get_valid_neighbors(current)
                # Filter out already visited nodes to prevent cycles
                valid_options = [n for n in neighbors if n not in visited]

                if not valid_options:
                    stuck = True
                    break

                # Compute ACO probabilities: P ~ (pheromone^alpha) * (heuristic^beta)
                probs = []
                for n in valid_options:
                    tau = self.pheromones[current, n]
                    nr, nc = self._idx_to_coord(n)
                    # Heuristic: Inverse distance to goal
                    dist = np.sqrt((nr - gr)**2 + (nc - gc)**2)
                    eta = 1.0 / (dist + 1e-5)
                    probs.append((tau ** self.alpha) * (eta ** self.beta))

                sum_probs = sum(probs)
                if sum_probs == 0:
                    probs = np.ones(len(valid_options)) / len(valid_options)
                else:
                    probs = np.array(probs) / sum_probs

                # Choose next node
                next_node = np.random.choice(valid_options, p=probs)
                path.append(next_node)
                visited.add(next_node)
                current = next_node

                if len(path) > self.num_nodes // 2: # Fail-safe threshold
                    stuck = True
                    break

            if not stuck:
                paths.append(path)
                # Calculate cost (Euclidean path distance accumulated)
                cost = sum(
                    np.sqrt(sum((np.array(self._idx_to_coord(path[i])) - np.array(self._idx_to_coord(path[i+1])))**2))
                    for i in range(len(path)-1)
                )
                costs.append(cost)

        return paths, costs

    def update_pheromones(self, paths, costs):
        # Evaporation step
        self.pheromones *= (1.0 - self.evaporation_rate)
        # Prevent pheromones from completely disappearing
        self.pheromones = np.clip(self.pheromones, 0.01, None)

        # Intensification step
        for path, cost in zip(paths, costs):
            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                deposit = 10.0 / (cost + 1e-5)
                self.pheromones[u, v] += deposit
                self.pheromones[v, u] += deposit # Undirected link tracking

    def get_spatial_intensity(self):
        """Collapses node-link intensities into a 2D grid for mapping."""
        intensity = np.zeros((self.rows, self.cols))
        for idx in range(self.num_nodes):
            r, c = self._idx_to_coord(idx)
            intensity[r, c] = np.sum(self.pheromones[idx, :])
        return intensity