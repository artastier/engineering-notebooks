import heapq

import numpy as np


class AStarOptimizer:
    @staticmethod
    def get_heuristic(p1, p2, method="manhattan"):
        if method == "manhattan":
            # TODO
            pass
        elif method == "euclidean":
            # TODO
            pass
        return 0

    @staticmethod
    def optimize(grid, start, goal, heuristic_mode):
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        close_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: AStarOptimizer.get_heuristic(start, goal, heuristic_mode)}
        o_heap = []

        heapq.heappush(o_heap, (f_score[start], start))

        while o_heap:
            current = heapq.heappop(o_heap)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            close_set.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = g_score[current] + np.sqrt(i ** 2 + j ** 2)

                if 0 <= neighbor[0] < grid.shape[0]:
                    if 0 <= neighbor[1] < grid.shape[1]:
                        if grid[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        continue
                else:
                    continue

                if neighbor in close_set and tentative_g_score >= g_score.get(
                        neighbor, 0
                ):
                    continue

                if tentative_g_score < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + AStarOptimizer.get_heuristic(
                        neighbor, goal, heuristic_mode
                    )
                    heapq.heappush(o_heap, (f_score[neighbor], neighbor))
        return []