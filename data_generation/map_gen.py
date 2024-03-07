import numpy as np
import math
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt

class args:
    TERRAIN_SIZE = 10.0
    OBSTACLE_SIZE_RANGE = (1, 3)
    TERRAIN_POPULATION = 200
    GRAIN = 10
    GAUSS_SIGMA = 1.2
    PREFERRED_BASE_DISTANCE = 8.0
    TOLERANCE_STEP = 0.2

    OBSTACLE_VALUE = 1
    ACCESSIBLE_VALUE = 2

def clamp(value, minim=0, maxim=(round(args.TERRAIN_SIZE) * args.GRAIN) - 1):
    return max(minim, min(value, maxim))

def createBitMap(shape, obstacle_size_range=(1, 3)) -> np.ndarray:
    """ Populate a map with obstacles distributed using beta distribution (a=.6, b=.6)
    :param shape:
    :param obstacle_size_range:
    :return:
    """
    bit_map = np.zeros(shape=shape, dtype=np.uint8)

    # Spawn obstacles
    for i in range(args.TERRAIN_POPULATION):
        x = round(np.random.beta(.6, .6) * shape[0])
        y = round(np.random.beta(.6, .6) * shape[1])
        size = round(np.random.uniform(obstacle_size_range[0], obstacle_size_range[1]))

        for i in range(-size, size + 1):
            for j in range(-size, size + 1):
                bit_map[clamp(x + i, maxim=round(shape[0]) - 1)][clamp(y + j, maxim=round(shape[1]) - 1)] = args.OBSTACLE_VALUE

    return bit_map

def DFS(grid, start, goal):
    visited = [[False for _ in range(bit_map_accessible.shape[0])] for _ in range(bit_map_accessible.shape[1])]

    return dfs(grid, start[0], start[1], goal, visited)
def dfs(grid, x, y, goal, visited):
    if (x, y) == goal:
        return [goal]
    visited[x][y] = True
    best = []
    for x2, y2 in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
        if not 0 <= x2 < grid.shape[0] or not 0 <= y2 < grid.shape[1] or visited[x2][y2] or bit_map_accessible[x2][y2] != args.ACCESSIBLE_VALUE:
            continue
        distance = math.sqrt((x2 - goal[0]) ** 2 + (y2 - goal[1]) ** 2)
        if distance > math.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2):
            #TODO: gets stuck at local maximum because of this
            continue
        best.append((distance, x2, y2))
    if best:
        go_to = min(best)
    else:
        return [None]
    del best
    return [(x, y)] + dfs(grid, go_to[1], go_to[2], goal, visited)


""" Main
"""
if __name__ == "__main__":
    for t in range(1):
        bit_map = createBitMap(
            (math.ceil(args.TERRAIN_SIZE) * args.GRAIN, math.ceil(args.TERRAIN_SIZE) * args.GRAIN),
            obstacle_size_range=args.OBSTACLE_SIZE_RANGE
        )

        # Establish and highlight valid routes
        bit_map_gauss = gaussian_filter(bit_map.astype('float32'), sigma=args.GAUSS_SIGMA)
        bit_map_accessible = np.copy(bit_map)
        bit_map_accessible[bit_map_gauss < 0.1] = args.ACCESSIBLE_VALUE

        # Highlight and find valid base locations (favors open areas; NOT OPTIMAL)
        row_ratings = np.argmin(bit_map_gauss, 1)
        column_ratings = np.argmin(bit_map_gauss, 0)
        candidates = []
        for i, j in enumerate(row_ratings):
            candidates.append((i, j))
            bit_map_gauss[i][j] = args.ACCESSIBLE_VALUE
        for i, j in enumerate(column_ratings):
            candidates.append((j, i))
            bit_map_gauss[j][i] = args.ACCESSIBLE_VALUE

        best_candidates = []
        tolerance = args.PREFERRED_BASE_DISTANCE * args.GRAIN       # TODO: Randomize with favor to higher distances
        while not best_candidates:
            for i, cand1 in enumerate(candidates):
                for cand2 in candidates[i:]:
                    distance = math.sqrt((cand1[0] - cand2[0]) ** 2 + (cand1[1] - cand2[1]) ** 2)
                    if distance > tolerance:
                        best_candidates.append((cand1, cand2))

            tolerance -= args.TOLERANCE_STEP        # If candidates are not found, decrease the tolerated base distance






        plt.imshow(bit_map)
        plt.show()
        plt.imshow(bit_map_gauss)
        plt.show()
        plt.imshow(bit_map_accessible)
        plt.show()