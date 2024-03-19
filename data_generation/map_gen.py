import numpy as np
import math
import scipy.ndimage as ndi
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class args:
    SEED = 69
    DATA_GEN_SIZE = 20
    DATA_DIR = "/home/kevin/Desktop/flockingeaglesisaacsim/data_generation/data/"
    DEBUG_PYPLOT = False

    TERRAIN_SIZE = 10.0
    OBSTACLE_SIZE_RANGE = (1, 3)
    TERRAIN_POPULATION = 200
    GRAIN = 10
    GAUSS_SIGMA = 1.2
    RELATIVE_BASE_DISTANCE = 8.0
    TOLERANCE_STEP = 0.2

    OBSTACLE_VALUE = 1
    BASE_VALUE = 2
    ACCESSIBLE_VALUE = 3

np.random.seed(args.SEED)


def clamp(value, minim=0, maxim=(round(args.TERRAIN_SIZE) * args.GRAIN) - 1):
    return max(minim, min(value, maxim))

def spawn_square(grid, pos, size, val):
    for i in range(-size, size + 1):
        for j in range(-size, size + 1):
            grid[clamp(pos[0] + i, maxim=round(grid.shape[0]) - 1)][clamp(pos[1] + j, maxim=round(grid.shape[1]) - 1)] = val
def BitMap(shape, obstacle_size_range=(1, 3)) -> np.ndarray:
    """ Populate a map with obstacles distributed using beta distribution (a=.6, b=.6)
    :param shape:
    :param obstacle_size_range:
    :return:
    """
    bit_map = np.zeros(shape=shape, dtype=np.uint8)
    spawns = []

    # Spawn obstacles
    for i in range(args.TERRAIN_POPULATION):
        x = round(np.random.beta(.6, .6) * shape[0])
        y = round(np.random.beta(.6, .6) * shape[1])
        size = round(np.random.uniform(obstacle_size_range[0], obstacle_size_range[1]))

        spawn_square(bit_map, (x, y), size, args.OBSTACLE_VALUE)
        spawns.append([(x, y), size])

    return bit_map, spawns

# def DFS(grid, start, goal):
#     visited = [[False for _ in range(bit_map_accessible.shape[0])] for _ in range(bit_map_accessible.shape[1])]
#
#     return dfs(grid, start[0], start[1], goal, visited)
# def dfs(grid, x, y, goal, visited):
#     if (x, y) == goal:
#         return [goal]
#     visited[x][y] = True
#     best = []
#     for x2, y2 in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
#         if not 0 <= x2 < grid.shape[0] or not 0 <= y2 < grid.shape[1] or visited[x2][y2] or bit_map_accessible[x2][y2] != args.ACCESSIBLE_VALUE:
#             continue
#         distance = math.sqrt((x2 - goal[0]) ** 2 + (y2 - goal[1]) ** 2)
#         if distance > math.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2):
#             #gets stuck at local maximums because of this
#             continue
#         best.append((distance, x2, y2))
#     if best:
#         go_to = min(best)
#     else:
#         return [None]
#     del best
#     return [(x, y)] + dfs(grid, go_to[1], go_to[2], goal, visited)


""" Main
"""
if __name__ == "__main__":
    for gen in range(args.DATA_GEN_SIZE):
        bit_map, spawns = BitMap(
            (math.ceil(args.TERRAIN_SIZE) * args.GRAIN, math.ceil(args.TERRAIN_SIZE) * args.GRAIN),
            obstacle_size_range=args.OBSTACLE_SIZE_RANGE
        )

        # Establish and highlight valid routes
        bit_map_gauss = gaussian_filter(bit_map.astype('float32'), sigma=args.GAUSS_SIGMA)
        bit_map_accessible = np.zeros((bit_map.shape[0], bit_map.shape[1]), dtype=np.uint8)
        bit_map_accessible[bit_map_gauss < 0.1] = args.ACCESSIBLE_VALUE
        bit_map_access_labeled, n_labels = ndi.label(bit_map_accessible)

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

        # Filter through candidate locations and collect valid pairs (must meet a minimum distance and be connected)
        best_candidates = []
        tolerance = args.RELATIVE_BASE_DISTANCE * args.GRAIN       # TODO: Randomize with favor to higher distances
        while not best_candidates:
            for i, cand1 in enumerate(candidates):
                for cand2 in candidates[i:]:
                    # Check if euclidian distance is within tolerance
                    eucl_distance = math.sqrt((cand1[0] - cand2[0]) ** 2 + (cand1[1] - cand2[1]) ** 2)

                    # Connected component labelling to determine connectivity of two candidates
                    connected = bit_map_access_labeled[cand1[0]][cand1[1]] == bit_map_access_labeled[cand2[0]][cand2[1]]

                    if eucl_distance >= tolerance and connected:
                        best_candidates.append((eucl_distance, cand1, cand2))

            tolerance -= args.TOLERANCE_STEP        # If candidates are not found, decrease the tolerated base distance

        # Spawn the candidate pair with the greatest distance apart
        # _, spawn_A, spawn_B = max(best_candidates)

        # Choose a random candidate
        _, spawn_A, spawn_B = best_candidates[np.random.choice(len(best_candidates))]

        spawn_square(bit_map, spawn_A, 1, args.BASE_VALUE)
        spawn_square(bit_map, spawn_B, 1, args.BASE_VALUE)
        
        spawns.append([spawn_A, 1])
        spawns.append([spawn_B, 1])



        if args.DEBUG_PYPLOT:
            fig, axis = plt.subplots(2, 2)
            axis[0, 0].imshow(bit_map)
            axis[0, 0].axis('off')
            axis[0, 0].set_title("Final Map", fontsize=10, fontweight='bold')
            axis[0, 1].imshow(bit_map_gauss)
            axis[0, 1].axis('off')
            axis[0, 1].set_title("Gaussian w/ valid base spawns", fontsize=10, fontweight='bold')
            axis[1, 0].imshow(bit_map_accessible)
            axis[1, 0].axis('off')
            axis[1, 0].set_title("Accessibility Map", fontsize=10, fontweight='bold')
            axis[1, 1].imshow(bit_map_access_labeled)
            axis[1, 1].axis('off')
            axis[1, 1].set_title("Accessibility w/ CC Labeling", fontsize=10, fontweight='bold')
            plt.show()
        else:
            mpimg.imsave(args.DATA_DIR + f'map{gen}.png', bit_map)
            np.save(args.DATA_DIR + f'spawns{gen}', np.array(spawns, dtype=object))
