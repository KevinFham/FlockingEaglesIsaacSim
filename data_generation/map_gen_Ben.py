import numpy as np
import math
import scipy.ndimage as ndi
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import heapq
import json
import jax.numpy as jnp



class args:
    show = False
    SEED = 69
    num_maps = 1
    data_set = "data/data_set"
    anwser = 'Ben_data/key'
    pieces = 'Ben_data/pieces'
    DEBUG_PYPLOT = False
    num_smap = 5
    samp_size = 100

    TERRAIN_SIZE = 50.0
    OBSTACLE_SIZE_RANGE = (1, 10)
    TERRAIN_POPULATION = 2100
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

    # Spawn obstacles
    for i in range(args.TERRAIN_POPULATION):
        x = round(np.random.beta(.6, .6) * shape[0])
        y = round(np.random.beta(.6, .6) * shape[1])
        size = round(np.random.uniform(obstacle_size_range[0], obstacle_size_range[1]))

        spawn_square(bit_map, (x, y), size, args.OBSTACLE_VALUE)

    return bit_map



def dijkstra(array, start):
    rows, cols = array.shape
    visited = np.zeros(array.shape, dtype=bool)
    distances = np.full(array.shape, np.inf)
    distances[start] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, (current_row, current_col) = heapq.heappop(priority_queue)

        if visited[current_row, current_col]:
            continue

        visited[current_row, current_col] = True

        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_row, new_col = current_row + dr, current_col + dc

            if 0 <= new_row < rows and 0 <= new_col < cols and not visited[new_row, new_col]:
                # Check if the cell is not an obstacle
                if array[new_row, new_col] != args.OBSTACLE_VALUE:
                    distance = current_distance + 1  # Assuming uniform weight for edges
                    if distance < distances[new_row, new_col]:
                        distances[new_row, new_col] = distance
                        heapq.heappush(priority_queue, (distance, (new_row, new_col)))

    return distances

def numpy_array_to_graph(array):
    rows, cols = array.shape
    graph = {}

    for i in range(rows):
        for j in range(cols):
            if array[i][j] == 0:
                graph[(i, j)] = {}
                # Check neighbors
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    x, y = i + dx, j + dy
                    if 0 <= x < rows and 0 <= y < cols and array[x][y] == 0:
                        graph[(i, j)][(x, y)] = 1  # Assuming uniform weight for edges

    return graph

def plot_grid(array, path=None):
    if args.show == True:
        plt.imshow(array, cmap='binary')
        plt.colorbar()

        # Plot path
        if path is not None:
            plt.plot([point[1] for point in path], [point[0] for point in path], color='red')

        plt.show()


def shortest_path(array, start_point, end_point):
    distances = dijkstra(array, start_point)

    # Find shortest path
    current_point = end_point
    shortest_path = [current_point]
    next_point = start_point
    while current_point != start_point:
        min_neighbor_distance = np.inf
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            neighbor_row, neighbor_col = current_point[0] + dr, current_point[1] + dc
            if 0 <= neighbor_row < 100 and 0 <= neighbor_col < 100 and distances[
                neighbor_row, neighbor_col] < min_neighbor_distance:
                min_neighbor_distance = distances[neighbor_row, neighbor_col]
                next_point = (neighbor_row, neighbor_col)
        shortest_path.append(next_point)
        current_point = next_point

    return shortest_path

def get_sample(full_arr, smapsize):    # Generate random indices to extract a 20x20 subarray
    start_row = np.random.randint(0, 500 - smapsize)  # To ensure the subarray fits within the array
    start_col = np.random.randint(0, 500 - smapsize)
    end_row = start_row + smapsize
    end_col = start_col + smapsize
    subarray = full_arr[start_row:end_row, start_col:end_col]
    print("form generation:", start_row, start_col)
    return subarray



def find_coordinates(larger_array, smaller_array):
    larger_shape = larger_array.shape
    smaller_shape = smaller_array.shape
    best_match_score = float('inf')  # Initialize with a large value
    best_match_coords = None

    for i in range(larger_shape[0] - smaller_shape[0] + 1):
        for j in range(larger_shape[1] - smaller_shape[1] + 1):
            subarray = larger_array[i:i + smaller_shape[0], j:j + smaller_shape[1]]
            mse = np.mean(np.square(subarray - smaller_array))
            confidence = 1 / (1 + mse)  # Confidence level based on inverse of MSE
            if confidence > 0.5:  # You can adjust this threshold as needed
                if mse < best_match_score:
                    best_match_score = mse
                    best_match_coords = (i, j)

    return best_match_coords, 1 / (1 + best_match_score)  # Return coordinates and confidence level

def randomly_rotate_array(array):
    num_rotations = np.random.randint(4)  # Randomly choose 0, 1, 2, or 3 rotations
    rotated_array = np.rot90(array, num_rotations)  # Rotate the array
    return rotated_array, num_rotations


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
    name = []
    for gen in range(args.num_maps):
        bit_map = BitMap(
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

        path = shortest_path(bit_map, spawn_A, spawn_B)


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
            # pass
            # mpimg.imsave(args.DATA_DIR + f'map{gen}.png', bit_map)
            # print(type(bit_map), type(path))
            plot_grid(bit_map, path)
            for i in range(args.num_smap):
                samples = []
                matches = []
                piece = get_sample(bit_map, args.samp_size)
                plot_grid(piece)
                noise = gaussian_filter(piece, 0.5)
                rot, _ = randomly_rotate_array(noise)
                match, confidence = find_coordinates(bit_map, noise)
                print(match,  confidence)
                matches.append(match)
                samples.append(piece)
            samples = np.array(samples)
            key = np.array(matches)
            # np.savez('/home/ben/Research/flockingeaglesisaacsim/data_generation/data_set/' + 'map' + str(gen + 58) + '.npz', bit_map=bit_map, path=path, section=samples, key=key)
            print('saves map number:', gen +58)



