import random
import pickle
from multiprocessing import Pool
import timeit
import os
import sys

sys.path.insert(0, os.path.abspath('../build'))
import multicost_pathfind_bindings as pt


def get_random_direction_val(available_dir):
    d_in = 'd' in available_dir
    r_in = 'r' in available_dir

    # influence downwards
    if (d_in and r_in):
        k = random.randint(0, 2)
        if (k == 1):
            return 'r'
        elif (k == 2):
            return 'd'
    elif (d_in):
        k = random.randint(0, 1)
        if (k == 1):
            return 'd'
    elif (r_in):
        k = random.randint(0, 1)
        if (k == 1):
            return 'r'

    rdir = available_dir[random.randint(0, len(available_dir) - 1)]
    return rdir



def direction_sat(curx, cury, curdir, maxx, maxy):
    directionS = ['r', 'l', 'd', 'u']
    maskS = [1, 1, 1, 1]

    maskS[1] = 0 # must keep going right

    if cury == maxy:
        maskS[2] = 0

    if cury == 0:
        maskS[3] = 0

    if curx == maxx:
        maskS[0] = 0

    if curx == 0:
        maskS[1] = 0

    if curx == maxx:
        maskS[0] = 0
        maskS[3] = 0

    if curdir == 'u':
        maskS[2] = 0

    if curdir == 'd':
        maskS[3] = 0

    adirS = [directionS[i] for i in range(len(maskS)) if maskS[i] == 1]
    return get_random_direction_val(adirS)



def get_random_direction_xy(curx, cury, curdir, maxx, maxy):
    rdir = direction_sat(curx, cury, curdir, maxx, maxy)

    if rdir == 'r':
        xyd = (curx + 1, cury, rdir)
    elif rdir == 'd':
        xyd = (curx, cury + 1, rdir)
    elif rdir == 'u':
        xyd = (curx, cury - 1, rdir)

    return xyd



def print_2d_array(grid):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            print(grid[i][j], end="")
        print()



def generates_path_fctc(width, height):
    # fctc = from corner to corner
    grid = [[0 for _ in range(width)] for _ in range(height)]
    x = y = 0
    curdir = '.'

    grid[y][x] = 1
    while (x != width - 1 or y != height - 1):
        x, y, curdir = get_random_direction_xy(x, y, curdir, width - 1, height - 1)
        grid[y][x] = 1
    return grid



def generate_obs_grid(width, height):
    grid = [[1 if random.randint(0, 1) == 1 else 0 for _ in range(width)] for _ in range(height)]
    return grid



def apply_path_on_obs(path, obs, width, height):
    grid = [[0 if path[j][i] == 1 else obs[j][i] for i in range(width)] for j in range(height)]
    return grid


def generate_graph_data(width, height):
    cache_file_graph = "cached_benchmark_data/gridgraph_" + str(width) + ".pkl"
    cache_file_path = "cached_benchmark_data/path_" + str(width) + ".pkl"
    
    if (os.path.exists(cache_file_graph)):
        return
    
    path = generates_path_fctc(width, height)
    obs_grid = generate_obs_grid(width, height)
    path_obs = apply_path_on_obs(path, obs_grid, width, height)

    with open(cache_file_graph, 'wb+') as f:
        pickle.dump(path_obs, f)

    with open(cache_file_path, 'wb+') as f:
        pickle.dump(path, f)


def generate_all_graph_data():
    NUM_GRAPHS = 10
    SIZE_INCREMENT = 1000

    vals = [(i * SIZE_INCREMENT, i * SIZE_INCREMENT) for i in range(1, NUM_GRAPHS + 1)]

    with Pool(10) as p:
        p.starmap(generate_graph_data, vals)


# expecting a square grid of length size
def run_benchmark_single(size):
    cached_file = "cached_benchmark_data/gridgraph_" + str(size) + ".pkl"

    with open(cached_file, 'rb') as f:
        grid = pickle.load(f)
        startx = starty = 0
        goalx = goaly = size - 1

        pt.compute(grid, size, size, startx, starty, goalx, goaly)

        with open("iter_op1_benchmark_data/nodes", 'a+') as f:
            f.write(f"{pt.num_nodes()}\n")

        with open("iter_op1_benchmark_data/multicost_allocated", 'a+') as f:
            f.write(f"{pt.multicost_allocated()}\n")

    print("COMPLETED! " + str(size))


def run_benchmark_all():
    NUM_GRAPHS = 10
    SIZE_INCREMENT = 1000

    vals = [i * SIZE_INCREMENT for i in range(1, NUM_GRAPHS + 1)]

    # im not sure if i can pool :( 
    # im worry about mutual exclusive issues since im importing things from c++
    for v in vals:
        run_benchmark_single(v)


if __name__ == "__main__":
    #generate_graph_data(100, 100)
    #generate_all_graph_data()
    run_benchmark_all()
    #run_benchmark_single(1000)