import pandas as pd
import numpy as np
import heapq
import config

def _get_pos_and_dest(df: pd.DataFrame) -> list[tuple] | None:
    """
    Helper function to extract current boat position and destination from the dataframe.
    """
    try:
        gps_points = df[df['category'] == 'gps']
        current_location = gps_points.loc[gps_points['id'].idxmax()]
        
        destination_points = df[df['category'] == 'destination']
        current_destination = destination_points.loc[destination_points['id'].idxmax()]
        
        return [
            (current_location['latitude'], current_location['longitude']), 
            (current_destination['latitude'], current_destination['longitude'])
        ]
    except (ValueError, KeyError):
        # return None or empty if data not found
        return None

def _heuristic(a, b):
    """Straight-line distance (Euclidean) between two grid cells."""
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def _astar(grid, start, goal):
    """Finds a path from start to goal avoiding 1.0 values (obstacles/hitboxes)."""
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: _heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
 
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start) # fix: ensure the red line visually connects to the start dot
            return data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            # boundary and obstacle check
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
 
            tentative_g_score = gscore[current] + _heuristic(current, neighbor)
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
 
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = gscore[neighbor] + _heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

def _bresenham_line(r0, c0, r1, c1):
    """Returns all grid cells along the line from (r0,c0) to (r1,c1) using Bresenham's algorithm."""
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells

def _los_clear(grid, p1, p2):
    """Returns True if no obstacle lies on the straight line between two grid points."""
    for r, c in _bresenham_line(p1[0], p1[1], p2[0], p2[1]):
        if grid[r][c] == 1:
            return False
    return True

def _prune_path_los(grid, path):
    """
    Greedily remove intermediate waypoints whenever a direct line-of-sight exists between non-adjacent path points.
    """
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _los_clear(grid, path[i], path[j]):
                break
            j -= 1
        pruned.append(path[j])
        i = j
    return pruned

def _rdp(path, epsilon):
    """
    Ramer-Douglas-Peucker algorithm: recursively removes points that deviate
    less than epsilon (grid cells) from the straight line between endpoints.
    """
    if len(path) <= 2:
        return path
    start, end = np.array(path[0], dtype=float), np.array(path[-1], dtype=float)
    line = end - start
    line_len = np.linalg.norm(line)
    if line_len == 0:
        dists = [np.linalg.norm(np.array(p, dtype=float) - start) for p in path]
    else:
        dists = [abs(np.cross(line, start - np.array(p, dtype=float))) / line_len for p in path]
    max_dist = max(dists)
    max_idx = int(np.argmax(dists))
    if max_dist > epsilon:
        left = _rdp(path[:max_idx + 1], epsilon)
        right = _rdp(path[max_idx:], epsilon)
        return left[:-1] + right
    else:
        return [path[0], path[-1]]

def smooth_path(grid, path):
    """
    Reduce the number of waypoints in a grid path.

    Two passes are applied in order:
      1. Line-of-sight (LOS) pruning - removes any waypoint that can be
         directly connected to a later point without crossing an obstacle.
      2. Ramer-Douglas-Peucker (RDP) - removes points that deviate less than
         *tolerance* grid cells from the line between their neighbours.

    Parameters
    ----------
    grid      : 2-D numpy array (0 = free, 1 = obstacle)
    path      : list of (row, col) tuples produced by A*
    tolerance : RDP epsilon in grid cells.
                0  → skip RDP, apply LOS pruning only.
                Higher values produce fewer, less accurate waypoints.
    """
    if not path or len(path) <= 2:
        return path
    path = _prune_path_los(grid, path)
    if config.SMOOTHING_TOLERANCE > 0 and len(path) > 2:
        path = _rdp(path, config.SMOOTHING_TOLERANCE)
    return path

def update_path(run_dir, grid, mapper):
    """
    Main planning function. Reads GPS points, uses A* on the grid, 
    and saves the resulting GPS path.
    """
    points_path = run_dir / 'points.csv'
    if not points_path.exists():
        return

    df = pd.read_csv(points_path)
    pts = _get_pos_and_dest(df)
    if not pts:
        return

    # use the 'mapper' passed from main.py to handle coordinate conversions
    start_idx = mapper._get_grid_indices(pts[0][0], pts[0][1])
    goal_idx = mapper._get_grid_indices(pts[1][0], pts[1][1])

    # run the A* algorithm
    path_indices = _astar(grid, start_idx, goal_idx)

    if path_indices:
        # reduce aliasing by removing redundant waypoints
        path_indices = smooth_path(grid, path_indices)

        # convert the grid path back into GPS coordinates for mapping/navigation
        path_gps = []
        center = grid.shape[0] // 2
        for r, c in path_indices:
            # revert local meters to GPS
            y_m = (r - center) * mapper.res
            x_m = (c - center) * mapper.res
            
            lat = (y_m / config.METERS_PER_DEGREE_LAT) + mapper.origin_lat
            lon = (x_m / (config.METERS_PER_DEGREE_LAT * np.cos(np.radians(mapper.origin_lat)))) + mapper.origin_lon
            path_gps.append({'latitude': lat, 'longitude': lon})
        
        # save the collision-free path
        pd.DataFrame(path_gps).to_csv(run_dir / 'path.csv', index=False)
    else:
        print("PLANNING ERROR: No valid path found. Check if hitboxes are blocking the route.")