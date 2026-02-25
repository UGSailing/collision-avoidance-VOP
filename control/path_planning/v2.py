import pandas as pd
import numpy as np
import heapq

def get_path_points(df):
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
        # Return None or empty if data isn't ready yet
        return None

def heuristic(a, b):
    """Straight-line distance (Euclidean) between two grid cells."""
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(grid, start, goal):
    """Finds a path from start to goal avoiding 1.0 values (obstacles/hitboxes)."""
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
 
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start) # FIX: Ensure the red line visually connects to the start dot
            return data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            
            # Boundary and obstacle check
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
 
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
 
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = gscore[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

def update_path(run_dir, grid, mapper):
    """
    Main planning function. Reads GPS points, uses A* on the grid, 
    and saves the resulting GPS path.
    """
    points_path = run_dir / 'points.csv'
    if not points_path.exists():
        return

    df = pd.read_csv(points_path)
    pts = get_path_points(df)
    if not pts:
        return

    # Use the 'mapper' passed from main.py to handle coordinate conversions.
    # This prevents the GPS translation offset error!
    start_idx = mapper.get_grid_indices(pts[0][0], pts[0][1])
    goal_idx = mapper.get_grid_indices(pts[1][0], pts[1][1])

    # Run the A* algorithm
    path_indices = astar(grid, start_idx, goal_idx)

    if path_indices:
        # Convert the grid path back into GPS coordinates for mapping/navigation
        path_gps = []
        center = grid.shape[0] // 2
        for r, c in path_indices:
            # Revert local meters to GPS
            y_m = (r - center) * mapper.res
            x_m = (c - center) * mapper.res
            
            lat = (y_m / 111320) + mapper.origin_lat
            lon = (x_m / (111320 * np.cos(np.radians(mapper.origin_lat)))) + mapper.origin_lon
            path_gps.append({'latitude': lat, 'longitude': lon})
        
        # Save the collision-free path
        pd.DataFrame(path_gps).to_csv(run_dir / 'path.csv', index=False)
    else:
        print("PLANNING ERROR: No valid path found. Check if hitboxes are blocking the route.")