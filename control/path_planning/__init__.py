from .v1 import get_path_points
import pandas as pd

def update_path(run_dir):
    """Recomputes the path from points.csv and write it to path.csv."""
    path = get_path_points(pd.read_csv(run_dir / 'points.csv'))
    pd.DataFrame(path, columns=['latitude', 'longitude']).to_csv(run_dir / 'path.csv', index=False)