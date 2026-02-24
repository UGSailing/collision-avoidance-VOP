from datetime import datetime
from pathlib import Path

run_dir = Path("runs") / datetime.now().strftime("%Y-%m-%d_%H%M%S")
run_dir.mkdir(parents=True)

# TODO