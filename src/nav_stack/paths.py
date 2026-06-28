"""Project root paths — shared by all nav_stack modules."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / 'src'
CONFIG_DIR = ROOT / 'config'
DATA_DIR = ROOT / 'data'

MISSION_WAYPOINTS_JSON = CONFIG_DIR / 'mission_waypoints.json'
FAKE_MOCAP_POSES_JSON = CONFIG_DIR / 'fake_mocap_poses.json'
TSP_PATH_NPY = DATA_DIR / 'tsp_path.npy'
TSP_POLYLINE_NPY = DATA_DIR / 'tsp_polyline.npy'
