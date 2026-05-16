from importlib import import_module
from pathlib import Path
import sys

from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

_find_nearest_free_start_pose = import_module(
    "nav_planning_console.node"
)._find_nearest_free_start_pose


def _make_map(width, height, resolution, occupied_cells=()):
    map_msg = OccupancyGrid()
    map_msg.info.width = width
    map_msg.info.height = height
    map_msg.info.resolution = resolution
    data = [0] * (width * height)
    for cell_x, cell_y in occupied_cells:
        data[cell_y * width + cell_x] = 100
    map_msg.data = data
    return map_msg


def test_start_pose_rescue_keeps_clear_start_pose():
    map_msg = _make_map(20, 20, 0.1)
    start_pose = {"x": 1.0, "y": 1.0, "z": 0.0, "yaw": 0.5}

    adjusted_pose, distance = _find_nearest_free_start_pose(
        map_msg,
        start_pose,
        search_radius_m=0.5,
        clearance_m=0.2,
        occupied_threshold=65,
        treat_unknown_as_occupied=False,
    )

    assert adjusted_pose == start_pose
    assert distance == 0.0


def test_start_pose_rescue_moves_occupied_start_to_nearest_clear_cell():
    map_msg = _make_map(20, 20, 0.1, occupied_cells=[(10, 10)])
    start_pose = {"x": 1.05, "y": 1.05, "z": 0.0, "yaw": -0.25}

    adjusted_pose, distance = _find_nearest_free_start_pose(
        map_msg,
        start_pose,
        search_radius_m=0.5,
        clearance_m=0.2,
        occupied_threshold=65,
        treat_unknown_as_occupied=False,
    )

    assert adjusted_pose is not None
    assert adjusted_pose["yaw"] == start_pose["yaw"]
    assert distance >= 0.2
    assert distance <= 0.5


def test_start_pose_rescue_returns_none_when_no_free_cell_is_nearby():
    occupied_cells = [
        (cell_x, cell_y)
        for cell_y in range(7, 14)
        for cell_x in range(7, 14)
    ]
    map_msg = _make_map(20, 20, 0.1, occupied_cells=occupied_cells)
    start_pose = {"x": 1.05, "y": 1.05, "z": 0.0, "yaw": 0.0}

    adjusted_pose, distance = _find_nearest_free_start_pose(
        map_msg,
        start_pose,
        search_radius_m=0.2,
        clearance_m=0.1,
        occupied_threshold=65,
        treat_unknown_as_occupied=False,
    )

    assert adjusted_pose is None
    assert distance == 0.0
