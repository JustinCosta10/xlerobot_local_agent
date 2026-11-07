import math

from nav2_msgs.msg import Costmap
import pytest

from xlerobot_navigation.destinations import ApproachGrid, DestinationError, approach_radius, relative_destination


def grid():
    message = Costmap()
    message.header.frame_id = "map"
    message.metadata.resolution = .05
    message.metadata.size_x = message.metadata.size_y = 160
    message.metadata.origin.position.x = message.metadata.origin.position.y = -4.
    message.metadata.origin.orientation.w = 1.
    message.data = [0] * (160*160)
    return ApproachGrid(message)


def test_relative_pose_uses_starting_heading_and_wraps_turn():
    goal = relative_destination(dict(x=2., y=3., yaw=math.pi/2), 1., .5, math.pi)
    assert goal == pytest.approx(dict(x=1.5, y=4., yaw=-math.pi/2))
    assert relative_destination(dict(x=0., y=0., yaw=math.pi), -1., 0., 0.)["x"] == pytest.approx(1.)


def test_clearance_encloses_rear_corner_padding_and_goal_tolerance():
    radius = approach_radius('[[-0.356,-0.252],[0.064,-0.252],[0.064,0.252],[-0.356,0.252]]', .02, .12)
    assert radius == pytest.approx(math.hypot(.376, .272) + .17)
    assert radius > .63  # A center-cell or front-only check misses the rear overhang.


@pytest.mark.parametrize("cost", [254, 255])
def test_clearance_checks_disk_interior_unknown_and_map_edges(cost):
    mapping = grid()
    assert mapping.safe(0., 0., .65)
    mapping.data[80, 85] = cost
    assert not mapping.safe(0., 0., .65)
    assert not mapping.safe(3.8, 0., .65)


def test_cell_intersection_is_conservative_and_inflation_is_not_double_counted():
    mapping = grid()
    mapping.data[:] = 100  # Inflation cost, not occupied geometry.
    assert mapping.safe(0., 0., .65)
    mapping.data[80, 92] = 254  # Cell [0.60,0.65] intersects the clearance disk.
    assert not mapping.safe(0., 0., .61)


def test_table_requires_farther_approach_and_heading_faces_object():
    mapping = grid()
    mapping.data[72:88, 102:120] = 254  # x=[1.1,2], y=[-.4,.4]
    candidates = mapping.candidates((1.5, 0.), dict(x=0., y=0.), .8, .64)
    assert candidates
    goal = candidates[0]
    assert math.hypot(goal["x"]-1.5, goal["y"]) >= 1.2-1e-6
    assert goal["yaw"] == pytest.approx(math.atan2(-goal["y"], 1.5-goal["x"]))
    assert mapping.safe(goal["x"], goal["y"], .64)


def test_no_unknown_space_is_invented_when_candidates_do_not_fit():
    mapping = grid()
    mapping.data[:] = 255
    assert mapping.candidates((1.5, 0.), dict(x=0., y=0.), .8, .64) == []
    with pytest.raises(DestinationError):
        approach_radius("not a polygon", .02, .12)
