"""Local pose (odometry) and local -> global coordinate chain. A mathematical test here proves the
MATH, not real-world localisation accuracy (see docs/phase1_traceability.md)."""
import math

import pytest

from common.coordinates import gps_to_robot, robot_to_gps
from common.pose import LocalPose, OdometryModel, wrap_angle
from simulation.world import EntryFrame

REF = (36.8065, 10.1815)


def test_pose_starts_at_origin_facing_forward():
    o = OdometryModel()
    assert (o.pose.x, o.pose.y, o.pose.heading) == (0.0, 0.0, 0.0)


def test_forward_moves_along_heading():
    o = OdometryModel()
    o.forward(2.0)
    assert o.pose.x == pytest.approx(2.0) and o.pose.y == pytest.approx(0.0)


def test_turn_left_then_forward_differs_from_turn_right_then_forward():
    left, right = OdometryModel(), OdometryModel()
    left.turn(math.pi / 2); left.forward(1.0)
    right.turn(-math.pi / 2); right.forward(1.0)
    assert (left.pose.x, left.pose.y) == pytest.approx((0.0, 1.0), abs=1e-9)
    assert (right.pose.x, right.pose.y) == pytest.approx((0.0, -1.0), abs=1e-9)
    # ... and they map to DIFFERENT global positions
    g_l = robot_to_gps(left.pose.x, left.pose.y, *REF, 0.0)
    g_r = robot_to_gps(right.pose.x, right.pose.y, *REF, 0.0)
    assert g_l != g_r and g_l[0] > REF[0] > g_r[0]          # left = north of the reference, right = south


def test_pose_integrates_a_square_back_to_the_start():
    o = OdometryModel()
    for _ in range(4):
        o.forward(1.0); o.turn(math.pi / 2)
    assert (o.pose.x, o.pose.y) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert o.path_length_m == pytest.approx(4.0)


def test_move_by_turns_toward_the_step():
    o = OdometryModel()
    o.move_by(0.0, 0.5)
    assert o.pose.heading == pytest.approx(math.pi / 2) and o.pose.y == pytest.approx(0.5)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_noise_off_by_default_and_reproducible_when_on():
    a, b = OdometryModel(dist_noise_std=0.05, seed=3), OdometryModel(dist_noise_std=0.05, seed=3)
    for o in (a, b):
        for _ in range(20):
            o.forward(0.5)
    assert a.pose.x == b.pose.x != 20 * 0.5                  # same seed -> same drift; drift exists
    ideal = OdometryModel()
    for _ in range(20):
        ideal.forward(0.5)
    assert ideal.pose.x == pytest.approx(10.0)


def test_origin_maps_to_reference_point():
    lat, lon = robot_to_gps(0.0, 0.0, *REF, 0.0)
    assert (lat, lon) == pytest.approx(REF)


def test_translation_east_and_north():
    lat0, lon0 = REF
    lat, lon = robot_to_gps(100.0, 0.0, *REF, 0.0)
    assert lat == pytest.approx(lat0) and lon > lon0
    lat, lon = robot_to_gps(0.0, 100.0, *REF, 0.0)
    assert lat > lat0 and lon == pytest.approx(lon0)


def test_heading_rotates_the_local_frame():
    # local +x pointing NORTH (heading 90 deg ccw from east)
    lat, lon = robot_to_gps(10.0, 0.0, *REF, 90.0)
    assert lat > REF[0] and lon == pytest.approx(REF[1], abs=1e-9)


@pytest.mark.parametrize("heading", [0.0, 37.0, 90.0, 180.0, 271.5])
@pytest.mark.parametrize("pose", [(0.0, 0.0), (3.2, -1.1), (-7.5, 4.25), (12.0, 12.0)])
def test_round_trip_for_many_headings_and_poses(heading, pose):
    lat, lon = robot_to_gps(pose[0], pose[1], *REF, heading)
    x, y = gps_to_robot(lat, lon, *REF, heading)
    # robot_to_gps rounds to 7 decimal degrees (~1 cm): that is the transform's OUTPUT RESOLUTION,
    # not a localisation accuracy figure.
    assert (x, y) == pytest.approx(pose, abs=0.02)


def test_entry_frame_round_trip_and_axes():
    f = EntryFrame((1, 1))
    assert f.cell_to_local((1, 1)) == (0.0, 0.0)             # origin = the zone ENTRY (not the arena corner)
    assert f.cell_to_local((1, 5)) == (2.0, 0.0)             # +x = right
    assert f.cell_to_local((4, 1)) == (0.0, -1.5)            # +y = up, so a lower row is negative y
    for cell in [(1, 1), (3, 8), (9, 17), (5, 2)]:
        assert f.local_to_cell(*f.cell_to_local(cell)) == cell


def test_writer_beacon_coordinates_come_from_the_pose_not_the_map():
    """With odometry drift enabled the reported coordinates drift away from the true cell position:
    the Writer reports its dead-reckoned pose, not a lookup in the global grid."""
    from simulation.phase1 import build_scenario
    from simulation.system import LiveMapSystem
    sc = build_scenario("pose-test", 42, writer_max_ticks=None, odometry_noise=(0.03, 0.0))
    sys_ = LiveMapSystem(sc)
    w = sys_.deploy_writer()
    while not w.dead:
        sys_.update()
    true_xy = EntryFrame(sys_.world.entry).cell_to_local(w.route[-1])
    assert w.pose.x != true_xy[0] or w.pose.y != true_xy[1] or w.route[-1] == sys_.world.entry
    assert w.odometry.path_length_m > 0
