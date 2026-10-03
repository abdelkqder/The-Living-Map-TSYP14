from common.coordinates import robot_to_gps, gps_to_robot

REF_LAT, REF_LON = 36.8065, 10.1815


def test_zero_offset_returns_reference():
    lat, lon = robot_to_gps(0.0, 0.0, REF_LAT, REF_LON, 0.0)
    assert abs(lat - REF_LAT) < 1e-6
    assert abs(lon - REF_LON) < 1e-6


def test_roundtrip_zero_heading():
    x, y = 12.3, -7.8
    lat, lon = robot_to_gps(x, y, REF_LAT, REF_LON, 0.0)
    x2, y2 = gps_to_robot(lat, lon, REF_LAT, REF_LON, 0.0)
    assert abs(x - x2) < 1e-2
    assert abs(y - y2) < 1e-2


def test_roundtrip_positive_heading():
    x, y = 5.0, 5.0
    lat, lon = robot_to_gps(x, y, REF_LAT, REF_LON, 37.0)
    x2, y2 = gps_to_robot(lat, lon, REF_LAT, REF_LON, 37.0)
    assert abs(x - x2) < 1e-2
    assert abs(y - y2) < 1e-2


def test_roundtrip_negative_heading():
    x, y = -3.0, 8.0
    lat, lon = robot_to_gps(x, y, REF_LAT, REF_LON, -50.0)
    x2, y2 = gps_to_robot(lat, lon, REF_LAT, REF_LON, -50.0)
    assert abs(x - x2) < 1e-2
    assert abs(y - y2) < 1e-2


def test_north_increases_latitude():
    lat, _ = robot_to_gps(0.0, 100.0, REF_LAT, REF_LON, 0.0)
    assert lat > REF_LAT


def test_east_increases_longitude():
    _, lon = robot_to_gps(100.0, 0.0, REF_LAT, REF_LON, 0.0)
    assert lon > REF_LON
