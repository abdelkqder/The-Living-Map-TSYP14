"""
common/coordinates.py
=====================
Single authoritative local-to-global coordinate transformation.

The robot operates in a LOCAL frame (right-handed, metres):
  origin:  zone entry point
  x-axis:  the direction the robot faced at entry
  y-axis:  90 deg to the LEFT of the x-axis
  heading: angle of the local x-axis measured COUNTER-CLOCKWISE from geographic
           EAST, in degrees (mathematical convention).
           heading=0  -> local x points east, local y points north
           heading=90 -> local x points north, local y points west

NOTE (Phase-1 audit fix): earlier revisions of this docstring described
`heading` as a compass bearing (clockwise from north). The CODE has always
implemented the counter-clockwise-from-east convention above, and every test
(and the Writer's pose model, common/pose.py) uses it. Only the documentation
was wrong; the mathematics is unchanged.

The ONA gateway converts this to approximate GPS using the flat-earth
(equirectangular) approximation.

[ASSUMPTION] Accuracy degrades outside a ~500 m radius from the reference.
             For the competition arena this is acceptable.
[ASSUMPTION] Reference GPS and heading are measured before the mission begins.
             Errors in these propagate directly into all reported coordinates.
[IMPLEMENTED] The mathematical transformation.
[NOT VALIDATED] No field measurement has been taken to verify accuracy.
"""

from __future__ import annotations

import math
from typing import Tuple

_METRES_PER_DEG_LAT = 111_111.0


def robot_to_gps(
    x_m: float, y_m: float,
    ref_lat: float, ref_lon: float,
    heading_deg: float = 0.0,
) -> Tuple[float, float]:
    """
    Convert robot-frame coordinates (metres) to GPS (lat, lon).
    [IMPLEMENTED] [ASSUMPTION: zone <= 500 m diameter, flat terrain]
    """
    theta = math.radians(heading_deg)
    dx_east  = x_m * math.cos(theta) - y_m * math.sin(theta)
    dy_north = x_m * math.sin(theta) + y_m * math.cos(theta)
    metres_per_deg_lon = _METRES_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    lat = ref_lat + dy_north / _METRES_PER_DEG_LAT
    lon = ref_lon + dx_east / metres_per_deg_lon
    return round(lat, 7), round(lon, 7)


def gps_to_robot(
    lat: float, lon: float,
    ref_lat: float, ref_lon: float,
    heading_deg: float = 0.0,
) -> Tuple[float, float]:
    """Inverse of robot_to_gps. [IMPLEMENTED] [ASSUMPTION: same limits as robot_to_gps]"""
    metres_per_deg_lon = _METRES_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    dx_east  = (lon - ref_lon) * metres_per_deg_lon
    dy_north = (lat - ref_lat) * _METRES_PER_DEG_LAT
    theta = math.radians(heading_deg)
    x_m =  dx_east * math.cos(theta) + dy_north * math.sin(theta)
    y_m = -dx_east * math.sin(theta) + dy_north * math.cos(theta)
    return round(x_m, 3), round(y_m, 3)
