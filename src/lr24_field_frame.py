#!/usr/bin/env python3

"""Shared WGS84-to-ENU field frame used by both rover computers."""

from __future__ import annotations

import math
from dataclasses import dataclass


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + altitude_m) * cos_lat * math.cos(lon)
    y = (radius + altitude_m) * cos_lat * math.sin(lon)
    z = (radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return x, y, z


@dataclass(frozen=True)
class FieldFrame:
    origin_id: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        if not 1 <= self.origin_id <= 0xFFFF:
            raise ValueError("origin_id must be in [1, 65535]")
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude out of range")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("longitude out of range")

    def to_enu(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> tuple[float, float, float]:
        origin_ecef = geodetic_to_ecef(
            self.latitude_deg, self.longitude_deg, self.altitude_m
        )
        point_ecef = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx = point_ecef[0] - origin_ecef[0]
        dy = point_ecef[1] - origin_ecef[1]
        dz = point_ecef[2] - origin_ecef[2]

        lat0 = math.radians(self.latitude_deg)
        lon0 = math.radians(self.longitude_deg)
        sin_lat = math.sin(lat0)
        cos_lat = math.cos(lat0)
        sin_lon = math.sin(lon0)
        cos_lon = math.cos(lon0)

        east = -sin_lon * dx + cos_lon * dy
        north = (
            -sin_lat * cos_lon * dx
            - sin_lat * sin_lon * dy
            + cos_lat * dz
        )
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up


def px4_ned_yaw_to_field_enu(px4_yaw_rad: float) -> float:
    """Convert clockwise-from-North NED yaw to CCW-from-East ENU yaw."""

    return wrap_pi(math.pi / 2.0 - px4_yaw_rad)


def field_enu_yaw_to_px4_ned(field_yaw_rad: float) -> float:
    return wrap_pi(math.pi / 2.0 - field_yaw_rad)


def wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi
