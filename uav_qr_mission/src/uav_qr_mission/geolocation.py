"""Installation geometry and WGS84 conversion. No ROS or third-party dependencies."""
import math


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
IDENTITY = (0.0, 0.0, 0.0, 1.0)


def vector(values, size):
    if not isinstance(values, (tuple, list)) or len(values) != size:
        raise ValueError('Expected {} numbers'.format(size))
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Expected finite numbers')
    return result


def normalize_quaternion(values):
    q = vector(values, 4)
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid attitude quaternion')
    return tuple(v / norm for v in q)


def multiply_quaternion(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y,
            w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W,
            w*W - x*X - y*Y - z*Z)


def quaternion_from_rpy(rpy):
    roll, pitch, yaw = vector(rpy, 3)
    sr, sp, sy = (math.sin(v / 2) for v in (roll, pitch, yaw))
    cr, cp, cy = (math.cos(v / 2) for v in (roll, pitch, yaw))
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def rotate(q, point):
    x, y, z, w = q
    px, py, pz = point
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    return (px + w*tx + y*tz-z*ty,
            py + w*ty + z*tx-x*tz,
            pz + w*tz + x*ty-y*tx)


def frame_name(value):
    if (not isinstance(value, str) or not value or value.startswith('/') or
            any(c.isspace() for c in value) or value == 'uav_qr_report_target'):
        raise ValueError('Invalid installation frame: {!r}'.format(value))
    return value


class Installation:
    def __init__(self, config):
        self.base = frame_name(config['base_frame'])
        self.camera = frame_name(config['camera_frame'])
        self.rtk = frame_name(config['rtk_frame'])
        self.edges = []
        parents = {}
        for item in config['static_transforms']:
            parent = frame_name(item.get('parent', self.base))
            child = frame_name(item['child'])
            if child == self.base or child in parents:
                raise ValueError('Duplicate or root static child: ' + child)
            xyz = vector(item['xyz'], 3)
            q = quaternion_from_rpy(item.get('rpy', [0, 0, 0]))
            parents[child] = (parent, xyz, q)
            self.edges.append((parent, child, xyz, q))
        self.transforms = {self.base: ((0.0, 0.0, 0.0), IDENTITY)}

        def resolve(child, visiting):
            if child in self.transforms:
                return self.transforms[child]
            if child in visiting or child not in parents:
                raise ValueError('Installation has a cycle or disconnected frame: ' + child)
            parent, offset, q = parents[child]
            parent_xyz, parent_q = resolve(parent, visiting | {child})
            rotated = rotate(parent_q, offset)
            result = (tuple(a+b for a, b in zip(parent_xyz, rotated)),
                      multiply_quaternion(parent_q, q))
            self.transforms[child] = result
            return result

        for child in parents:
            resolve(child, set())
        if self.camera == self.base or self.rtk == self.base or self.camera == self.rtk:
            raise ValueError('Camera and antenna must be separate installation children')
        self.camera_xyz, self.camera_q = resolve(self.camera, set())
        self.rtk_xyz, _ = resolve(self.rtk, set())

    def antenna_to_target_enu(self, camera_point, attitude):
        point = rotate(self.camera_q, vector(camera_point, 3))
        body_offset = tuple(p+c-a for p, c, a in zip(point, self.camera_xyz, self.rtk_xyz))
        return rotate(normalize_quaternion(attitude), body_offset)


def geodetic_to_ecef(latitude, longitude, altitude):
    lat, lon, h = vector([latitude, longitude, altitude], 3)
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('Invalid WGS84 latitude/longitude')
    lat, lon = math.radians(lat), math.radians(lon)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2*sin_lat*sin_lat)
    return ((n+h)*cos_lat*math.cos(lon), (n+h)*cos_lat*math.sin(lon),
            (n*(1-WGS84_E2)+h)*sin_lat)


def ecef_to_geodetic(x, y, z):
    x, y, z = vector([x, y, z], 3)
    p = math.hypot(x, y)
    if p < 1e-8:
        if abs(z) < 1e-8:
            raise ValueError('ECEF earth centre has no geodetic position')
        return (math.copysign(90.0, z), 0.0,
                abs(z) - WGS84_A*(1-WGS84_F))
    lat = math.atan2(z, p*(1-WGS84_E2))
    for _ in range(15):
        n = WGS84_A / math.sqrt(1-WGS84_E2*math.sin(lat)**2)
        updated = math.atan2(z + WGS84_E2*n*math.sin(lat), p)
        if abs(updated-lat) < 1e-14:
            lat = updated
            break
        lat = updated
    # Dot against the geodetic normal avoids dividing by cos(lat) at the poles.
    h = p*math.cos(lat) + z*math.sin(lat) - WGS84_A*math.sqrt(1-WGS84_E2*math.sin(lat)**2)
    return math.degrees(lat), math.degrees(math.atan2(y, x)), h


def offset_wgs84(latitude, longitude, altitude, enu):
    """Add a local ENU displacement at the antenna in ECEF, then invert WGS84."""
    origin = geodetic_to_ecef(latitude, longitude, altitude)
    east, north, up = vector(enu, 3)
    lat, lon = math.radians(latitude), math.radians(longitude)
    slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    delta = (-slon*east - slat*clon*north + clat*clon*up,
             clon*east - slat*slon*north + clat*slon*up,
             clat*north + slat*up)
    return ecef_to_geodetic(*(a+b for a, b in zip(origin, delta)))
