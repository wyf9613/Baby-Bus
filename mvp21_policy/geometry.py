"""MVP2.1 geometry; relocated from pinned f11428b, without algorithm tuning."""

from numbers import Real
import math


def finite_number(value):
    """Reject booleans/strings/NaN/infinity instead of turning them into actions."""
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def valid_frame(frame):
    return bool(frame) and not frame.startswith("/") and not any(c.isspace() for c in frame)


def quaternion_xyzw(quaternion):
    """ROS quaternion order is (x, y, z, w), with identity (0, 0, 0, 1)."""
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(finite_number(v) for v in values):
        raise ValueError("quaternion contains a nonfinite value")
    norm = math.hypot(*values)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion has no usable orientation")
    return tuple(v / norm for v in values)


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def quaternion_inverse(q):
    # Inputs have already been normalized, so conjugation is the inverse.
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q, vector):
    values = (vector.x, vector.y, vector.z)
    if not all(finite_number(v) for v in values):
        raise ValueError("IMU vector contains a nonfinite value")
    rotated = quaternion_product(quaternion_product(q, (*values, 0.0)), quaternion_inverse(q))
    if not all(finite_number(v) for v in rotated[:3]):
        raise ValueError("rotated IMU vector is not finite")
    return rotated[:3]


def transform_position(translation, rotation, position):
    values = (position.x, position.y, position.z)
    offsets = (translation.x, translation.y, translation.z)
    if not all(finite_number(v) for v in values + offsets):
        raise ValueError("pose position or TF translation contains a nonfinite value")
    rotated = quaternion_product(
        quaternion_product(rotation, (*values, 0.0)), quaternion_inverse(rotation))
    result = tuple(rotated[i] + offsets[i] for i in range(3))
    if not all(finite_number(v) for v in result):
        raise ValueError("transformed pose position is not finite")
    return result


def roll_pitch_yaw(q):
    x, y, z, w = q
    return (math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))),
            math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _path_points(value):
    """Validate an ordered polyline, without assuming increasing body x."""
    if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= 201:
        return None
    if any(not isinstance(p, (list, tuple)) or len(p) != 2
           or not all(finite_number(v) for v in p) for p in value):
        return None
    points = [tuple(float(v) for v in p) for p in value]
    if any(math.hypot(b[0]-a[0], b[1]-a[1]) <= 1e-6 for a, b in zip(points, points[1:])):
        return None
    return points


def _path_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    return lengths


def _path_sample(points, lengths, distance):
    distance = min(lengths[-1], max(0.0, distance))
    for i, (a, b) in enumerate(zip(points, points[1:])):
        if distance <= lengths[i+1]+1e-9:
            u = (distance-lengths[i])/(lengths[i+1]-lengths[i])
            return (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
    return points[-1]


def _path_tangent(points, lengths, distance, window=0.15):
    a = _path_sample(points, lengths, distance-window)
    b = _path_sample(points, lengths, distance+window)
    norm = math.hypot(b[0]-a[0], b[1]-a[1])
    return ((b[0]-a[0])/norm, (b[1]-a[1])/norm)


def _path_project(points, lengths, query=(0.0, 0.0), forward=True):
    """Project onto the local travel branch; a returning leg is not a shortcut.

    Forward motion cannot acquire a rear-facing segment on a cold start. The
    branch is selected in the current body frame on every tick, so a completed
    U turn naturally presents its exit as forward-facing. At ties prefer the
    earlier path segment. No global Euclidean reordering is performed.
    """
    best = None
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = lengths[i+1]-lengths[i]
        if forward and dx/length < -0.25:
            continue
        u = min(1.0, max(0.0, ((query[0]-a[0])*dx+(query[1]-a[1])*dy)/length**2))
        point = (a[0]+u*dx, a[1]+u*dy)
        cost = (point[0]-query[0])**2+(point[1]-query[1])**2
        candidate = (cost, lengths[i]+u*length, point, i, u)
        if best is None or cost < best[0]-1e-9:
            best = candidate
    return best
