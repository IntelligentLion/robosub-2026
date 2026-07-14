"""Pure-Python quaternion helpers shared by the imu nodes.

Convention: quaternions are (x, y, z, w) tuples matching
geometry_msgs/Quaternion field order. Hamilton product. Stdlib math only
(no numpy) so this module imports and tests without a sourced workspace.
"""
import math

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def normalize(q):
    """Return the unit quaternion; identity if the norm is ~0."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return IDENTITY
    return (x / n, y / n, z / n, w / n)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_inverse(q):
    """Conjugate divided by squared norm (== conjugate for a unit quat)."""
    x, y, z, w = q
    n2 = x * x + y * y + z * z + w * w
    if n2 < 1e-12:
        return IDENTITY
    cx, cy, cz, cw = quat_conjugate(q)
    return (cx / n2, cy / n2, cz / n2, cw / n2)


def quat_multiply(a, b):
    """Hamilton product a ⊗ b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_relative(q_ref, q_cur):
    """Orientation of q_cur expressed in q_ref's frame: inv(q_ref) ⊗ q_cur.

    This is the zeroing operation — with q_cur == q_ref it returns identity.
    """
    return quat_multiply(quat_inverse(q_ref), q_cur)


def quat_average(quats):
    """Sign-aligned componentwise mean of unit quaternions, normalized.

    q and -q represent the same rotation; align every sample's sign to the
    first (via dot-product sign) before summing so opposite hemispheres do
    not cancel. Good enough for the tight cluster seen during a ~1 s
    startup hold.
    """
    if not quats:
        return IDENTITY
    ref = normalize(quats[0])
    acc = [0.0, 0.0, 0.0, 0.0]
    for q in quats:
        qn = normalize(q)
        dot = sum(a * b for a, b in zip(qn, ref))
        s = -1.0 if dot < 0.0 else 1.0
        for i in range(4):
            acc[i] += s * qn[i]
    return normalize(tuple(acc))


def euler_from_quat(q):
    """Return (roll, pitch, yaw) radians, REP-103 XYZ order.

    Pitch uses asin with the argument clamped to [-1, 1] so the vertical
    singularity (gimbal lock) yields ±pi/2 instead of a math domain error.
    """
    x, y, z, w = normalize(q)
    # roll (x-axis)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # pitch (y-axis) — clamp asin domain
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def rotate_vector(q, v):
    """Rotate 3-vector v by quaternion q: q ⊗ (v,0) ⊗ q*."""
    x, y, z, w = normalize(q)
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    r = quat_multiply(quat_multiply((x, y, z, w), qv), quat_conjugate((x, y, z, w)))
    return (r[0], r[1], r[2])
