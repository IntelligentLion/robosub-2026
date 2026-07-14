# ZED IMU Orientation Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone ROS 2 package that visualizes the RoboSub's orientation and motion in RViz, driven by the ZED 2i built-in IMU.

**Architecture:** The `zed-ros2-wrapper` is the IMU driver (already publishes `sensor_msgs/Imu`). A new `src/imu` package consumes that topic through three rclpy nodes — `orientation_node` (owns the TF chain `map→odom→base_link→imu_link`, quaternion drives `odom→base_link`, zeroed relative to startup), `diagnostics_node` (10 Hz text block), `marker_node` (six body-frame vector markers). A single launch file pre-kills any stale ZED node, starts a fresh camera, the three nodes, and RViz.

**Tech Stack:** ROS 2 Humble, rclpy, sensor_msgs, geometry_msgs, visualization_msgs, tf2_ros, std_srvs, pure-Python quaternion math, pytest.

## Global Constraints

- ROS 2 Humble / rclpy only. Python 3.10.
- `numpy < 2` (repo constraint) — the shared math module uses the stdlib `math` only, no numpy, so its pytest runs without a sourced workspace.
- Camera: model `zed2i`, `camera_name = zed2i`. IMU topic `/zed2i/zed_node/imu/data`, mag `/zed2i/zed_node/imu/mag`, IMU frame `zed2i_imu_link`.
- Hardware only — no simulation/synthetic fallback.
- All IMU subscriptions use `rclpy.qos.qos_profile_sensor_data` (best-effort) to match the ZED publisher.
- Build with `colcon build --symlink-install --packages-select imu` (stale `install/` is the recurring "code not taking effect" trap).
- Tests: repo `pytest.ini` sets `testpaths = tests`, `python_files = test_*.py`. New unit tests go in the root `tests/` dir named `test_*.py`. NEVER name a runnable hardware-style file `*_test.py`.
- Quaternion convention: `(x, y, z, w)` tuples, matching `geometry_msgs/Quaternion` field order. Hamilton product.

---

## File Structure

- `src/imu/package.xml` — ament_python package manifest + deps.
- `src/imu/setup.py` — entry points, launch/rviz data_files.
- `src/imu/setup.cfg` — script install dir.
- `src/imu/resource/imu` — ament resource marker (empty).
- `src/imu/imu/__init__.py` — package init.
- `src/imu/imu/imu_math.py` — pure-Python quaternion helpers (shared by all nodes; no rclpy import).
- `src/imu/imu/orientation_node.py` — TF broadcaster + zeroing + reset service + rpy publisher.
- `src/imu/imu/diagnostics_node.py` — 10 Hz text diagnostics.
- `src/imu/imu/marker_node.py` — MarkerArray of six vectors.
- `src/imu/launch/imu_viz.launch.py` — one-command bringup.
- `src/imu/rviz/imu.rviz` — RViz display config.
- `src/imu/README.md` — run instructions.
- `tests/test_imu_math.py` — pytest for the math module (root tests dir).

---

### Task 1: Package scaffold

**Files:**
- Create: `src/imu/package.xml`
- Create: `src/imu/setup.py`
- Create: `src/imu/setup.cfg`
- Create: `src/imu/resource/imu`
- Create: `src/imu/imu/__init__.py`

**Interfaces:**
- Consumes: nothing.
- Produces: a buildable ament_python package named `imu`. Entry points `orientation_node`, `diagnostics_node`, `marker_node` are declared now (pointing at modules created in later tasks) so `setup.py` is not re-edited each task.

- [ ] **Step 1: Create the ament resource marker (empty file)**

`src/imu/resource/imu`:
```
```
(zero-byte file — create it empty)

- [ ] **Step 2: Create the package init**

`src/imu/imu/__init__.py`:
```python
```
(empty file)

- [ ] **Step 3: Write `package.xml`**

`src/imu/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>imu</name>
  <version>0.0.0</version>
  <description>Standalone RViz visualization of RoboSub orientation from the ZED 2i built-in IMU.</description>
  <maintainer email="robosub@robosub.com">robosub</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `setup.cfg`**

`src/imu/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/imu
[install]
install_scripts=$base/lib/imu
```

- [ ] **Step 5: Write `setup.py`**

`src/imu/setup.py`:
```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@robosub.com',
    description='ZED 2i IMU orientation visualization for RViz.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orientation_node = imu.orientation_node:main',
            'diagnostics_node = imu.diagnostics_node:main',
            'marker_node = imu.marker_node:main',
        ],
    },
)
```

- [ ] **Step 6: Build to verify the package is recognized**

Run: `cd /home/robosub/robosub2026/robosub-2026 && colcon build --symlink-install --packages-select imu`
Expected: `Finished <<< imu` with no errors. (Entry-point modules don't exist yet; ament_python installs the console scripts lazily, so the build still succeeds.)

- [ ] **Step 7: Commit**

```bash
git add src/imu/package.xml src/imu/setup.py src/imu/setup.cfg src/imu/resource/imu src/imu/imu/__init__.py
git commit -m "feat(imu): scaffold ZED IMU visualization package"
```

---

### Task 2: Quaternion math module (TDD)

**Files:**
- Create: `src/imu/imu/imu_math.py`
- Test: `tests/test_imu_math.py`

**Interfaces:**
- Consumes: nothing (pure `math` stdlib).
- Produces, all operating on `(x, y, z, w)` float tuples:
  - `normalize(q) -> tuple` — unit quaternion; returns identity `(0,0,0,1)` if norm is ~0.
  - `quat_multiply(a, b) -> tuple` — Hamilton product `a ⊗ b`.
  - `quat_conjugate(q) -> tuple` — `(-x,-y,-z,w)`.
  - `quat_inverse(q) -> tuple` — conjugate / norm² (== conjugate for unit input).
  - `quat_relative(q_ref, q_cur) -> tuple` — `quat_inverse(q_ref) ⊗ q_cur`, the orientation of `q_cur` in `q_ref`'s frame (the zeroing operation).
  - `quat_average(quats) -> tuple` — sign-aligned componentwise mean, normalized; for startup calibration.
  - `euler_from_quat(q) -> (roll, pitch, yaw)` — radians, REP-103 XYZ, pitch clamped to avoid asin domain error (gimbal-safe).
  - `rotate_vector(q, v) -> (x,y,z)` — rotate 3-vector `v` by quaternion `q`.

- [ ] **Step 1: Write the failing tests**

`tests/test_imu_math.py`:
```python
"""Unit tests for imu.imu_math — pure quaternion helpers, no ROS needed."""
import math
import os
import sys

# imu_math lives in the (unbuilt) ROS package; import it straight from source
# so this runs without a sourced colcon workspace.
_PKG = os.path.join(os.path.dirname(__file__), '..', 'src', 'imu')
sys.path.insert(0, os.path.abspath(_PKG))

from imu.imu_math import (  # noqa: E402
    normalize, quat_multiply, quat_conjugate, quat_inverse,
    quat_relative, quat_average, euler_from_quat, rotate_vector,
)

IDENT = (0.0, 0.0, 0.0, 1.0)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_normalize_unit_stays_unit():
    assert _close(normalize(IDENT), IDENT)


def test_normalize_scales_to_unit_length():
    q = normalize((0.0, 0.0, 0.0, 2.0))
    assert _close(q, IDENT)


def test_normalize_zero_returns_identity():
    assert _close(normalize((0.0, 0.0, 0.0, 0.0)), IDENT)


def test_multiply_identity_is_noop():
    q = normalize((0.1, 0.2, 0.3, 0.9))
    assert _close(quat_multiply(q, IDENT), q)
    assert _close(quat_multiply(IDENT, q), q)


def test_conjugate_flips_vector_part():
    assert _close(quat_conjugate((0.1, 0.2, 0.3, 0.9)), (-0.1, -0.2, -0.3, 0.9))


def test_inverse_times_self_is_identity():
    q = normalize((0.1, -0.2, 0.4, 0.8))
    assert _close(quat_multiply(quat_inverse(q), q), IDENT, tol=1e-9)


def test_relative_of_equal_is_identity():
    # Zeroing: current == reference must read as no rotation.
    q = normalize((0.2, 0.1, -0.3, 0.9))
    assert _close(quat_relative(q, q), IDENT, tol=1e-9)


def test_relative_recovers_delta():
    # q_cur = q_ref ⊗ q_delta  =>  quat_relative(q_ref, q_cur) == q_delta
    q_ref = normalize((0.0, 0.0, math.sin(0.3), math.cos(0.3)))
    q_delta = normalize((0.0, 0.0, math.sin(0.5), math.cos(0.5)))
    q_cur = quat_multiply(q_ref, q_delta)
    assert _close(quat_relative(q_ref, q_cur), q_delta, tol=1e-9)


def test_average_of_identicals_is_that_quat():
    q = normalize((0.1, 0.2, 0.2, 0.95))
    assert _close(quat_average([q, q, q]), q, tol=1e-9)


def test_average_handles_sign_flips():
    # q and -q are the same rotation; averaging must not cancel to zero.
    q = normalize((0.1, 0.2, 0.2, 0.95))
    nq = tuple(-c for c in q)
    avg = quat_average([q, nq, q])
    assert _close(avg, q, tol=1e-9) or _close(avg, nq, tol=1e-9)


def test_euler_identity_is_zero():
    r, p, y = euler_from_quat(IDENT)
    assert _close((r, p, y), (0.0, 0.0, 0.0))


def test_euler_yaw_90deg():
    # +90deg about Z
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    r, p, y = euler_from_quat(q)
    assert abs(y - math.pi / 2) < 1e-6
    assert abs(r) < 1e-6 and abs(p) < 1e-6


def test_euler_pitch_clamped_at_singularity():
    # +90deg about Y — asin argument must not exceed 1.0 and blow up.
    q = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    r, p, y = euler_from_quat(q)
    assert abs(p - math.pi / 2) < 1e-6


def test_rotate_vector_identity_noop():
    assert _close(rotate_vector(IDENT, (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))


def test_rotate_vector_yaw_90_maps_x_to_y():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, z = rotate_vector(q, (1.0, 0.0, 0.0))
    assert abs(x) < 1e-6 and abs(y - 1.0) < 1e-6 and abs(z) < 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/robosub/robosub2026/robosub-2026 && python3 -m pytest tests/test_imu_math.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'imu.imu_math'` (module not created yet).

- [ ] **Step 3: Write the math module**

`src/imu/imu/imu_math.py`:
```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/robosub/robosub2026/robosub-2026 && python3 -m pytest tests/test_imu_math.py -v`
Expected: PASS — all 16 tests green.

- [ ] **Step 5: Commit**

```bash
git add src/imu/imu/imu_math.py tests/test_imu_math.py
git commit -m "feat(imu): quaternion math helpers with pytest suite"
```

---

### Task 3: orientation_node — TF chain, zeroing, reset service

**Files:**
- Create: `src/imu/imu/orientation_node.py`

**Interfaces:**
- Consumes: `imu.imu_math.{normalize, quat_relative, quat_average, euler_from_quat}`.
- Produces:
  - Subscribes `/zed2i/zed_node/imu/data` (`sensor_msgs/Imu`, `qos_profile_sensor_data`).
  - Broadcasts dynamic TF `odom → base_link` (zeroed IMU quaternion) per message.
  - Broadcasts static TF `map → odom` (identity) and `base_link → imu_link` (from `mount_rpy`) once.
  - Publishes `imu/rpy` (`geometry_msgs/Vector3Stamped`, radians, frame `base_link`).
  - Service `/imu/reset_orientation` (`std_srvs/Trigger`) → recapture reference.
  - Params: `imu_topic` (`/zed2i/zed_node/imu/data`), `parent_frame` (`odom`), `child_frame` (`base_link`), `imu_frame` (`imu_link`), `map_frame` (`map`), `calib_samples` (100), `mount_rpy` (`[0.0,0.0,0.0]`).

- [ ] **Step 1: Write the node**

`src/imu/imu/orientation_node.py`:
```python
#!/usr/bin/env python3
"""orientation_node — owns the TF tree and zeroes IMU orientation.

Subscribes the ZED 2i IMU, captures a startup reference quaternion (first
`calib_samples` averaged), and broadcasts odom->base_link as the current
orientation RELATIVE to that reference. So the sub reads level/identity at
launch and there is no cross-run accumulation. map->odom and
base_link->imu_link are static. Call /imu/reset_orientation to re-zero live.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from imu.imu_math import (
    normalize, quat_relative, quat_average, euler_from_quat,
)

# math is stdlib; used for the mount-offset rpy->quat conversion.
import math


def quat_from_euler(roll, pitch, yaw):
    """(x,y,z,w) from roll/pitch/yaw radians — REP-103 XYZ."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OrientationNode(Node):
    def __init__(self):
        super().__init__('orientation_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('calib_samples', 100)
        self.declare_parameter('mount_rpy', [0.0, 0.0, 0.0])

        self._imu_topic = self.get_parameter('imu_topic').value
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._map = self.get_parameter('map_frame').value
        self._calib_n = int(self.get_parameter('calib_samples').value)
        self._mount_rpy = list(self.get_parameter('mount_rpy').value)

        self._q_ref = None            # reference quaternion (None until calibrated)
        self._calib_buf = []          # samples collected during calibration
        self._recalibrate = False     # set by the reset service
        self._last_warn = self.get_clock().now()

        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._rpy_pub = self.create_publisher(Vector3Stamped, 'imu/rpy', 10)
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_service(
            Trigger, 'imu/reset_orientation', self._reset_cb)

        self._publish_static_tf()
        self.get_logger().info(
            f'orientation_node up — IMU topic {self._imu_topic}, '
            f'calibrating over {self._calib_n} samples')

    # ---- static transforms (map->odom identity, base_link->imu_link mount) ----
    def _publish_static_tf(self):
        now = self.get_clock().now().to_msg()
        m2o = TransformStamped()
        m2o.header.stamp = now
        m2o.header.frame_id = self._map
        m2o.child_frame_id = self._parent
        m2o.transform.rotation.w = 1.0

        b2i = TransformStamped()
        b2i.header.stamp = now
        b2i.header.frame_id = self._child
        b2i.child_frame_id = self._imu_frame
        qx, qy, qz, qw = quat_from_euler(*self._mount_rpy)
        b2i.transform.rotation.x = qx
        b2i.transform.rotation.y = qy
        b2i.transform.rotation.z = qz
        b2i.transform.rotation.w = qw
        self._static_tf.sendTransform([m2o, b2i])

    def _reset_cb(self, request, response):
        self._q_ref = None
        self._calib_buf = []
        self._recalibrate = True
        response.success = True
        response.message = 'orientation reference cleared; recalibrating'
        self.get_logger().info('reset_orientation: recalibrating reference')
        return response

    def _imu_cb(self, msg: Imu):
        q_cur = normalize((
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w))

        # --- calibration phase: gather reference, publish identity meanwhile ---
        if self._q_ref is None:
            self._calib_buf.append(q_cur)
            if len(self._calib_buf) >= self._calib_n:
                self._q_ref = quat_average(self._calib_buf)
                self._recalibrate = False
                self.get_logger().info('reference captured — orientation zeroed')
            self._broadcast((0.0, 0.0, 0.0, 1.0), msg.header.stamp)
            return

        # --- steady state: current relative to reference (the zeroing) ---
        q_zeroed = quat_relative(self._q_ref, q_cur)
        self._broadcast(q_zeroed, msg.header.stamp)

        r, p, y = euler_from_quat(q_zeroed)
        rpy = Vector3Stamped()
        rpy.header.stamp = msg.header.stamp
        rpy.header.frame_id = self._child
        rpy.vector.x, rpy.vector.y, rpy.vector.z = r, p, y
        self._rpy_pub.publish(rpy)

    def _broadcast(self, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
```

- [ ] **Step 2: Build**

Run: `cd /home/robosub/robosub2026/robosub-2026 && colcon build --symlink-install --packages-select imu`
Expected: `Finished <<< imu`.

- [ ] **Step 3: Import-smoke the entry point (no hardware)**

Run: `cd /home/robosub/robosub2026/robosub-2026 && source install/setup.bash && python3 -c "import imu.orientation_node as m; print('ok', bool(m.OrientationNode) and bool(m.quat_from_euler))"`
Expected: `ok True` (module imports, class + helper resolve; no ROS spin, no IMU needed).

- [ ] **Step 4: Commit**

```bash
git add src/imu/imu/orientation_node.py
git commit -m "feat(imu): orientation_node — TF chain, startup zeroing, reset service"
```

---

### Task 4: diagnostics_node — 10 Hz text block

**Files:**
- Create: `src/imu/imu/diagnostics_node.py`

**Interfaces:**
- Consumes: `imu.imu_math.euler_from_quat`.
- Produces: subscribes the IMU topic; prints the fixed Orientation/Quaternion/Gyroscope/Accelerometer block at 10 Hz via a timer. Params: `imu_topic` (default `/zed2i/zed_node/imu/data`).

- [ ] **Step 1: Write the node**

`src/imu/imu/diagnostics_node.py`:
```python
#!/usr/bin/env python3
"""diagnostics_node — prints IMU orientation/gyro/accel at 10 Hz.

Caches the latest ZED IMU message and renders the fixed text block on a
timer (decoupled from the IMU rate so output stays a steady 10 Hz).
Roll/pitch/yaw come from the absolute reported orientation via the shared
euler helper. Angles printed in degrees for human reading.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from imu.imu_math import euler_from_quat


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self._imu_topic = self.get_parameter('imu_topic').value
        self._last = None
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_timer(0.1, self._print)  # 10 Hz
        self.get_logger().info(
            f'diagnostics_node up — reading {self._imu_topic}')

    def _imu_cb(self, msg: Imu):
        self._last = msg

    def _print(self):
        if self._last is None:
            self.get_logger().warn('waiting for IMU data...', throttle_duration_sec=5.0)
            return
        m = self._last
        q = (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        roll, pitch, yaw = (math.degrees(a) for a in euler_from_quat(q))
        g = m.angular_velocity
        a = m.linear_acceleration
        block = (
            "\nOrientation\n"
            f"\nRoll:  {roll:8.2f} deg"
            f"\nPitch: {pitch:8.2f} deg"
            f"\nYaw:   {yaw:8.2f} deg"
            "\n\nQuaternion\n"
            f"\nx {q[0]:+.4f}"
            f"\ny {q[1]:+.4f}"
            f"\nz {q[2]:+.4f}"
            f"\nw {q[3]:+.4f}"
            "\n\nGyroscope (rad/s)\n"
            f"\nX {g.x:+.4f}"
            f"\nY {g.y:+.4f}"
            f"\nZ {g.z:+.4f}"
            "\n\nAccelerometer (m/s^2)\n"
            f"\nX {a.x:+.4f}"
            f"\nY {a.y:+.4f}"
            f"\nZ {a.z:+.4f}\n"
        )
        # print() keeps the block clean; logger would prefix every line.
        print(block, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
```

- [ ] **Step 2: Build**

Run: `cd /home/robosub/robosub2026/robosub-2026 && colcon build --symlink-install --packages-select imu`
Expected: `Finished <<< imu`.

- [ ] **Step 3: Import-smoke**

Run: `cd /home/robosub/robosub2026/robosub-2026 && source install/setup.bash && python3 -c "import imu.diagnostics_node as m; print('ok', bool(m.DiagnosticsNode))"`
Expected: `ok True`.

- [ ] **Step 4: Commit**

```bash
git add src/imu/imu/diagnostics_node.py
git commit -m "feat(imu): diagnostics_node — 10 Hz orientation/gyro/accel text block"
```

---

### Task 5: marker_node — six body-frame vectors

**Files:**
- Create: `src/imu/imu/marker_node.py`

**Interfaces:**
- Consumes: nothing from imu_math (arrows are built from raw IMU vectors in `base_link`).
- Produces: subscribes the IMU topic; publishes `imu/markers` (`visualization_msgs/MarkerArray`) at 20 Hz in frame `base_link`. Six arrows: forward(+X red), up(+Z blue), right(+Y green), gravity, angular-velocity, linear-acceleration. Params: `imu_topic`, `marker_frame` (default `base_link`).

- [ ] **Step 1: Write the node**

`src/imu/imu/marker_node.py`:
```python
#!/usr/bin/env python3
"""marker_node — RViz arrows for body axes and IMU motion vectors.

All arrows are drawn in `base_link` (the frame orientation_node rotates), so
they turn with the sub in RViz. Body axes are fixed unit arrows; gravity,
angular-velocity and linear-acceleration arrows are built from the live IMU
sample. Each arrow has a stable (ns,id) so RViz updates in place.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


def _arrow(frame, stamp, ns, mid, vec, rgb, scale=1.0):
    """One ARROW marker from origin to `vec` (scaled), colored rgb."""
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale.x = 0.02  # shaft diameter
    m.scale.y = 0.04  # head diameter
    m.scale.z = 0.06  # head length
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    m.points = [Point(x=0.0, y=0.0, z=0.0),
                Point(x=vec[0] * scale, y=vec[1] * scale, z=vec[2] * scale)]
    return m


class MarkerNode(Node):
    def __init__(self):
        super().__init__('marker_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self.declare_parameter('marker_frame', 'base_link')
        self._imu_topic = self.get_parameter('imu_topic').value
        self._frame = self.get_parameter('marker_frame').value
        self._last = None
        self._pub = self.create_publisher(MarkerArray, 'imu/markers', 10)
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._publish)  # 20 Hz
        self.get_logger().info(
            f'marker_node up — reading {self._imu_topic}, frame {self._frame}')

    def _imu_cb(self, msg: Imu):
        self._last = msg

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # fixed body axes (unit length)
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 0, (1, 0, 0), (1.0, 0.0, 0.0)))  # forward +X red
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 1, (0, 1, 0), (0.0, 1.0, 0.0)))  # right   +Y green
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 2, (0, 0, 1), (0.0, 0.0, 1.0)))  # up      +Z blue
        if self._last is not None:
            a = self._last.linear_acceleration
            g = self._last.angular_velocity
            # gravity = measured accel direction (yellow), scaled down from ~9.8
            arr.markers.append(_arrow(
                self._frame, stamp, 'gravity', 3, (a.x, a.y, a.z), (1.0, 1.0, 0.0), scale=0.1))
            # linear acceleration (magenta)
            arr.markers.append(_arrow(
                self._frame, stamp, 'accel', 4, (a.x, a.y, a.z), (1.0, 0.0, 1.0), scale=0.1))
            # angular velocity (cyan)
            arr.markers.append(_arrow(
                self._frame, stamp, 'gyro', 5, (g.x, g.y, g.z), (0.0, 1.0, 1.0), scale=1.0))
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
```

- [ ] **Step 2: Build**

Run: `cd /home/robosub/robosub2026/robosub-2026 && colcon build --symlink-install --packages-select imu`
Expected: `Finished <<< imu`.

- [ ] **Step 3: Import-smoke**

Run: `cd /home/robosub/robosub2026/robosub-2026 && source install/setup.bash && python3 -c "import imu.marker_node as m; print('ok', bool(m.MarkerNode) and bool(m._arrow))"`
Expected: `ok True`.

- [ ] **Step 4: Commit**

```bash
git add src/imu/imu/marker_node.py
git commit -m "feat(imu): marker_node — body-axis and IMU motion arrows"
```

---

### Task 6: Launch file, RViz config, README

**Files:**
- Create: `src/imu/launch/imu_viz.launch.py`
- Create: `src/imu/rviz/imu.rviz`
- Create: `src/imu/README.md`

**Interfaces:**
- Consumes: entry points `orientation_node`, `diagnostics_node`, `marker_node` (Tasks 3–5); the ZED wrapper launch `zed_camera.launch.py`.
- Produces: `ros2 launch imu imu_viz.launch.py` brings up (pre-kill stale ZED) → fresh ZED → three nodes → RViz. Args: `serial_number` (''), `camera_name` (`zed2i`), `rviz` (`true`), `start_zed` (`true`).

- [ ] **Step 1: Write the launch file**

`src/imu/launch/imu_viz.launch.py`:
```python
"""One-command bringup for the ZED IMU orientation visualization.

Order: pre-kill any stale zed_wrapper node (single-owner + fresh fused
orientation, no cross-run accumulation) -> fresh ZED camera -> orientation /
diagnostics / marker nodes -> RViz. Set start_zed:=false to attach to an
already-running ZED and skip the pre-kill.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    TimerAction, GroupAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_number = LaunchConfiguration('serial_number')
    camera_name = LaunchConfiguration('camera_name')
    use_rviz = LaunchConfiguration('rviz')
    start_zed = LaunchConfiguration('start_zed')

    imu_share = get_package_share_directory('imu')
    rviz_cfg = os.path.join(imu_share, 'rviz', 'imu.rviz')

    # ---- pre-kill any running ZED node (best-effort; ok if none) ----
    prekill = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -f zed_wrapper || true; sleep 3'],
        condition=IfCondition(start_zed),
        output='screen')

    # ---- fresh ZED camera (started after the settle) ----
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': camera_name,
            'serial_number': serial_number,
        }.items())
    zed_group = GroupAction(
        actions=[TimerAction(period=4.0, actions=[zed_launch])],
        condition=IfCondition(start_zed))

    orientation = Node(
        package='imu', executable='orientation_node',
        name='orientation_node', output='screen')
    diagnostics = Node(
        package='imu', executable='diagnostics_node',
        name='diagnostics_node', output='screen')
    markers = Node(
        package='imu', executable='marker_node',
        name='marker_node', output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        DeclareLaunchArgument('serial_number', default_value='',
                              description='ZED serial to select one of two cameras'),
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_zed', default_value='true',
                              description='false = attach to a running ZED, skip pre-kill'),
        LogInfo(msg='=== ZED IMU orientation viz starting ==='),
        prekill,
        zed_group,
        orientation,
        diagnostics,
        markers,
        rviz,
    ])
```

- [ ] **Step 2: Write the RViz config**

`src/imu/rviz/imu.rviz`:
```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Global Options:
    Fixed Frame: odom
    Background Color: 48; 48; 48
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Plane Cell Count: 10
      Cell Size: 0.5
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true
      Show Arrows: true
      Marker Scale: 0.5
    - Class: rviz_default_plugins/Axes
      Name: BaseLinkAxes
      Enabled: true
      Reference Frame: base_link
      Length: 0.4
      Radius: 0.03
    - Class: rviz_default_plugins/Imu
      Name: Imu
      Enabled: true
      Topic:
        Value: /zed2i/zed_node/imu/data
        Depth: 5
        Reliability: Best Effort
        History: Keep Last
      Box Scale: 0.3
      Acc. vector scale: 0.1
      Enable acceleration: true
    - Class: rviz_default_plugins/MarkerArray
      Name: IMU Markers
      Enabled: true
      Topic:
        Value: /imu/markers
        Depth: 5
        Reliability: Reliable
        History: Keep Last
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0
```

- [ ] **Step 3: Write the README**

`src/imu/README.md`:
```markdown
# imu — ZED 2i IMU orientation visualization

Standalone RViz visualization of the RoboSub's orientation from the ZED 2i
built-in IMU. Connects to nothing else in the stack — it owns its own TF tree
(`map -> odom -> base_link -> imu_link`) and consumes only the ZED IMU topic.

## Build

```bash
cd /home/robosub/robosub2026/robosub-2026
colcon build --symlink-install --packages-select imu
source install/setup.bash
```

## Run (one command)

```bash
ros2 launch imu imu_viz.launch.py
```

This pre-kills any running ZED node (fresh fused orientation, no accumulation),
starts a fresh `zed2i` camera, the three nodes, and RViz. Pick up the sub and
rotate it — `base_link` rotates live in RViz with axes, IMU box, and vector
arrows; the diagnostics block prints in the terminal at 10 Hz.

### Options

```bash
# select one of two ZED 2i cameras by serial
ros2 launch imu imu_viz.launch.py serial_number:=<serial>

# attach to an already-running ZED (skip pre-kill + camera bringup)
ros2 launch imu imu_viz.launch.py start_zed:=false

# no RViz (headless)
ros2 launch imu imu_viz.launch.py rviz:=false
```

## Re-zero orientation live

The orientation is zeroed to a reference captured at startup. To re-zero
(e.g. after re-mounting), call:

```bash
ros2 service call /imu/reset_orientation std_srvs/srv/Trigger {}
```

## Nodes

| Node | Output |
|------|--------|
| `orientation_node` | TF `odom->base_link` (zeroed quat), static `map->odom` + `base_link->imu_link`, `imu/rpy` |
| `diagnostics_node` | 10 Hz orientation/quaternion/gyro/accel text block |
| `marker_node` | `imu/markers` — forward/up/right/gravity/gyro/accel arrows |

## Future fusion

All output is standard ROS messages. When adding robot_localization EKF, the
`odom->base_link` transform this package publishes is exactly what the EKF will
own — swap the broadcaster for the EKF and feed BAR02 depth / DVL / mag / VSLAM
as additional EKF inputs. No changes needed to the ZED IMU driver.
```

- [ ] **Step 4: Build and verify launch + rviz files install**

Run: `cd /home/robosub/robosub2026/robosub-2026 && colcon build --symlink-install --packages-select imu && source install/setup.bash && test -f install/imu/share/imu/launch/imu_viz.launch.py && test -f install/imu/share/imu/rviz/imu.rviz && echo INSTALLED_OK`
Expected: `INSTALLED_OK`.

- [ ] **Step 5: Verify the launch file parses (no hardware start)**

Run: `cd /home/robosub/robosub2026/robosub-2026 && source install/setup.bash && ros2 launch imu imu_viz.launch.py start_zed:=false rviz:=false --print`
Expected: prints the launch description tree without error (three imu nodes listed; ZED group and pre-kill absent because `start_zed:=false`). Ctrl-C if it does not exit on its own.

- [ ] **Step 6: Commit**

```bash
git add src/imu/launch/imu_viz.launch.py src/imu/rviz/imu.rviz src/imu/README.md
git commit -m "feat(imu): one-command launch, RViz config, README"
```

---

### Task 7: On-vehicle integration verification

**Files:** none (manual verification, documented outcome).

**Interfaces:**
- Consumes: everything from Tasks 1–6, a connected ZED 2i.

- [ ] **Step 1: Launch the full stack**

Run: `cd /home/robosub/robosub2026/robosub-2026 && source install/setup.bash && ros2 launch imu imu_viz.launch.py`
Expected: pre-kill runs, ZED comes up, `orientation_node` logs "reference captured — orientation zeroed" after ~1 s, RViz opens.

- [ ] **Step 2: Confirm the TF tree**

Run (second terminal): `source install/setup.bash && ros2 run tf2_tools view_frames && echo FRAMES_OK`
Expected: `map -> odom -> base_link -> imu_link` chain present.

- [ ] **Step 3: Confirm live rotation**

Pick up the sub and rotate/tilt it. In RViz, `base_link` axes and the arrows rotate smoothly in real time. The diagnostics terminal prints the Roll/Pitch/Yaw/Quaternion/Gyro/Accel block at 10 Hz with values tracking motion.

- [ ] **Step 4: Confirm re-zero**

Hold the sub in a new attitude, run `ros2 service call /imu/reset_orientation std_srvs/srv/Trigger {}`, and confirm `base_link` snaps back to identity/level at the held pose.

- [ ] **Step 5: Record the result**

Note pass/fail of Steps 2–4 in the PR description. No commit (verification only).

---

## Self-Review

**Spec coverage:**
- IMU driver (ZED wrapper) → launch includes it (Task 6). ✓
- `sensor_msgs/Imu` + `MagneticField` → provided by ZED wrapper; consumed by nodes. ✓ (Mag not re-published — it is available on `/zed2i/zed_node/imu/mag` for future EKF; noted in spec.)
- Quaternion/angular-vel/linear-accel/timestamps/covariance → ZED wrapper; orientation math in Task 2. ✓
- 100 Hz → ZED wrapper native rate. ✓
- Roll/pitch/yaw via quaternion math, normalized, gimbal-safe → Task 2 (`euler_from_quat` asin clamp) + Task 3 (`imu/rpy`). ✓
- TF tree map→odom→base_link→imu_link → Task 3. ✓
- RViz displays (TF, IMU, Axes, Grid, orientation, markers) → Task 6 rviz. ✓
- Diagnostics 10 Hz block → Task 4. ✓
- Markers (forward/up/right/gravity/gyro/accel) → Task 5. ✓
- Launch one command → Task 6. ✓
- Reset + auto-calibrate no accumulation → Task 3 zeroing + Task 6 pre-kill. ✓
- Future fusion (standard msgs) → satisfied by design; README documents. ✓

**Placeholder scan:** No TBD/TODO in code steps (license "TODO" strings match the repo's existing convention, intentional). All code blocks complete.

**Type consistency:** `(x,y,z,w)` tuple convention used uniformly; `quat_relative(q_ref, q_cur)`, `quat_average(list)`, `euler_from_quat(q)`, `normalize(q)` names match between Task 2 definitions and Tasks 3–4 usage. `imu/markers`, `imu/rpy`, `/imu/reset_orientation`, `/zed2i/zed_node/imu/data` topic/service names consistent across tasks and rviz.
