"""set_mode readback + preflight gate. simulate=True; no serial, no vehicle."""
import rclpy
from auv_msgs.srv import SetFlightMode
from std_srvs.srv import Trigger

from mavlink_thruster_control.thruster_node import ThrusterController
from mavlink_thruster_control.thruster_params import ALL_MOTORS, expected_params


def setup_module(_m):
    rclpy.init()


def teardown_module(_m):
    rclpy.shutdown()


def make_node():
    # Built in simulation (no serial port), then flipped to the connected
    # state so the real handler paths run. The vehicle is never touched:
    # _send_set_mode is stubbed.
    node = ThrusterController(simulate=True)
    node.simulate = False
    node.connected = True
    node.master = object()
    node._send_set_mode = lambda mode_id: None
    return node


def test_set_mode_succeeds_when_heartbeat_reads_back():
    node = make_node()
    node.MODE_ACK_TIMEOUT_S = 0.2
    node._mode_name = 'ALT_HOLD'                    # heartbeat already agrees
    resp = node._on_set_mode(SetFlightMode.Request(mode='ALT_HOLD'),
                             SetFlightMode.Response())
    assert resp.success
    assert resp.reason == ''
    assert node.flight_mode_name == 'ALT_HOLD'      # watchdog now enforces it
    node.destroy_node()


def test_set_mode_fails_when_vehicle_refuses_and_returns_the_statustext():
    # A dead Bar02: ArduSub refuses ALT_HOLD, stays in MANUAL, and says why
    # only via STATUSTEXT. That reason is what the caller needs to abort on.
    node = make_node()
    node.MODE_ACK_TIMEOUT_S = 0.2
    node._mode_name = 'MANUAL'                      # never reads back

    # ArduSub emits the refusal AFTER receiving the request, so the STATUSTEXT
    # must land during the wait — _on_set_mode clears the stale one first.
    def refuse(mode_id):
        node._last_statustext = 'Depth sensor is not connected.'

    node._send_set_mode = refuse
    resp = node._on_set_mode(SetFlightMode.Request(mode='ALT_HOLD'),
                             SetFlightMode.Response())
    assert not resp.success
    assert 'Depth sensor is not connected.' in resp.reason
    node.destroy_node()


def test_set_mode_rejects_unknown_mode_without_touching_the_vehicle():
    node = make_node()
    sent = []
    node._send_set_mode = lambda mode_id: sent.append(mode_id)
    resp = node._on_set_mode(SetFlightMode.Request(mode='WARP9'),
                             SetFlightMode.Response())
    assert not resp.success
    assert 'unknown mode' in resp.reason.lower()
    assert sent == []
    node.destroy_node()


def test_set_mode_in_simulation_succeeds_without_a_vehicle():
    node = ThrusterController(simulate=True)
    resp = node._on_set_mode(SetFlightMode.Request(mode='ALT_HOLD'),
                             SetFlightMode.Response())
    assert resp.success
    node.destroy_node()


def test_preflight_passes_when_every_param_matches_the_backup():
    node = make_node()
    good = expected_params(ALL_MOTORS)
    node._read_param = lambda name, timeout=3.0: good[name]
    resp = node._on_preflight(Trigger.Request(), Trigger.Response())
    assert resp.success
    node.destroy_node()


def test_preflight_fails_on_a_flipped_horizontal_thruster():
    # MOT_3_DIRECTION flipped: a pure forward command becomes a yaw torque and
    # the sub spins. Must never reach the water.
    node = make_node()
    bad = dict(expected_params(ALL_MOTORS))
    bad['MOT_3_DIRECTION'] = -1.0
    node._read_param = lambda name, timeout=3.0: bad[name]
    resp = node._on_preflight(Trigger.Request(), Trigger.Response())
    assert not resp.success
    assert 'MOT_3_DIRECTION' in resp.message
    node.destroy_node()


def test_preflight_fails_closed_when_a_param_cannot_be_read():
    node = make_node()
    node._read_param = lambda name, timeout=3.0: None
    resp = node._on_preflight(Trigger.Request(), Trigger.Response())
    assert not resp.success
    assert 'NO RESPONSE' in resp.message
    node.destroy_node()


def test_preflight_never_writes_params():
    # Read-only by policy: uncommitted runtime param writes from ad-hoc scripts
    # are what flipped a vertical thruster on 2026-07-13.
    node = make_node()
    good = expected_params(ALL_MOTORS)
    reads = []

    def spy(name, timeout=3.0):
        reads.append(name)
        return good[name]

    node._read_param = spy
    node._on_preflight(Trigger.Request(), Trigger.Response())
    assert len(reads) == len(good)
    assert not hasattr(node, '_write_param')
    node.destroy_node()
