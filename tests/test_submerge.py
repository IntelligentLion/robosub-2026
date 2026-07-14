"""SubmergeController phase sequencing. Pure — fake Effects, injected clock."""
import pytest

from control.depth_controller import DepthController
from control.heading_lock import HeadingLock, LockState
from control.pid import PID
from control.submerge import Effects, SubmergeController, SubmergeState


class FakeEffects(Effects):
    """Scriptable stand-in for the gateway services."""

    def __init__(self, preflight=(True, ''), mode=(True, ''), armed=True):
        self._preflight = preflight
        self._mode = mode
        self._armed = armed
        self.preflight_requested = 0
        self.modes_requested = []
        self._preflight_pending = True
        self._mode_pending = True

    def request_preflight(self):
        self.preflight_requested += 1
        self._preflight_pending = False

    def preflight_result(self):
        return None if self._preflight_pending else self._preflight

    def request_mode(self, name):
        self.modes_requested.append(name)
        self._mode_pending = False

    def mode_result(self):
        return None if self._mode_pending else self._mode

    def is_armed(self):
        return self._armed


def make(effects=None, **kw):
    effects = effects or FakeEffects()
    depth = DepthController(tolerance_m=0.15, min_heave=0.12, timeout_s=30.0)
    heading = HeadingLock(PID(kp=1.2, ki=0.0, kd=0.3, limit=1.0, i_limit=0.3),
                          max_yaw_authority=0.4, grace_s=1.0)
    return SubmergeController(depth, heading, effects, **kw), effects, heading


def run_to_hold(sc, depths=(0.0, 1.0, 2.0)):
    t = 0.0
    for _ in range(50):
        depth = depths[min(int(t), len(depths) - 1)]
        sc.update(depth, yaw_rad=0.5, now_s=t)
        if sc.state in (SubmergeState.HOLD, SubmergeState.FAILED):
            return t
        t += 1.0
    raise AssertionError(f'never settled — stuck in {sc.state}')


def test_happy_path_reaches_hold():
    sc, effects, _ = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    run_to_hold(sc)
    assert sc.state is SubmergeState.HOLD
    assert effects.preflight_requested == 1
    assert effects.modes_requested == ['ALT_HOLD']


def test_no_mode_is_requested_while_preflight_is_still_pending():
    # Preflight gates everything. Until it answers, we must not be reaching for
    # ALT_HOLD, let alone thrusting.
    sc, effects, _ = make()
    effects._preflight_pending_forever = True
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    effects._preflight_pending = True          # answer never arrives
    for i in range(5):
        heave, state = sc.update(0.0, 0.5, float(i))
        assert heave == 0.0
    assert effects.preflight_requested == 1
    assert effects.modes_requested == []
    assert state is SubmergeState.PREFLIGHT


def test_alt_hold_is_confirmed_before_any_heave_is_commanded():
    # The core safety ordering. If ALT_HOLD cannot be entered we must not have
    # descended a single centimetre: the failure has to be "sits at the
    # surface", never "sinks with no depth hold".
    sc, _, _ = make(effects=FakeEffects(
        mode=(False, 'Depth sensor is not connected.')))
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heaves = []
    for i in range(10):
        heave, _ = sc.update(0.0, 0.5, float(i))
        heaves.append(heave)
    assert sc.state is SubmergeState.FAILED
    assert 'Depth sensor is not connected.' in sc.failure_reason
    assert all(h == 0.0 for h in heaves)


def test_preflight_failure_aborts_and_never_requests_a_mode():
    sc, effects, _ = make(effects=FakeEffects(
        preflight=(False, 'MOT_3_DIRECTION = -1 but backup says +1')))
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    for i in range(5):
        sc.update(0.0, 0.5, float(i))
    assert sc.state is SubmergeState.FAILED
    assert 'MOT_3_DIRECTION' in sc.failure_reason
    assert effects.modes_requested == []


def test_never_armed_times_out_rather_than_hanging():
    sc, _, _ = make(effects=FakeEffects(armed=False), phase_timeout_s=5.0)
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    for i in range(20):
        sc.update(0.0, 0.5, float(i))
    assert sc.state is SubmergeState.FAILED
    assert 'arm' in sc.failure_reason.lower()


def test_heave_is_commanded_only_while_diving():
    sc, _, _ = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    seen = {}
    for i in range(12):
        depth = min(2.0, 0.4 * i)
        heave, state = sc.update(depth, 0.5, float(i))
        seen.setdefault(state, []).append(heave)
    assert all(h > 0.0 for h in seen[SubmergeState.DIVING])
    assert all(h == 0.0 for h in seen.get(SubmergeState.HOLD, [0.0]))


def test_heading_is_captured_at_depth_not_at_launch():
    # desired_heading = heading AT the target depth. Capturing it at the surface
    # would lock in whatever the sub pointed at before the dive yawed it around.
    sc, _, heading = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    sc.update(0.0, yaw_rad=0.1, now_s=0.0)    # surface heading
    sc.update(0.5, yaw_rad=0.2, now_s=1.0)
    sc.update(1.5, yaw_rad=0.3, now_s=2.0)
    sc.update(2.0, yaw_rad=1.4, now_s=3.0)    # AT DEPTH, pointing at 1.4 rad
    assert sc.state is SubmergeState.HOLD
    assert heading.state is LockState.LOCKED
    assert heading.target_yaw == pytest.approx(1.4)


def test_dive_timeout_fails_and_stops_thrusting():
    sc, _, _ = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave = None
    for i in range(40):
        heave, _ = sc.update(0.1, 0.5, float(i))   # never gets deeper
        if sc.state is SubmergeState.FAILED:
            break
    assert sc.state is SubmergeState.FAILED
    assert 'timeout' in sc.failure_reason.lower()
    assert heave == 0.0


def test_lost_depth_mid_dive_stops_thrust_without_failing_outright():
    # A dropout is not necessarily fatal — the sensor may come back. Stop
    # pushing, but keep the dive alive until the dive timeout says otherwise.
    sc, _, _ = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    for i in range(4):
        sc.update(0.2 * i, 0.5, float(i))
    heave, state = sc.update(None, 0.5, 5.0)
    assert heave == 0.0
    assert state is not SubmergeState.HOLD
    assert state is not SubmergeState.FAILED


def test_missing_yaw_at_depth_fails_rather_than_locking_a_garbage_heading():
    sc, _, heading = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    for i in range(6):
        sc.update(min(2.0, 0.5 * i), yaw_rad=None, now_s=float(i))
    assert sc.state is SubmergeState.FAILED
    assert 'yaw' in sc.failure_reason.lower()
    assert heading.state is LockState.IDLE


def test_abort_is_terminal_and_stops_thrust():
    sc, _, _ = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    sc.update(0.5, 0.5, 1.0)
    sc.abort('operator e-stop')
    heave, state = sc.update(0.5, 0.5, 2.0)
    assert state is SubmergeState.FAILED
    assert sc.failure_reason == 'operator e-stop'
    assert heave == 0.0


def test_stop_releases_the_heading_lock_and_returns_to_idle():
    sc, _, heading = make()
    sc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    run_to_hold(sc)
    sc.stop()
    assert sc.state is SubmergeState.IDLE
    assert heading.state is LockState.IDLE


def test_start_rejects_a_surface_or_negative_target():
    sc, _, _ = make()
    with pytest.raises(ValueError):
        sc.start(target_depth_m=0.0, dive_speed=0.3, now_s=0.0)
    with pytest.raises(ValueError):
        sc.start(target_depth_m=-1.0, dive_speed=0.3, now_s=0.0)
