"""Known-good thruster config, and the comparison that gates every dive.

Values copied verbatim from pixhawk_params_4.5.7_backup_2026-07-08.param (via
depth_hold_bar02_test.py, which has used them in the water). This is the one
config that decides whether a stick command maps to coherent thrust:

  * a flipped VERTICAL (5-8) makes one thruster fight the other three on the
    heave axis — the sub rolls and will not descend;
  * a flipped HORIZONTAL (1-4) turns a pure forward command into a yaw torque
    — the sub spins instead of driving straight.

Both have actually happened here (2026-07-13), caused by ad-hoc diagnostic
scripts leaving runtime param writes uncommitted. Hence: check every run, no
bypass flag, and the check is READ-ONLY. Never write these params at runtime.
"""
ALL_MOTORS = (1, 2, 3, 4, 5, 6, 7, 8)
HORIZONTAL_MOTORS = (1, 2, 3, 4)
VERTICAL_MOTORS = (5, 6, 7, 8)

_MOT_DIRECTION = {1: -1.0, 2: -1.0, 3: 1.0, 4: 1.0,
                  5: -1.0, 6: 1.0, 7: 1.0, 8: -1.0}
_SERVO_FUNCTION = {1: 33.0, 2: 34.0, 3: 35.0, 4: 36.0,
                   5: 37.0, 6: 38.0, 7: 39.0, 8: 40.0}
_SERVO_REVERSED = 0.0
_SERVO_TRIM = 1500.0
_SERVO_MIN = 1100.0
_SERVO_MAX = 1900.0

# PARAM_VALUE is a float32; exact equality would false-alarm on rounding. The
# values we care about are integers at least 1 apart, so half a unit is a
# generous tolerance that still catches every real flip.
TOLERANCE = 0.5


def expected_params(motors):
    """Flat {param_name: expected_value} for the given motor numbers."""
    exp = {}
    for m in motors:
        exp[f'MOT_{m}_DIRECTION'] = _MOT_DIRECTION[m]
        exp[f'SERVO{m}_FUNCTION'] = _SERVO_FUNCTION[m]
        exp[f'SERVO{m}_REVERSED'] = _SERVO_REVERSED
        exp[f'SERVO{m}_TRIM'] = _SERVO_TRIM
        exp[f'SERVO{m}_MIN'] = _SERVO_MIN
        exp[f'SERVO{m}_MAX'] = _SERVO_MAX
    return exp


def compare(live, expected):
    """(ok, problems). `live` maps param name → value, or None if the
    autopilot did not answer.

    Fails CLOSED: an unreadable or absent param is a problem, not a pass. We
    are gating a dive on this — "could not confirm" and "confirmed wrong" both
    mean do not dive.
    """
    problems = []
    for name, want in sorted(expected.items()):
        if name not in live:
            problems.append(f'{name}: MISSING from readback (expect {want:+.0f})')
            continue
        got = live[name]
        if got is None:
            problems.append(f'{name}: NO RESPONSE — cannot confirm '
                            f'(expect {want:+.0f})')
            continue
        if abs(got - want) >= TOLERANCE:
            problems.append(f'{name} = {got:+.0f} but backup says {want:+.0f}')
    return (not problems), problems
