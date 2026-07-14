"""Bar02 pressure → depth, as pure functions.

Ported from the proven logic in depth_and_forward.py so the gateway and the
standalone scripts agree on what "depth" means. No MAVLink here: callers hand
in plain numbers, which is what makes it testable without a robot.

Depth is metres, POSITIVE DOWN, matching MovementCommand.heave.
"""
import statistics

G = 9.80665                  # m/s^2
RHO_FRESH = 1000.0           # kg/m^3 — pool water

PRESSURE_TYPES = ('SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3')

# Preference order. SCALED_PRESSURE (instance 0) is the FMU's own baro, sealed
# inside the hull — it measures cabin air and is NOT a depth source. Only the
# external Bar02 on I2C (instance 2, occasionally 3) sees the water.
_PREFERENCE = ('SCALED_PRESSURE2', 'SCALED_PRESSURE3')

SURFACE_HPA_MIN = 900.0
SURFACE_HPA_MAX = 1100.0


def pick_pressure_type(seen):
    """Choose the external-baro message type from those actually streaming.

    Returns None when no external baro is present. Deliberately does NOT fall
    back to the hull baro: a constant "depth" that never responds to descent
    would let ALT_HOLD report a held depth while the sub sinks. No depth is a
    safe abort; a wrong depth is not.
    """
    available = set(seen)
    for mtype in _PREFERENCE:
        if mtype in available:
            return mtype
    return None


def latch_surface(samples):
    """Zero reference from surface samples, or None if there are none.

    Median, not mean — a single I2C glitch sample must not drag the reference.
    """
    values = [s for s in samples if s is not None]
    return statistics.median(values) if values else None


def surface_sane(hpa):
    """True if a surface latch is plausible sea-level-ish atmospheric pressure."""
    return SURFACE_HPA_MIN <= hpa <= SURFACE_HPA_MAX


def depth_from_pressure(press_abs_hpa, surface_hpa, rho=RHO_FRESH):
    """Metres below the latched surface. Negative when above it (bobbing) —
    not clamped, because a persistently negative depth is how a bad surface
    latch announces itself."""
    return (press_abs_hpa - surface_hpa) * 100.0 / (rho * G)
