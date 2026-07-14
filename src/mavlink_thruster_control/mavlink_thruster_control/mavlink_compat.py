"""pymavlink 2.4.49 compatibility guard. Import and install before connecting.

The bug: a MAVLink1 packet (no instance field) caches a message whose
`_instances` is None. A later MAVLink2 packet of the same type then does
`_instances[i] = msg` → TypeError. That exception propagates out of recv_match
and kills the ENTIRE receive path for the rest of the process — the symptom is
a node that connects, reports healthy, and never sees another message.

Copies of this already exist in field_common.py and pix_imu/pixhawk_imu_bridge.py
(both standalone, both outside the ROS package tree). This is the one the ROS
gateway uses.
"""
from pymavlink import mavutil

_installed = False
_orig_add_message = mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    stored = messages.get(mtype)
    if (stored is not None
            and getattr(stored, '_instances', None) is None
            and msg._instance_field is not None
            and getattr(msg, msg._instance_field, None) is not None):
        del messages[mtype]
    _orig_add_message(messages, mtype, msg)


def install_add_message_guard():
    """Idempotent. MUST be called before mavutil.mavlink_connection()."""
    global _installed
    if not _installed:
        mavutil.add_message = _safe_add_message
        _installed = True
