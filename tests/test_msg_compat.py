"""Message-compat smoke test (audit F2): catches a stale auv_msgs install
after a pull. Run under a sourced workspace."""


def test_movement_command_has_6dof_fields():
    from auv_msgs.msg import MovementCommand
    m = MovementCommand()
    for field in ('command', 'speed', 'duration', 'surge', 'strafe',
                  'heave', 'yaw_rate', 'pitch_rate', 'roll_rate'):
        assert hasattr(m, field), (
            f'MovementCommand missing "{field}" — stale auv_msgs: '
            f'colcon build --symlink-install --packages-select auv_msgs')
