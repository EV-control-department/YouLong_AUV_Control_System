"""Host-side ZIT6 control-core facade used by SIL."""

from __future__ import annotations

from zit6_control_core import Zit6Controller


class Zit6Emulator:
    """Expose the firmware control core without sensor/world responsibilities."""

    def __init__(self, chassis_config=None):
        self.core = Zit6Controller(chassis_config or {})

    @property
    def control_level(self):
        return self.core.control_level

    def update(self, position, velocity, setpoint=None):
        self.core.update_nav(position, velocity)
        if setpoint is not None:
            mode, values, mask, is_body, is_incremental = setpoint
            self.core.update_setpoint(mode, values, mask, is_body, is_incremental)
        return self.core.step()

    def __getattr__(self, name):
        """Keep the firmware-core API available during incremental migration."""
        return getattr(self.core, name)
