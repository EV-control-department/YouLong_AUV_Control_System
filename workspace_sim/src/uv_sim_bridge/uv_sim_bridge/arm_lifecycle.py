"""SIL ARM policy matching the firmware heartbeat lifecycle."""


class ArmLifecycle:
    def __init__(self, heartbeat_timeout_s=1.0):
        self.armed = False
        self.arm_mode = 0
        self.last_heartbeat_s = None
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.heartbeat_count = 0
        self.arm_start_s = None

    def heartbeat(self, mode, now_s, *, origin_ready, nav_ready):
        self.check_timeout(now_s)
        self.last_heartbeat_s = now_s
        self.arm_mode = mode
        if self.heartbeat_count == 0:
            self.arm_start_s = now_s
        self.heartbeat_count += 1
        self.check(now_s, origin_ready=origin_ready, nav_ready=nav_ready)
        # mode 0 refreshes the heartbeat but is not a DISARM command.
        return self.armed

    def check(self, now_s, *, origin_ready, nav_ready):
        timed_out = self.check_timeout(now_s)
        if (not self.armed and self.heartbeat_count >= 10
                and self.arm_start_s is not None
                and now_s - self.arm_start_s >= 1.0):
            if origin_ready and (self.arm_mode == 3
                                 or (self.arm_mode == 1 and nav_ready)):
                self.armed = True
            else:
                self.heartbeat_count = 0
                self.arm_start_s = None
        return timed_out

    def check_timeout(self, now_s):
        if (self.last_heartbeat_s is not None
                and now_s - self.last_heartbeat_s > self.heartbeat_timeout_s):
            was_armed = self.armed
            self.disarm()
            return was_armed
        return False

    def reset_arming_qualification(self):
        self.heartbeat_count = 0
        self.arm_start_s = None

    def disarm(self):
        self.armed = False
        self.reset_arming_qualification()
