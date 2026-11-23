#!/usr/bin/env python3


class SafetyMonitor:
    """E-stop flag, servo-communication watchdog and the status text shown to the user."""

    COMM_FAIL_LIMIT = 5            # failed servo writes in a row before auto E-stop

    def __init__(self):
        self.estopped = False
        self.reason = ""
        self.comm_fail = 0

    def trigger(self, reason: str) -> None:
        self.estopped, self.reason = True, reason

    def reset(self) -> None:
        self.estopped, self.reason, self.comm_fail = False, "", 0

    def servo_write_failed(self, results) -> bool:
        """Feed in the result of each servo write round.  True = time to E-stop."""
        self.comm_fail = 0 if all(results) else self.comm_fail + 1
        return self.comm_fail >= self.COMM_FAIL_LIMIT

    def status(self, mode: str, blocked: bool, sim_mode: bool, in_window: bool,
               workspace_msg: str = "", torque_off: bool = False):
        """Returns (text, level) with level = ok | warn | error, most urgent first."""
        if self.estopped:
            return f"EMERGENCY STOP ({self.reason})", "error"
        if workspace_msg:
            return f"WORKSPACE ALARM - {workspace_msg}", "error"
        if blocked:
            return "Joint limit (+/-70 deg) - motion blocked", "warn"
        if torque_off:
            return "TORQUE OFF - position held", "warn"
        if sim_mode:
            return "SIMULATION - arm not connected", "warn"
        if not in_window:
            return "Arm outside +/-70 deg window", "warn"
        return ("Ready" if mode == "IDLE" else "Moving"), "ok"
