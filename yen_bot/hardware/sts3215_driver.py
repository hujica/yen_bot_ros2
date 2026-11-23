#!/usr/bin/env python3
"""Feetech STS3215 bus-servo driver (serial communication) with a simulation fallback."""
from .servo_map import ServoMap


class STS3215Driver:
    # control-table addresses
    ADDR_TORQUE_ENABLE = 40        # 1 byte
    ADDR_GOAL_POSITION = 42        # 2 bytes
    ADDR_GOAL_SPEED = 46           # 2 bytes
    ADDR_PRESENT_POSITION = 56     # 2 bytes

    SERVO_IDS = (1, 2, 3, 4)

    # speed scaling: one full turn = 4096 steps
    COUNTS_PER_REV = 4096
    MAX_SPEED_STEPS = 3400
    STARTUP_SPEED_DEG_S = 30.0     # slow, safe speed right after connecting

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.servo_map = ServoMap()
        self.port_handler = None
        self.packet_handler = None
        self._comm_ok = None
        self._takes_port = True        # SDK flavour, see _open_sdk()
        self.is_connected = False
        self.simulated_angles = {sid: 0.0 for sid in self.SERVO_IDS}
        self._clamp_warned = set()

    # ------------------------------------------------------- low-level access
    def _open_sdk(self) -> None:
        """Create the port/packet handlers.  Works with both flavours of the Feetech python SDK."""
        import scservo_sdk as sdk

        self._comm_ok = sdk.COMM_SUCCESS
        self.port_handler = sdk.PortHandler(self.port)
        if hasattr(sdk, "PacketHandler"):          # Dynamixel-style:  handler.method(port, id, ...)
            self.packet_handler = sdk.PacketHandler(0)
            self._takes_port = True
        else:                                      # FTServo-style:    sms_sts(port).method(id, ...)
            self.packet_handler = sdk.sms_sts(self.port_handler)
            self._takes_port = False

    def _call(self, method: str, *args):
        fn = getattr(self.packet_handler, method)
        return fn(self.port_handler, *args) if self._takes_port else fn(*args)

    def _write2(self, servo_id: int, address: int, value: int) -> bool:
        result, _ = self._call("write2ByteTxRx", servo_id, address, value)
        return result == self._comm_ok

    def _read2(self, servo_id: int, address: int):
        value, result, _ = self._call("read2ByteTxRx", servo_id, address)
        return value if result == self._comm_ok else None

    # ------------------------------------------------------------- connection
    def connect(self) -> bool:
        try:
            self._open_sdk()
            if not (self.port_handler.openPort() and self.port_handler.setBaudRate(self.baudrate)):
                print(f"[STS3215Driver] Cannot open {self.port} (device missing, no permission or bad baudrate).")
            else:
                _, result, _ = self._call("ping", 1)
                if result == self._comm_ok:
                    self.is_connected = True
                    print(f"[STS3215Driver] PHYSICAL HARDWARE DETECTED on {self.port} @ {self.baudrate} bps.")
                    for sid in self.SERVO_IDS:          # slow start BEFORE any torque / motion
                        self.set_servo_speed(sid, self.STARTUP_SPEED_DEG_S)
                    print(f"[STS3215Driver] Startup speed limited to {self.STARTUP_SPEED_DEG_S:.0f} deg/s.")
                    return True
                print("[STS3215Driver] Port opened but servo ID 1 did not answer (power, baudrate or ID?).")
                self.port_handler.closePort()
        except Exception as e:
            print(f"[STS3215Driver] Connect Error: {e}")

        self.is_connected = False
        print(f"[STS3215Driver] No physical servos on {self.port} - SIMULATION (MOCK) MODE.")
        return False

    def disconnect(self) -> None:
        if self.is_connected:
            self.port_handler.closePort()
            self.is_connected = False
            print("[STS3215Driver] Serial connection closed.")

    # ---------------------------------------------------------------- motion
    def set_servo_angle(self, servo_id: int, angle_deg: float) -> bool:
        """Goal position in degrees (0 = 1024 steps), clamped to +/-60 deg."""
        clamped = ServoMap.clamp_angle(angle_deg)
        if clamped != float(angle_deg) and servo_id not in self._clamp_warned:
            self._clamp_warned.add(servo_id)
            print(f"[STS3215Driver] WARNING: servo {servo_id} target {angle_deg:.1f} deg "
                  f"clamped to {clamped:.1f} deg.")
        self.simulated_angles[servo_id] = clamped            # mock state always follows
        if not self.is_connected:
            return True
        return self._write2(servo_id, self.ADDR_GOAL_POSITION, self.servo_map.to_count(servo_id, clamped))

    def set_servo_speed(self, servo_id: int, speed_deg_s: float) -> bool:
        """Speed ceiling in deg/s.  Never 0 (0 means unlimited on the STS3215)."""
        if not self.is_connected:
            return True
        steps = int(speed_deg_s / 360.0 * self.COUNTS_PER_REV)
        return self._write2(servo_id, self.ADDR_GOAL_SPEED, max(1, min(self.MAX_SPEED_STEPS, steps)))

    def get_servo_angle(self, servo_id: int) -> float:
        if self.is_connected:
            count = self._read2(servo_id, self.ADDR_PRESENT_POSITION)
            if count is not None:
                self.simulated_angles[servo_id] = self.servo_map.to_deg(servo_id, count)
        return self.simulated_angles.get(servo_id, 0.0)

    def enable_torque(self, servo_id: int, enable: bool) -> None:
        if not self.is_connected:
            print(f"[STS3215Driver Mock] Servo ID {servo_id} torque {'ENABLED' if enable else 'DISABLED'}.")
            return
        if enable:      # goal := present position, so torque-on never jumps to an old goal
            count = self._read2(servo_id, self.ADDR_PRESENT_POSITION)
            if count is not None:
                self._write2(servo_id, self.ADDR_GOAL_POSITION, int(count))
        self._call("write1ByteTxRx", servo_id, self.ADDR_TORQUE_ENABLE, 1 if enable else 0)
