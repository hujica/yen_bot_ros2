#!/usr/bin/env python3
"""READ-ONLY helper: print the raw step count of servos 1-4 and the angle the software derives from it.

It never writes to a servo (no torque, no speed, no goal), so it cannot move the arm.

Use it to find the real "zero":
  1. Put the arm in the physical L-shape home pose (upper arm horizontal, forearm hanging down).
     Torque must be OFF to move it by hand - press "TORQUE OFF" in the GUI first and SUPPORT the arm,
     it will drop under gravity.  Close the GUI afterwards (only one program may use the serial port).
  2. ros2 run yen_bot tools_node [--ros-args -p port:=/dev/ttyACM0]
  3. Copy the counts into ServoMap.CENTER_COUNT (hardware/servo_map.py).
"""
from yen_bot.hardware.servo_map import ServoMap
from yen_bot.hardware.sts3215_driver import STS3215Driver


def read_counts(port="/dev/ttyACM0", baudrate=1000000, emit=print):
    """Print (via emit) the raw counts of servos 1-4; returns {servo_id: count}."""
    drv = STS3215Driver(port, baudrate)
    drv._open_sdk()                                           # read-only access from here on
    if not (drv.port_handler.openPort() and drv.port_handler.setBaudRate(drv.baudrate)):
        raise RuntimeError(f"Cannot open {port}")

    smap = ServoMap()
    emit(f"{'ID':>3} {'count':>7} {'angle now (deg)':>16} {'count at 0 deg now':>19} {'offset from it':>15}")
    measured = {}
    for sid in drv.SERVO_IDS:
        count = drv._read2(sid, drv.ADDR_PRESENT_POSITION)
        if count is None:
            emit(f"{sid:>3}  no answer")
            continue
        measured[sid] = int(count)
        emit(f"{sid:>3} {count:>7} {smap.to_deg(sid, count):>16.1f} {smap.CENTER_COUNT:>19} "
             f"{count - smap.CENTER_COUNT:>+15}")
    drv.port_handler.closePort()

    if len(measured) == 4:
        emit("If the arm is held in the L-shape right now, use:")
        emit(f"    CENTER_COUNT per servo = {measured}")
    return measured
