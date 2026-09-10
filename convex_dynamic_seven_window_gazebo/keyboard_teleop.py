#!/usr/bin/env python3
"""Small terminal keyboard controller for the local PX4 SITL instance."""

from __future__ import annotations

import curses
import time

from pymavlink import mavutil


PX4_SYSTEM_ID = 1
PX4_UDP_PORT = 18570
AXIS_STEP = 260
THROTTLE_STEP = 60
SEND_PERIOD_S = 0.05
AXIS_HOLD_S = 0.16


def command_mode(link, main_mode: int) -> None:
    custom_mode = main_mode << 16
    link.mav.set_mode_send(
        PX4_SYSTEM_ID,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )


def command_arm(link, armed: bool) -> None:
    link.mav.command_long_send(
        PX4_SYSTEM_ID,
        1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if armed else 0.0,
        0, 0, 0, 0, 0, 0,
    )


def run(screen) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    screen.timeout(25)
    link = mavutil.mavlink_connection(
        f"udpout:127.0.0.1:{PX4_UDP_PORT}",
        source_system=254,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
    )
    throttle = 0
    roll = pitch = yaw = 0
    axis_deadline = 0.0
    status = "disarmed; throttle low"
    last_heartbeat = 0.0
    last_send = 0.0

    while True:
        now = time.monotonic()
        key = screen.getch()
        if key in (ord("x"), ord("X"), 27):
            break
        if key in (ord("w"), ord("W")):
            pitch, axis_deadline = AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("s"), ord("S")):
            pitch, axis_deadline = -AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("a"), ord("A")):
            roll, axis_deadline = -AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("d"), ord("D")):
            roll, axis_deadline = AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("q"), ord("Q")):
            yaw, axis_deadline = -AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("e"), ord("E")):
            yaw, axis_deadline = AXIS_STEP, now + AXIS_HOLD_S
        elif key in (ord("r"), ord("R")):
            throttle = min(1000, throttle + THROTTLE_STEP)
        elif key in (ord("f"), ord("F")):
            throttle = max(0, throttle - THROTTLE_STEP)
        elif key in (ord("c"), ord("C"), ord(" ")):
            roll = pitch = yaw = 0
        elif key in (ord("b"), ord("B")):
            command_arm(link, True)
            status = "arm requested"
        elif key in (ord("n"), ord("N")):
            command_arm(link, False)
            throttle = 0
            status = "disarm requested"
        elif key in (ord("p"), ord("P")):
            command_mode(link, 3)  # PX4_CUSTOM_MAIN_MODE_POSCTL
            status = "Position mode requested"
        elif key in (ord("m"), ord("M")):
            command_mode(link, 1)  # PX4_CUSTOM_MAIN_MODE_MANUAL
            status = "Manual mode requested"

        if now >= axis_deadline:
            roll = pitch = yaw = 0
        if now - last_heartbeat >= 1.0:
            link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            last_heartbeat = now
        if now - last_send >= SEND_PERIOD_S:
            link.mav.manual_control_send(
                PX4_SYSTEM_ID, pitch, roll, throttle, yaw, 0,
            )
            last_send = now

        screen.erase()
        screen.addstr(0, 0, "PX4 x500 keyboard control (terminal must have focus)", curses.A_BOLD)
        screen.addstr(2, 0, "W/S pitch   A/D roll   Q/E yaw   R/F throttle")
        screen.addstr(3, 0, "P Position  M Manual   B arm     N disarm")
        screen.addstr(4, 0, "Space center axes      X/Esc quit")
        screen.addstr(6, 0, f"pitch={pitch:+4d} roll={roll:+4d} yaw={yaw:+4d} throttle={throttle:4d}")
        screen.addstr(7, 0, f"status: {status}")
        screen.refresh()

    command_arm(link, False)
    for _ in range(3):
        link.mav.manual_control_send(PX4_SYSTEM_ID, 0, 0, 0, 0, 0)
        time.sleep(0.05)


if __name__ == "__main__":
    curses.wrapper(run)
