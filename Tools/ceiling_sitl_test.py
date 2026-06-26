#!/usr/bin/env python3
"""
Ceiling Contact Controller SITL Test Script

Test sequence:
  1. Takeoff to 1m (Offboard)
  2. Inject upward distance sensor (simulate ceiling above)
  3. AUX1=ON -> trigger CEILING_ARM -> APPROACH -> ATTACH_CONTROL
  4. Hold attach for a few seconds
  5. AUX1=OFF -> trigger DETACH -> RECOVERY -> NORMAL
  6. Stop sensor injection
  7. Land & disarm

Prerequisites (run in PX4 nsh before this script):
    ceiling_controller start
    param set RC_CHAN_CNT 8
    param set RC_MAP_ROLL 1
    param set RC_MAP_THROTTLE 4
    param set RC_MAP_AUX1 5
    param set RC_MAP_AUX2 6
    rc_update stop
    rc_update start
"""

import time
import threading
from collections import deque
from pymavlink import mavutil

# ============== Config ==============
AUX_OFF = 1000
AUX_ON  = 2000
TARGET_Z = -1.0          # 1m altitude (NED)
# target_dist = D0 - comp_tgt = 0.22 - 0.02 = 0.20m = 20cm
# Drone hovers at 1m, ceiling at 1.5m → initial distance 50cm
# Controller drives to target_dist=20cm → final height ~1.3m
CEILING_HEIGHT_M = 1.5
ATTACH_DIST_CM = 20      # fallback fixed distance (cm)
DETACH_DIST_CM = 200     # injected ceiling distance during detach (cm)
OFFBOARD_RATE = 50       # Hz
#
# IMPORTANT: run these in PX4 nsh before test:
#   param set CEIL_D0 0.22
#   param set CEIL_DIST_THR 0.60
#   param set CEIL_COMP_TGT 0.02

# ============== Globals ==============
running = True            # pos_thread flag (runs until script exit)
inject_running = False    # sensor_inject_thread flag (started in phase 5, stopped in phase 9)
inject_dist_cm = DETACH_DIST_CM
pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
log_buffer = deque(maxlen=500)
lock = threading.Lock()

# ============== MAVLink ==============
print("Connecting to PX4 SITL (udp:127.0.0.1:14540) ...")
port = mavutil.mavlink_connection('udp:127.0.0.1:14540')
port.wait_heartbeat()
print(f"  Connected (sysid={port.target_system}, compid={port.target_component})")

port.mav.request_data_stream_send(
    port.target_system, port.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
port.mav.request_data_stream_send(
    port.target_system, port.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)


# ============== Threads ==============
def pos_thread():
    while running:
        m = port.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if m:
            with lock:
                pos['x'], pos['y'], pos['z'] = m.x, m.y, m.z
                pos['vx'], pos['vy'], pos['vz'] = m.vx, m.vy, m.vz
        time.sleep(0.05)


def sensor_inject_thread():
    """Inject upward-facing distance sensor at 10Hz"""
    while inject_running:
        with lock:
            z = pos['z']
        # Compute distance to virtual ceiling; NED z is negative upward
        actual_height = -z
        dist = int(max(5.0, (CEILING_HEIGHT_M - actual_height) * 100.0))
        # Allow fixed override when explicitly set to non-zero
        if inject_dist_cm > 0:
            dist = inject_dist_cm
        port.mav.distance_sensor_send(
            time_boot_ms=0,
            min_distance=5,
            max_distance=500,
            current_distance=dist,
            type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            id=0,
            orientation=24,
            covariance=255)
        time.sleep(0.1)


threading.Thread(target=pos_thread, daemon=True).start()


# ============== Helpers ==============
def send_offboard_sp(x, y, z):
    port.mav.set_position_target_local_ned_send(
        0, port.target_system, port.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)


def send_offboard_vel(vx, vy, vz):
    """Send velocity-only setpoint (position/accel/yaw ignored)"""
    # Ignore: position(1+2+4), acceleration(64+128+256), yaw(1024), yaw_rate(2048)
    # Valid: velocity only
    type_mask = (1 | 2 | 4 | 64 | 128 | 256 | 1024 | 2048)
    port.mav.set_position_target_local_ned_send(
        0, port.target_system, port.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def set_rc(ch5, ch6):
    port.mav.rc_channels_override_send(
        port.target_system, port.target_component,
        65535, 65535, 65535, 65535, ch5, ch6,
        65535, 65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535)


def get_pos():
    with lock:
        return dict(pos)


def log_state(label):
    p = get_pos()
    line = (f"[{label}] t={time.time()-t_start:.1f}s  "
            f"pos=({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f})  "
            f"vel=({p['vx']:.2f}, {p['vy']:.2f}, {p['vz']:.2f})")
    print(f"    {line}", flush=True)
    log_buffer.append(line)


def wait_stable(target_z, tol=0.3, timeout=15, label="stable"):
    t0 = time.time()
    while time.time() - t0 < timeout:
        send_offboard_sp(0, 0, target_z)
        p = get_pos()
        if abs(p['z'] - target_z) < tol:
            print(f"    [{label}] reached in {time.time()-t0:.1f}s", flush=True)
            return True
        time.sleep(0.05)
    print(f"    [{label}] TIMEOUT after {timeout}s", flush=True)
    return False


def stream_sp(x, y, z, duration_sec):
    n = int(duration_sec * OFFBOARD_RATE)
    for _ in range(n):
        send_offboard_sp(x, y, z)
        time.sleep(1.0 / OFFBOARD_RATE)


def set_mode(mode_num):
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num, 0, 0, 0, 0, 0)


def wait_for_preflight(timeout=30):
    print("    waiting for EKF/preflight ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = port.recv_match(type='SYS_STATUS', blocking=False)
        if s:
            healthy = (s.onboard_control_sensors_health & 0b11111111) == 0b11111111
            if healthy:
                return True
        time.sleep(0.5)
    return False


def phase_delay(seconds, label="delay", x=0, y=0, z=None):
    """Print position every second during delay, keep streaming offboard setpoint at 20Hz"""
    if z is None:
        z = TARGET_Z
    for i in range(seconds):
        for _ in range(20):  # 20Hz setpoint stream
            send_offboard_sp(x, y, z)
            time.sleep(0.05)
        log_state(f"{label} {i+1}s/{seconds}s")


def phase_delay_no_sp(seconds, label="delay"):
    """Print position every second without sending setpoints (caller handles streaming)"""
    for i in range(seconds):
        time.sleep(1.0)
        log_state(f"{label} {i+1}s/{seconds}s")


def save_log(path="/tmp/ceiling_sitl_log.txt"):
    with open(path, 'w') as f:
        f.write("=== Ceiling SITL Flight Log ===\n\n")
        for line in log_buffer:
            f.write(line + "\n")
    print(f"\nLog saved to {path}", flush=True)


# ============== Main ==============
t_start = time.time()


def main():
    global running, inject_running, inject_dist_cm

    print("\n" + "=" * 60)
    print("  Ceiling Contact Controller SITL Test")
    print("=" * 60)

    # --- Phase 0: Wait EKF ---
    print("\n[PHASE 0] Waiting for EKF / preflight ...", flush=True)
    if not wait_for_preflight(timeout=30):
        print("    WARNING: preflight timeout, proceeding anyway", flush=True)
    phase_delay(3, "preflight")

    # --- Phase 1: Stream offboard setpoint ---
    print("\n[PHASE 1] Warming up offboard stream (2s) ...", flush=True)
    stream_sp(0, 0, TARGET_Z, 2.0)
    log_state("after stream")

    # --- Phase 2: Switch to Offboard ---
    print("\n[PHASE 2] Switching to Offboard mode ...", flush=True)
    set_mode(6)
    phase_delay(3, "offboard")

    # --- Phase 3: Arm ---
    print("\n[PHASE 3] Arming ...", flush=True)
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    phase_delay(3, "armed")

    # --- Phase 4: Takeoff ---
    print(f"\n[PHASE 4] Takeoff to {abs(TARGET_Z):.0f}m ...", flush=True)
    stream_sp(0, 0, TARGET_Z, 6.0)
    if wait_stable(TARGET_Z, tol=0.3, timeout=10, label="takeoff"):
        log_state("takeoff OK")
    else:
        log_state("takeoff FAIL")
    phase_delay(3, "hover")

    # --- Phase 5: Start distance sensor injection ---
    print("\n[PHASE 5] Starting distance sensor injection (far) ...", flush=True)
    print(f"    inject_dist = {DETACH_DIST_CM}cm (no ceiling)", flush=True)
    inject_dist_cm = DETACH_DIST_CM
    inject_running = True
    sensor_t = threading.Thread(target=sensor_inject_thread, daemon=True)
    sensor_t.start()
    phase_delay(3, "sensor_far")

    # --- Phase 6: AUX1=ON + dynamic close distance (simulate ceiling) ---
    print("\n[PHASE 6] AUX1=ON + dynamic close distance ...", flush=True)
    print("    inject_dist = dynamic (based on actual altitude)", flush=True)
    print("    Expected state transition: NORMAL(0) -> ARM(1) -> APPROACH(2)", flush=True)
    set_rc(AUX_ON, AUX_OFF)
    inject_dist_cm = 0   # enable dynamic injection immediately
    phase_delay(3, "inject_close")

    print("    Expected: ARM(1) -> APPROACH(2) -> ATTACH(3)", flush=True)
    # Stream position setpoint continuously so offboard doesn't timeout.
    # mc_pos_control's APPROACH_MODE will override vz_sp=-0.08 upward.
    # ATTACH mode will override thrust_body_z directly.
    # We keep sending so offboard stays active and horizontal position is held.
    t0 = time.time()
    attached = False
    while time.time() - t0 < 20.0:
        send_offboard_sp(0, 0, TARGET_Z)
        # Poll ceiling_contact_status to detect attach
        m = port.recv_match(type='CEILING_CONTACT_STATUS', blocking=False)
        if m:
            if m.state >= 3:  # ATTACH_CONTROL_MODE=3 or higher
                if not attached:
                    attached = True
                    print(f"    [ATTACH DETECTED] state={m.state} dist={m.ceiling_distance:.2f}m "
                          f"thrust_z={m.thrust_body_z_sp:.3f}", flush=True)
                break
        time.sleep(0.05)
    if attached:
        log_state("attach_confirmed")
    else:
        log_state("attach_timeout")

    # --- Phase 7: Hold attach ---
    print("\n[PHASE 7] Holding attach (6s) ...", flush=True)
    for i in range(6):
        for _ in range(20):  # 20Hz setpoint to keep offboard alive
            send_offboard_sp(0, 0, TARGET_Z)
            time.sleep(0.05)
        log_state(f"hold_attach {i+1}s/6s")
    log_state("after hold")

    # --- Phase 8: AUX1=OFF + detach ---
    print("\n[PHASE 8] AUX1=OFF + injecting far distance ...", flush=True)
    print("    Expected: DETACH(5) -> RECOVERY(6) -> NORMAL(0)", flush=True)
    set_rc(AUX_OFF, AUX_OFF)
    inject_dist_cm = DETACH_DIST_CM
    phase_delay(3, "AUX1=OFF")

    # Keep sending position setpoint during detach/recovery so offboard stays active
    t1 = time.time()
    detached = False
    while time.time() - t1 < 15.0:
        send_offboard_sp(0, 0, TARGET_Z)
        m = port.recv_match(type='CEILING_CONTACT_STATUS', blocking=False)
        if m:
            if m.state == 0:  # NORMAL_FLIGHT
                if not detached:
                    detached = True
                    print(f"    [DETACH CONFIRMED] state={m.state}", flush=True)
                break
        time.sleep(0.05)
    if detached:
        log_state("detach_confirmed")
    else:
        log_state("detach_timeout")

    # --- Phase 9: Stop injection ---
    print("\n[PHASE 9] Stopping sensor injection ...", flush=True)
    inject_running = False
    sensor_t.join(timeout=1)
    phase_delay(3, "post_inject", z=TARGET_Z)

    # --- Phase 10: Land ---
    print("\n[PHASE 10] Landing ...", flush=True)
    # Send LAND command to trigger auto-landing
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    # Stream z=0 setpoints to descend (keep offboard alive if still in offboard)
    stream_sp(0, 0, 0, 3.0)

    # Wait for actual touchdown
    t0 = time.time()
    landed_detected = False
    while time.time() - t0 < 15.0:
        send_offboard_sp(0, 0, 0)
        m = port.recv_match(type='EXTENDED_SYS_STATE', blocking=False)
        if m and m.landed_state == 1:  # MAV_LANDED_STATE_ON_GROUND
            landed_detected = True
            break
        time.sleep(0.1)
    if landed_detected:
        log_state("landed_confirmed")
    else:
        log_state("land_timeout")

    # --- Phase 11: Disarm ---
    print("\n[PHASE 11] Disarming ...", flush=True)
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    phase_delay(3, "disarmed", z=0)

    # Stop pos thread
    running = False

    # --- Summary ---
    print("\n" + "=" * 60)
    print("  Test Complete")
    print("=" * 60)
    save_log("/tmp/ceiling_sitl_log.txt")
    print("\nCheck PX4 console for [CeilCtrl] state transitions", flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        running = False
        save_log("/tmp/ceiling_sitl_log.txt")
        print("\nInterrupted by user. Log saved.", flush=True)
