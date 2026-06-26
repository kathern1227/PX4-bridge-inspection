#!/usr/bin/env python3
"""
PX4 SITL 简化测试: 起飞 → 悬停 → RC开关测试 → 降落
全程使用 Offboard 模式 + 位置控制
RC 参数在起飞前设置好，避免飞行中改参引起瞬态
"""

import time, threading, math, struct
from pymavlink import mavutil

# PX4 custom mode: OFFBOARD = main_mode 6
OFFBOARD_MODE = 6
OFFBOARD_HB = OFFBOARD_MODE << 16  # 393216

cx, cy, cz = 0.0, 0.0, 0.0
sx, sy, sz = 0.0, 0.0, 0.0
running = True
lock = threading.Lock()
port = None


def stream_setpoints():
    """持续发送位置目标点（Offboard模式要求>2Hz）"""
    global port, sx, sy, sz, running
    while running:
        with lock:
            x, y, z = sx, sy, sz
        port.mav.set_position_target_local_ned_send(
            0, port.target_system, port.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # 只用位置控制
            x, y, z,
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0)      # yaw, yaw_rate
        time.sleep(0.05)  # 20Hz


def read_pos():
    """读取当前位置"""
    global cx, cy, cz
    m = port.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
    if m:
        cx, cy, cz = m.x, m.y, m.z


def log_pos(label=''):
    """读取并打印当前位置"""
    read_pos()
    print(f'    [{label}] pos=({cx:.2f}, {cy:.2f}, {-cz:.2f})', flush=True)


def wait_ack(cmd_id, timeout=5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = port.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if m and m.command == cmd_id:
            return m.result
    return -1


def set_mode(mode_main, mode_sub=0):
    """切换PX4自定义飞行模式"""
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_main, mode_sub, 0, 0, 0, 0)
    time.sleep(0.5)
    return wait_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=5)


def arm():
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    return wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5)


def disarm():
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    return wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5)


def move_to(tx, ty, tz, speed=2.0, name='wp'):
    """平滑移动到目标位置"""
    global cx, cy, cz, sx, sy, sz
    time.sleep(0.2)
    for _ in range(5):
        read_pos()
        time.sleep(0.05)

    start_x, start_y, start_z = cx, cy, cz
    dist = math.sqrt((tx - start_x)**2 + (ty - start_y)**2 + (tz - start_z)**2)
    dur = max(dist / speed, 1.0)
    steps = max(int(dur / 0.05), 20)
    print(f'  -> {name}: ({tx:.1f},{ty:.1f},{-tz:.1f}) dist={dist:.1f}m', flush=True)

    for i in range(steps + 1):
        t = i / steps
        with lock:
            sx = start_x + (tx - start_x) * t
            sy = start_y + (ty - start_y) * t
            sz = start_z + (tz - start_z) * t
        read_pos()
        time.sleep(0.05)

    # 到达后保持位置3秒，确保稳定
    with lock:
        sx, sy, sz = tx, ty, tz
    t0 = time.time()
    while time.time() - t0 < 3:
        read_pos()
        time.sleep(0.3)

    print(f'    pos=({cx:.2f},{cy:.2f},{-cz:.2f}m)  OK', flush=True)


def drain_messages(duration=0.5):
    """排空 MAVLink 接收缓冲区中的旧消息"""
    t0 = time.time()
    while time.time() - t0 < duration:
        m = port.recv_match(blocking=True, timeout=0.1)
        if not m:
            break


def set_param(param_name, value, param_type=None):
    """通过 MAVLink 设置 PX4 参数（自动处理 INT32 字节编码）"""
    # PX4 通过 MAVLink 的 param_set 用 float 字段传输 INT32，
    # 存储的是 float 位模式（IEEE 754）而非转换后的整数值。
    # 因此需要把 int 值按位重新解释为 float 再发送。
    if param_type is None:
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        fval = struct.unpack('f', struct.pack('i', int(value)))[0]
    else:
        fval = float(value)
    drain_messages(0.3)
    port.mav.param_set_send(
        port.target_system, port.target_component,
        param_name.encode('utf-8'), fval, param_type)
    t0 = time.time()
    while time.time() - t0 < 5:
        m = port.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not m:
            continue
        pid = m.param_id.decode('utf-8') if isinstance(m.param_id, bytes) else m.param_id
        if pid.rstrip('\x00') == param_name:
            return m.param_value
    return None


def get_param(param_name):
    """通过 MAVLink 读取 PX4 参数（自动解码 INT32 位模式）"""
    drain_messages(0.3)
    port.mav.param_request_read_send(
        port.target_system, port.target_component,
        param_name.encode('utf-8'), -1)
    t0 = time.time()
    while time.time() - t0 < 5:
        m = port.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if not m:
            continue
        pid = m.param_id.decode('utf-8') if isinstance(m.param_id, bytes) else m.param_id
        if pid.rstrip('\x00') == param_name:
            return _decode_param(m.param_value, m.param_type)
    return None


def _decode_param(pval, ptype):
    """解码 MAVLink param_value：INT32 参数字节反向转义"""
    if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack('i', struct.pack('f', pval))[0]
    return pval


def rc_update_restart():
    """重启 rc_update 使其重新加载 RC 参数（RC_CHAN_CNT 等）"""
    port.mav.command_long_send(
        port.target_system, port.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 3, 0, 0, 0, 0, 0, 0)  # param2=3 -> restart rc_update
    time.sleep(2)


# ========== RC Override 线程 ==========
_rc_override_thread = None
_rc_override_running = False
_rc_ch5 = 1000   # AUX1 OFF
_rc_ch6 = 1000   # AUX2 OFF


def _rc_override_loop():
    """后台持续发送 RC_CHANNELS_OVERRIDE（10Hz）
    从一开始就运行，避免 RC 输入突然出现造成瞬态"""
    global _rc_ch5, _rc_ch6
    while _rc_override_running:
        c5, c6 = _rc_ch5, _rc_ch6
        port.mav.rc_channels_override_send(
            port.target_system, port.target_component,
            1500, 1500, 1500, 1500, c5, c6, 1500, 1500)
        time.sleep(0.1)  # 10Hz


def start_rc_override():
    global _rc_override_thread, _rc_override_running
    _rc_override_running = True
    _rc_override_thread = threading.Thread(target=_rc_override_loop, daemon=True)
    _rc_override_thread.start()


def set_rc_aux(ch5, ch6):
    """更新 RC AUX 开关值（线程安全）"""
    global _rc_ch5, _rc_ch6
    _rc_ch5 = ch5
    _rc_ch6 = ch6


def stop_rc_override_thread():
    global _rc_override_thread, _rc_override_running
    _rc_override_running = False
    if _rc_override_thread:
        _rc_override_thread.join(timeout=1)
        _rc_override_thread = None


# ========== 开关测试 ==========
def test_rc_aux_switches():
    """
    测试 RC AUX 开关: AUX1(ceiling_arm), AUX2(detach)
    RC 参数已在起飞前设好，此函数只切换开关值
    每步前后记录位置，定位偏移来源
    """
    AUX_OFF = 1000
    AUX_ON  = 2000

    print('\n' + '='*60, flush=True)
    print('  RC AUX SWITCH TEST (Ceiling Controller)', flush=True)
    print('='*60, flush=True)

    # 记录初始位置
    log_pos('before test')

    # ---- 测试1: AUX1 ON ----
    print('\n[T-1] AUX1=ON, AUX2=OFF  (expect: CEILING_ARM state=1)', flush=True)
    log_pos('before AUX1=ON')
    set_rc_aux(AUX_ON, AUX_OFF)
    time.sleep(3)
    log_pos('after AUX1=ON (3s)')
    print('    Check PX4 console: "Ceiling state: 0 -> 1"', flush=True)

    # ---- 测试2: AUX1 OFF ----
    print('\n[T-2] AUX1=OFF, AUX2=OFF  (expect: back to NORMAL_FLIGHT state=0)', flush=True)
    log_pos('before AUX1=OFF')
    set_rc_aux(AUX_OFF, AUX_OFF)
    time.sleep(3)
    log_pos('after AUX1=OFF (3s)')
    print('    Check PX4 console: "Ceiling state: 1 -> 0"', flush=True)

    # ---- 测试3: AUX2 ON ----
    print('\n[T-3] AUX1=OFF, AUX2=ON  (expect: detach switch active)', flush=True)
    log_pos('before AUX2=ON')
    set_rc_aux(AUX_OFF, AUX_ON)
    time.sleep(2)
    log_pos('after AUX2=ON (2s)')
    print('    Check PX4 console: AUX2=1, detach_switch_on=true', flush=True)

    # ---- 测试4: AUX1 ON + AUX2 ON ----
    print('\n[T-4] AUX1=ON, AUX2=ON  (expect: both active)', flush=True)
    log_pos('before both ON')
    set_rc_aux(AUX_ON, AUX_ON)
    time.sleep(2)
    log_pos('after both ON (2s)')
    print('    Check PX4 console: ARM=1 DET=1', flush=True)

    # ---- 恢复 ----
    print('\n[T-5] Restoring AUX1=OFF, AUX2=OFF ...', flush=True)
    set_rc_aux(AUX_OFF, AUX_OFF)
    time.sleep(1)
    log_pos('restored')

    print('\n' + '='*60, flush=True)
    print('  RC AUX SWITCH TEST COMPLETE', flush=True)
    print('  Verify: PX4 nsh terminal / QGC STATUSTEXT [CeilCtrl]', flush=True)
    print('='*60, flush=True)


def main():
    global port, running, sx, sy, sz

    print('\n=== PX4 Simplified Test: Takeoff + AUX Switch ===\n', flush=True)

    # =================== 0. 连接 ===================
    print('[0] Connecting udp:127.0.0.1:14540 ...', flush=True)
    port = mavutil.mavlink_connection('udp:127.0.0.1:14540')

    t0 = time.time()
    while time.time() - t0 < 20:
        hb = port.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            print(f'    Connected! sys={port.target_system} mode={hb.custom_mode}', flush=True)
            break
    else:
        print('    FAIL: No heartbeat after 20s', flush=True)
        return

    # =================== 1. 开始发送当前位置目标点 ===================
    print('\n[1] Start streaming setpoints ...', flush=True)
    read_pos()
    with lock:
        sx, sy, sz = cx, cy, cz
    stream_thread = threading.Thread(target=stream_setpoints, daemon=True)
    stream_thread.start()
    time.sleep(1.0)

    # =================== 1.5 启动 RC override 线程 ===================
    print('\n[1.5] Starting RC override thread (sticks center, AUX off) ...', flush=True)
    start_rc_override()
    time.sleep(2)

    # =================== 1.6 配置 RC 参数并重启 rc_update ===================
    # PX4 的 rc_update 要求 RC_CHAN_CNT>0 才发布 manual_control_setpoint。
    # 必须通过 MAVLink 正确设置 INT32 参数，然后重启 rc_update 生效。
    print('\n[1.6] Configuring RC params & restarting rc_update ...', flush=True)
    rc_params = {
        'RC_CHAN_CNT': 8,
        'RC_MAP_ROLL': 1,
        'RC_MAP_THROTTLE': 4,
        'RC_MAP_AUX1': 5,   # CH5 -> AUX1 (ceiling_arm)
        'RC_MAP_AUX2': 6,   # CH6 -> AUX2 (detach)
    }
    for name, val in rc_params.items():
        set_param(name, val)
        print(f'    set {name} = {val}', flush=True)
        time.sleep(0.3)

    # 重启 rc_update 使参数生效
    print('    restarting rc_update ...', flush=True)
    rc_update_restart()
    time.sleep(2)

    # 验证
    print('    verifying ...', flush=True)
    ok = True
    for name, val in rc_params.items():
        v = get_param(name)
        ok1 = (v is not None and int(v) == val)
        if not ok1:
            ok = False
        print(f'    {name} = {v} (expect {val})  {"OK" if ok1 else "MISMATCH"}', flush=True)
        time.sleep(0.2)

    if not ok:
        print('\n    WARNING: Some RC params did not verify. Trying to continue anyway.', flush=True)

    # =================== 2. 切换Offboard模式 ===================
    print('\n[2] Switch to OFFBOARD ...', flush=True)
    for attempt in range(5):
        r = set_mode(OFFBOARD_MODE)
        hb = port.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == OFFBOARD_HB:
            print(f'    OFFBOARD OK! (attempt {attempt+1})', flush=True)
            break
        print(f'    attempt {attempt+1}: mode={hb.custom_mode if hb else -1} (want {OFFBOARD_HB}), retrying...', flush=True)
        time.sleep(1)
    else:
        print('    FAIL: Cannot enter Offboard', flush=True)
        running = False
        return

    # =================== 3. 解锁 ===================
    print('\n[3] Arming ...', flush=True)
    for attempt in range(3):
        r = arm()
        print(f'    Arm ACK: {r} (0=OK)', flush=True)
        if r == 0:
            break
        time.sleep(1)

    hb = port.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f'    Armed: {armed}', flush=True)
    if not armed:
        print('    FAIL: Cannot arm, aborting', flush=True)
        running = False
        return

    # =================== 4. 起飞到5m ===================
    print('\n[4] Takeoff to 5m ...', flush=True)
    tgt_z = -5.0  # NED坐标系
    move_to(0, 0, tgt_z, speed=1.5, name='Takeoff5m')

    # =================== 5. 稳定悬停5秒，观察有无漂移 ===================
    print('\n[5] Hover stabilization (5s) ...', flush=True)
    with lock:
        sx, sy, sz = 0, 0, tgt_z
    for i in range(5):
        log_pos(f'hover {i+1}s')
        time.sleep(1)

    # =================== 6. RC辅助开关测试 ===================
    print('\n[6] === RC AUX Switch Test ===', flush=True)
    with lock:
        sx, sy, sz = cx, cy, cz  # 锁定当前位置
    test_rc_aux_switches()

    # =================== 7. 测试后悬停3秒，观察是否恢复 ===================
    print('\n[7] Post-test hover (3s) ...', flush=True)
    with lock:
        sx, sy, sz = 0, 0, tgt_z
    for i in range(3):
        log_pos(f'post-test {i+1}s')
        time.sleep(1)

    # =================== 8. 降落 ===================
    print('\n[8] === Landing ===', flush=True)
    move_to(0, 0, 0, speed=1.0, name='Land')

    # =================== 9. 上锁 ===================
    time.sleep(2)
    print('\n[9] Disarming ...', flush=True)
    disarm()
    stop_rc_override_thread()
    running = False
    time.sleep(1)
    port.close()
    print('\n=== Mission Complete ===', flush=True)


if __name__ == '__main__':
    main()
