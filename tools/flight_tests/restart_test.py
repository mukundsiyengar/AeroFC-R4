#!/usr/bin/env python3
"""restart_test.py - the things an operator does at a laptop: restart the bridge, restart Gazebo, pull the USB, wrong order."""
import json, math, os, signal, subprocess, sys, threading, time
sys.path.append('/usr/lib/python3/dist-packages')
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry

ROOT = os.path.expanduser('~/hil_clean/uav-hil-simulator-main')
HIL = ROOT + '/arduino_hil'
OUT = sys.argv[1]
PS = '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'
st = {'z': 0.0, 'tilt': 0.0, 't': 0.0}
res = {}
lock = threading.Lock()


def on_odom(m):
    q, p = m.pose.orientation, m.pose.position
    with lock:
        st.update(z=p.z, t=time.time(), tilt=math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y))))))


node = Node()
node.subscribe(Odometry, '/model/X3/odometry', on_odom)


def z():
    with lock:
        return st['z']


def wait_z(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond(z()):
            return round(time.time() - t0, 1)
        time.sleep(0.05)
    return -1


def gazebo_start():
    subprocess.run(['./start_gazebo.sh', '-b'], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=40)


def gazebo_stop():
    subprocess.run(['pkill', '-f', 'gz sim'])
    time.sleep(3)


def bridge_start(name):
    log = open('/tmp/rt_%s.log' % name, 'w')
    return subprocess.Popen(['python3', '-u', 'hil_bridge_v1.py', '--port', '/dev/ttyACM0'], cwd=HIL, stdout=log, stderr=subprocess.STDOUT)


def bridge_stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            p.kill()


def bridge_said(name, n=4):
    try:
        lines = [l.strip() for l in open('/tmp/rt_%s.log' % name) if l.strip() and not l.startswith(('=', ' UAV', 'Serial', 'Failsafe T', 'Control', 'Log', 'Gazebo T', 'Dispatched', 'Using'))]
        return lines[-n:]
    except OSError:
        return []


def save():
    json.dump(res, open(OUT, 'w'), indent=1)


def wait_device(timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists('/dev/ttyACM0'):
            return round(time.time() - t0, 1)
        time.sleep(0.2)
    return -1


try:
    # 1. Normal start, their order: Gazebo, then bridge straight after the board appeared
    gazebo_start()
    b = bridge_start('c1')
    res['1_normal_start_s'] = wait_z(lambda v: v > 1.2, 45)
    time.sleep(4)
    res['1_height'] = round(z(), 2)
    save()

    # 2. Ctrl+C on the bridge in flight, pause, start it again (R4 stays plugged in, Gazebo keeps running)
    bridge_stop(b)
    time.sleep(8)
    res['2_height_after_stop'] = round(z(), 2)
    b = bridge_start('c2')
    res['2_flies_again_s'] = wait_z(lambda v: v > 1.2, 45)
    time.sleep(4)
    res['2_height'] = round(z(), 2)
    res['2_bridge'] = bridge_said('c2', 3)
    save()

    # 3. Quick restart: 1 s between stop and start
    bridge_stop(b)
    time.sleep(1)
    b = bridge_start('c3')
    time.sleep(2)
    res['3_height_2s_after'] = round(z(), 2)
    res['3_flies_again_s'] = wait_z(lambda v: v > 1.2, 45)
    time.sleep(4)
    res['3_height'] = round(z(), 2)
    res['3_bridge'] = bridge_said('c3', 3)
    save()

    # 4. "Start again" on the sheet: stop bridge and Gazebo, start both, R4 stays plugged in
    bridge_stop(b)
    gazebo_stop()
    gazebo_start()
    time.sleep(1)
    b = bridge_start('c4')
    res['4_flies_again_s'] = wait_z(lambda v: v > 1.2, 45)
    time.sleep(4)
    res['4_height'] = round(z(), 2)
    res['4_bridge'] = bridge_said('c4', 3)
    save()

    # 5. USB pulled in flight: their failsafe, then recovery (usbipd puts it back after a few seconds, like plugging in again)
    subprocess.run([PS, '-NoProfile', '-Command', 'usbipd detach --hardware-id 2341:0069'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
    time.sleep(3)
    res['5_height_3s_after_pull'] = round(z(), 2)
    res['5_bridge_alive'] = b.poll() is None
    res['5_bridge_after_pull'] = bridge_said('c4', 3)
    res['5_device_back_s'] = wait_device(40)
    time.sleep(4)
    res['5_height_after_back'] = round(z(), 2)
    res['5_recovers_by_itself_s'] = wait_z(lambda v: v > 1.2, 15)
    save()
    if res['5_recovers_by_itself_s'] < 0:
        bridge_stop(b)
        time.sleep(1)
        b = bridge_start('c5')
        res['5_after_bridge_restart_s'] = wait_z(lambda v: v > 1.2, 45)
    time.sleep(3)
    res['5_height'] = round(z(), 2)
    res['5_bridge'] = bridge_said('c5' if '5_after_bridge_restart_s' in res else 'c4', 3)
    save()

    # 6. Wrong order: bridge first, Gazebo 25 s later
    bridge_stop(b)
    gazebo_stop()
    b = bridge_start('c6')
    time.sleep(25)
    res['6_bridge_alive_without_gazebo'] = b.poll() is None
    gazebo_start()
    res['6_flies_after_gazebo_s'] = wait_z(lambda v: v > 1.2, 40)
    time.sleep(3)
    res['6_height'] = round(z(), 2)
    res['6_bridge'] = bridge_said('c6', 3)
    save()
    bridge_stop(b)
finally:
    save()
    subprocess.run(['pkill', '-f', 'hil_bridge_v1'])
    subprocess.run(['pkill', '-f', 'gz sim'])
print('restart test done')
