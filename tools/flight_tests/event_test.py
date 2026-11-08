#!/usr/bin/env python3
"""event_test.py - flies the drone the way the organisers would: their keyboard tool and their command tool only.
Truth comes from Gazebo itself (odometry topic), so nothing of ours sits in the loop."""
import json, math, os, pty, select, subprocess, sys, threading, time
sys.path.append('/usr/lib/python3/dist-packages')
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.actuators_pb2 import Actuators

ROOT = sys.argv[1]
OUT = sys.argv[2]
MODE = sys.argv[3] if len(sys.argv) > 3 else 'full'   # full | hover
HIL = os.path.join(ROOT, 'arduino_hil')

state = {'t': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'tilt': 0.0, 'n': 0}
log = []
motor = {'n': 0, 'last': [0, 0, 0, 0], 't': 0.0}
lock = threading.Lock()


def on_odom(m):
    q = m.pose.orientation
    p = m.pose.position
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    r22 = 1 - 2 * (q.x * q.x + q.y * q.y)
    tilt = math.degrees(math.acos(max(-1, min(1, r22))))
    with lock:
        state.update(t=time.time(), x=p.x, y=p.y, z=p.z, yaw=yaw, tilt=tilt, n=state['n'] + 1)
        log.append((time.time(), p.x, p.y, p.z, math.degrees(yaw), tilt, motor['n']))


def on_motor(m):
    with lock:
        motor['n'] += 1
        motor['last'] = list(m.velocity)
        motor['t'] = time.time()


node = Node()
node.subscribe(Odometry, '/model/X3/odometry', on_odom)
node.subscribe(Actuators, '/X3/gazebo/command/motor_speed', on_motor)


def snap():
    with lock:
        return dict(state)


def sleep(s):
    time.sleep(s)


class KeyTool:
    """Their control_input.py --interactive in a real terminal: real key presses."""
    def __init__(self):
        self.pid, self.fd = pty.fork()
        if self.pid == 0:
            os.chdir(HIL)
            os.execvp('python3', ['python3', 'control_input.py', '--interactive'])
        time.sleep(1.0)

    def press(self, keys, gap=0.12):
        for k in keys:
            os.write(self.fd, k.encode())
            time.sleep(gap)
            r, _, _ = select.select([self.fd], [], [], 0)
            if r:
                try:
                    os.read(self.fd, 4096)
                except OSError:
                    pass

    def close(self):
        try:
            os.write(self.fd, b'x')
            time.sleep(0.5)
        except OSError:
            pass


def cli(*args):
    """Their control_input.py with ordinary command line arguments (timed manoeuvre)."""
    return subprocess.run(['python3', 'control_input.py'] + list(args), cwd=HIL, capture_output=True, text=True, timeout=60)


def body_move(a, b):
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    c, s = math.cos(a['yaw']), math.sin(a['yaw'])
    return {'forward': round(dx * c + dy * s, 2), 'right': round(dx * s - dy * c, 2), 'up': round(b['z'] - a['z'], 2)}


def wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


res = {}


def save():
    with lock:
        L = list(log)
    res['log_rows'] = len(L)
    if L:
        res['worst_tilt'] = round(max(r[5] for r in L), 1)
        span = L[-1][0] - L[0][0]
        res['seconds'] = round(span, 1)
        res['motor_rate_hz'] = round((L[-1][6] - L[0][6]) / span, 1) if span > 0 else 0
    json.dump(res, open(OUT, 'w'), indent=1)


def wait_until(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond(snap()):
            return round(time.time() - t0, 1)
        time.sleep(0.05)
    return -1


def hold_stats(sec):
    t0 = time.time()
    with lock:
        i0 = len(log)
    time.sleep(sec)
    with lock:
        seg = log[i0:]
        n0 = motor['n']
    if not seg:
        return {'error': 'no odometry'}
    xs, ys, zs = [r[1] for r in seg], [r[2] for r in seg], [r[3] for r in seg]
    return {'z_mean': round(sum(zs) / len(zs), 2), 'z_range': round(max(zs) - min(zs), 3),
            'xy_range': round(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 3), 'tilt_max': round(max(r[5] for r in seg), 1),
            'motor_hz': round((seg[-1][6] - seg[0][6]) / max(0.1, seg[-1][0] - seg[0][0]), 1)}


wait_until(lambda s: s['n'] > 5, 30)
res['takeoff_s'] = wait_until(lambda s: s['z'] > 1.2 or s['tilt'] > 80, 45)
save()
if res['takeoff_s'] < 0:
    res['verdict'] = 'NO TAKE-OFF'
    save()
    sys.exit(0)
sleep(4)
res['hover'] = hold_stats(10)
save()
if MODE == 'hover':
    res['hover_long'] = hold_stats(30)
    save()
    sys.exit(0)

k = KeyTool()

if MODE == 'long':
    # 10 minutes of random flying with their keyboard tool, then it must still hold its spot
    import random
    random.seed(7)
    t_end = time.time() + 600
    presses = 0
    while time.time() < t_end:
        n = random.randint(1, 4)
        k.press(random.choice('wsadqe') * n)
        presses += n
        sleep(random.uniform(1.5, 6))
        if random.random() < 0.5:
            k.press(' ')
            sleep(random.uniform(2, 7))
    k.press(' ')
    sleep(12)
    res['presses'] = presses
    res['after_long'] = hold_stats(20)
    res['verdict'] = 'long flight done'
    save()
    k.close()
    sys.exit(0)


def try_keys(name, keys, secs=3.0):
    a = snap()
    k.press(keys)
    sleep(secs)
    b = snap()
    k.press(' ')
    sleep(5)
    c = snap()
    r = body_move(a, b)
    r['turned'] = round(wrap(math.degrees(b['yaw'] - a['yaw'])), 1)
    r['z_after'] = round(c['z'], 2)
    r['tilt_now'] = round(b['tilt'], 1)
    res['keys_' + name] = r
    save()


try_keys('w_forward', 'www')
try_keys('s_back', 'sss')
try_keys('a_left', 'aaa')
try_keys('d_right', 'ddd')
try_keys('q_turn_left', 'qqq')
try_keys('e_turn_right', 'eee')

# u / j: throttle, then SPACE back to altitude hold
a = snap(); k.press('uuuu'); sleep(2.5); b = snap(); k.press(' '); sleep(4); c = snap()
res['keys_u_up'] = {'up': round(b['z'] - a['z'], 2), 'held_at': round(c['z'], 2), 'tilt': round(b['tilt'], 1)}
save()
a = snap(); k.press('jjj'); sleep(2.0); b = snap(); k.press(' '); sleep(4); c = snap()
res['keys_j_down'] = {'up': round(b['z'] - a['z'], 2), 'held_at': round(c['z'], 2)}
save()
k.close()
sleep(1)

# their command-line examples
for name, args in (('cli_roll', ['--roll', '0.2', '--duration', '2.0']), ('cli_pitch_forward', ['--pitch', '-0.2', '--duration', '2.0']),
                   ('cli_yaw', ['--yaw', '0.3', '--duration', '2.0']), ('cli_throttle', ['--throttle', '0.70', '--duration', '2.0'])):
    a = snap(); cli(*args); b = snap(); sleep(5); c = snap()
    r = body_move(a, b)
    r['turned'] = round(wrap(math.degrees(b['yaw'] - a['yaw'])), 1)
    r['z_after'] = round(c['z'], 2)
    res[name] = r
    save()

# land and take off again with the command tool
a = snap()
cli('--throttle', '13', '--duration', '0.2')
res['land_s'] = wait_until(lambda s: s['z'] < 0.12, 40)
sleep(3)
with lock:
    res['motors_after_land'] = [round(v, 0) for v in motor['last']]
b = snap()
res['land_moved'] = round(math.hypot(b['x'] - a['x'], b['y'] - a['y']), 2)
save()
cli('--throttle', '12', '--duration', '0.2')
res['takeoff_again_s'] = wait_until(lambda s: s['z'] > 1.2, 25)
sleep(4)
res['hover_after'] = hold_stats(8)

# land with nothing but their keyboard tool, then take off again
k = KeyTool()
with lock:
    i0 = len(log)
k.press('jj')
res['j_land_s'] = wait_until(lambda s: s['z'] < 0.12, 40)
t0 = time.time()
while time.time() - t0 < 12 and max(motor['last']) > 1:
    time.sleep(0.05)
res['j_motors_stop_after_touch_s'] = round(time.time() - t0, 1)
sleep(1)
with lock:
    res['motors_after_j_land'] = [round(v, 0) for v in motor['last']]
    seg = log[i0:]
res['j_land_peak_down_ms'] = round(max((a[3] - b[3]) / max(1e-3, b[0] - a[0]) for a, b in zip(seg[::10], seg[10::10])), 2)
res['j_land_tilt'] = round(max(r[5] for r in seg), 1)
k.press(' ')
sleep(1)
k.press('u')
res['u_takeoff_s'] = wait_until(lambda s: s['z'] > 1.0, 20)
k.press(' ')
sleep(4)
res['after_u_hold'] = hold_stats(5)
k.close()
res['verdict'] = 'flown to the end'
save()
