#!/usr/bin/env python3
"""Standalone contact-friction experiment: robot variants side by side on a bare
floor (same ground plane as 1F), driven over gz topics, measured by ground truth.
No ROS launch needed.

Variants are built from the robot xacro:
  base      as-is (caster friction only via <mu1>/<mu2>, i.e. <ode><mu>)
  c010      + <bullet><friction>0.1 on the caster collision
  c000      + <bullet><friction>0.0 on the caster collision

Usage: bullet_friction_test.py <bullet|dart> <out_dir> [variants comma list] [ground: default|t0]
"""
import math, os, re, subprocess, sys, threading, time

ENGINE = sys.argv[1]
OUT = sys.argv[2]
VARIANTS = (sys.argv[3] if len(sys.argv) > 3 else 'base,c010,c000').split(',')
GROUND = sys.argv[4] if len(sys.argv) > 4 else 'default'
XACRO = '/home/jason/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro'
SPIN, LIN, WINDOW = 0.6283185, 0.25, 20.0
os.makedirs(OUT, exist_ok=True)

CASTER = {'c010': 0.1, 'c000': 0.0}


def build(variant):
    """Variant tokens: c010/c000 caster <bullet> friction only; cpNNN caster kept as its
    own link (preserveFixedJoint) with ode and bullet friction NNN/100; cyl cylinder
    wheel collisions instead of spheres."""
    urdf = subprocess.run(['xacro', XACRO], capture_output=True, text=True, check=True).stdout
    blob = ''
    if variant in CASTER:
        f = CASTER[variant]
        blob = (f'  <gazebo reference="caster_wheel_link"><collision><surface><friction><bullet>'
                f'<friction>{f}</friction><friction2>{f}</friction2><rolling_friction>0</rolling_friction>'
                f'</bullet></friction></surface></collision></gazebo>\n')
    m = re.search(r'cp(\d{3})', variant)
    if m:
        f = int(m.group(1)) / 100
        blob = ('  <gazebo reference="base_caster_wheel_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>\n'
                f'  <gazebo reference="caster_wheel_link"><collision><surface><friction>'
                f'<ode><mu>{f}</mu><mu2>{f}</mu2></ode>'
                f'<bullet><friction>{f}</friction><friction2>{f}</friction2><rolling_friction>0</rolling_friction></bullet>'
                f'</friction></surface></collision></gazebo>\n')
    if 't0' in variant:
        for link in ('left_wheel_link', 'right_wheel_link') + (() if m else ('caster_wheel_link',)):
            blob += (f'  <gazebo reference="{link}"><collision><surface><friction><torsional>'
                     f'<coefficient>0</coefficient></torsional></friction></surface></collision></gazebo>\n')
        if m:
            blob = blob.replace('<ode><mu>', '<torsional><coefficient>0</coefficient></torsional><ode><mu>')
    if 'nb' in variant:                      # narrow base collision: wheel spheres no longer overlap it
        urdf, n = re.subn(r'(<link name="base_link">.*?<collision>\s*<geometry>\s*)<box size="0.6 0.4 0.2"/>',
                          r'\g<1><box size="0.6 0.2 0.2"/>', urdf, count=1, flags=re.S)
        assert n == 1, 'base collision not found'
    if 'noarm' in variant:
        urdf = re.sub(r'\s*<link name="(arm_base_link|forearm_link|hand_link)">.*?</link>', '', urdf, flags=re.S)
        urdf = re.sub(r'\s*<joint name="(arm_base_forearm_joint|forearm_hand_joint|mobile_base_arm_joint)".*?</joint>',
                      '', urdf, flags=re.S)
        urdf = re.sub(r'\s*<gazebo>\s*<plugin[^>]*>(?:(?!</gazebo>).)*?(forearm_joint|forearm_hand_joint).*?</gazebo>',
                      '', urdf, flags=re.S)
        assert 'forearm' not in urdf, 'arm not fully removed'
    if 'cyl' in variant:
        for side in ('left', 'right'):
            urdf, n = re.subn(
                rf'(<link name="{side}_wheel_link">.*?<collision>\s*<geometry>\s*)<sphere radius="0.1"/>'
                rf'(\s*</geometry>\s*)<origin rpy="0 0 0" xyz="0 0 0"/>',
                r'\g<1><cylinder length="0.05" radius="0.1"/>\g<2><origin rpy="1.5707963267948966 0 0" xyz="0 0 0"/>',
                urdf, count=1, flags=re.S)
            assert n == 1, f'{side} wheel collision not found'
    if blob:
        i = urdf.rfind('</robot>')
        urdf = urdf[:i] + blob + '</robot>' + urdf[i + len('</robot>'):]
    up = os.path.join(OUT, f'{variant}.urdf')
    open(up, 'w').write(urdf)
    sdf = subprocess.run(['gz', 'sdf', '-p', up], capture_output=True, text=True).stdout
    sp = os.path.join(OUT, f'{variant}.sdf')
    open(sp, 'w').write(sdf)
    return sp


WORLD = '''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="ftest">
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">{engine}</plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction>{ground}<ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
      </link>
      <pose>0 0 -0.01 0 0 0</pose>
    </model>
  </world>
</sdf>
'''


class Poses(threading.Thread):
    def __init__(self, names):
        super().__init__(daemon=True)
        self.names, self.lock, self.latest = set(names), threading.Lock(), {}
        self.proc = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/ftest/dynamic_pose/info'],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def run(self):
        stamp, cur, sec, vals = 0.0, None, None, {}
        s_sec = s_nsec = 0
        in_header = False
        for line in self.proc.stdout:
            s = line.strip()
            if line.startswith('header {'):
                in_header, s_sec, s_nsec = True, 0, 0
                continue
            if in_header:
                if s.startswith('sec:'): s_sec = int(s.split()[1])
                elif s.startswith('nsec:'): s_nsec = int(s.split()[1])
                elif line.startswith('}'): in_header = False
                continue
            m = re.fullmatch(r'name: "([^"]+)"', s)
            if m:
                cur, sec, vals = (m.group(1) if m.group(1) in self.names else None), None, {}
                continue
            if cur is None:
                continue
            if s in ('position {', 'orientation {'):
                sec = s[0]
            elif line.startswith('  }'):
                sec = None
            elif line.startswith('}'):
                qx, qy, qz, qw = (vals.get(('o', k), 0.0) for k in 'xyzw')
                if not any((qx, qy, qz, qw)): qw = 1.0
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                with self.lock:
                    self.latest[cur] = (s_sec + s_nsec * 1e-9, vals.get(('p', 'x'), 0.0), vals.get(('p', 'y'), 0.0), yaw)
                cur = None
            elif sec and ':' in s:
                k, v = s.split(':', 1); vals[(sec, k)] = float(v)

    def get(self):
        with self.lock:
            return dict(self.latest)


def gz(*args, timeout=15):
    return subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout)


def command(names, lin, ang):
    for _ in range(3):
        for n in names:
            gz('topic', '-t', f'/model/{n}/cmd_vel', '-m', 'gz.msgs.Twist',
               '-p', f'linear: {{x: {lin}}}, angular: {{z: {ang}}}')


def measure(poses, names, secs):
    """Sample for `secs` wall seconds; return per-name (sim dt, unwrapped yaw change, path length)."""
    acc = {n: [None, 0.0, 0.0, None] for n in names}      # first stamp, yaw sum, path, last sample
    t0 = time.time()
    while time.time() - t0 < secs:
        cur = poses.get()
        for n in names:
            if n not in cur: continue
            st, x, y, yaw = cur[n]
            a = acc[n]
            if a[3] is None:
                a[0], a[3] = st, cur[n]
                continue
            if st <= a[3][0]: continue
            a[1] += math.remainder(yaw - a[3][3], 2 * math.pi)
            a[2] += math.hypot(x - a[3][1], y - a[3][2])
            a[3] = cur[n]
        time.sleep(0.05)
    return {n: (acc[n][3][0] - acc[n][0] if acc[n][3] else 0.0, acc[n][1], acc[n][2]) for n in names}


def main():
    engine = ('<engine><filename>gz-physics7-bullet-featherstone-plugin</filename></engine>'
              if ENGINE == 'bullet' else '')
    wp = os.path.join(OUT, f'ftest_{ENGINE}.sdf')
    ground = '<torsional><coefficient>0</coefficient></torsional>' if GROUND == 't0' else ''
    open(wp, 'w').write(WORLD.format(engine=engine, ground=ground))
    sdfs = {v: build(v) for v in VARIANTS}
    names = [f'{v}_{ENGINE}_g{GROUND}' for v in VARIANTS]
    sim = subprocess.Popen(['gz', 'sim', '-s', '-r', wp], stdout=open(os.path.join(OUT, f'gz_{ENGINE}.log'), 'w'),
                           stderr=subprocess.STDOUT, start_new_session=True)
    try:
        for _ in range(60):
            if '/world/ftest/create' in gz('service', '-l').stdout: break
            time.sleep(1)
        for i, v in enumerate(VARIANTS):
            r = gz('service', '-s', '/world/ftest/create', '--reqtype', 'gz.msgs.EntityFactory',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '10000',
                   '--req', f'sdf_filename: "{sdfs[v]}", name: "{names[i]}", '
                            f'pose: {{position: {{x: 0, y: {4.0 * i}, z: 0.1}}}}')
            print(f'[{ENGINE}] spawn {names[i]}: {r.stdout.strip() or r.stderr.strip()}')
        poses = Poses(names); poses.start()
        time.sleep(8)
        missing = [n for n in names if n not in poses.get()]
        if missing:
            print(f'[{ENGINE}] no ground truth for {missing}'); return
        for label, lin, ang in (('spin', 0.0, SPIN), ('straight', LIN, 0.0), ('arc', 0.2, 0.4)):
            command(names, lin, ang)
            time.sleep(3)                                   # let wheels reach speed
            res = measure(poses, names, WINDOW)
            command(names, 0.0, 0.0)
            time.sleep(3)
            for n in names:
                dt, dyaw, path = res[n]
                if ang:
                    print(f'[{ENGINE}] {n} {label}: true rate {dyaw / dt if dt else 0:.4f} rad/s vs commanded '
                          f'{ang:.4f} -> ratio {dyaw / dt / ang if dt else 0:.3f} (over {dt:.1f} s sim)')
                if lin:
                    print(f'[{ENGINE}] {n} {label}: true speed {path / dt if dt else 0:.4f} m/s vs commanded '
                          f'{lin:.4f} -> ratio {path / dt / lin if dt else 0:.3f} (over {dt:.1f} s sim)')
        poses.proc.terminate()
    finally:
        os.killpg(sim.pid, 15)
        time.sleep(3)
        try: os.killpg(sim.pid, 9)
        except ProcessLookupError: pass


if __name__ == '__main__':
    main()
