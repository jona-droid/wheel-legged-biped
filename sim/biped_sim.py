"""
MuJoCo simulation of the wheel-legged biped, running the SAME controller as the firmware.

    pip install mujoco numpy matplotlib
    python biped_sim.py                    # stage 4: legs fixed at 110 mm, push test -> sim_push.png
    python biped_sim.py --test height      # stage 6: crouch to 75 mm, stand to 140 mm while balancing
    python biped_sim.py --test turn        # stage 6: drive in a circle, lean into the turn
    python biped_sim.py --depth 80         # balance at another fixed leg height
    python biped_sim.py --view             # 3D viewer
    python biped_sim.py --search           # grid-search robust balance gains
    python biped_sim.py --write-xml        # just write biped.xml (open it with `python -m mujoco.viewer`)

The model is generated from the same numbers as cad/wheel_biped.scad and leg_kinematics.py:
each leg is a closed five-bar loop (two cranks on position-controlled servos, link A and
link B joined by an equality constraint), the wheels are DC motors driven by a -1..1 command.

For RL later: build_model() + Firmware-free stepping is all you need for a Gymnasium env.
Observation: IMU pitch/roll + rates, wheel speed estimate, leg height, commands.
Action: two wheel commands (+ later the four servo targets). Randomise the knobs below.
"""
import argparse
import math
import os

import mujoco
import numpy as np

import leg_kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- geometry (metres), from the CAD ----------------
X_CRANK = 0.0497      # mid-plane of the cranks
X_A = 0.0583          # link A plane (608 bosses, 7.6 mm)
X_B = 0.0669          # link B plane
X_WHEEL = 0.0767      # wheel centre plane
X_PIN = 0.0626        # the knee joint (between the A and B planes)
WHEEL_R = 0.0335

# ---------------- masses (kg), from the STL volumes + parts ----------------
M = dict(servo=0.055, chassis=0.041, deck=0.022 + 0.045,   # deck + ESP32, MPU, MX1508, bucks, wires
         top=0.024 + 0.005, battery=0.107, crank=0.006, link_a=0.004 + 0.024, link_b=0.009 + 0.012,   # links incl. their 608s (12 g each)
         motor=0.010, wheel=0.013, screws=0.030)

# ---------------- N20 300 rpm on 2S, duty capped to ~6 V ----------------
STALL_TORQUE = 0.10   # N m at 6 V (datasheets say 0.1-0.2; we stay pessimistic)
NO_LOAD_SPEED = 31.0  # rad/s (300 rpm)
DEADBAND = 0.10
LOOP_HZ = 200

# ---------------- MG996R as a position servo ----------------
SERVO_KP = 6.0        # N m / rad
SERVO_FMAX = 1.0      # N m (stall ~1.1 at 6 V)
SERVO_SPEED = math.radians(60) / 0.17   # rad/s no-load (the firmware also ramps)


def xml(depth=0.110):
    """Return the MJCF text. Joint angles at qpos0 are 0 (all links hang straight down);
    simulate() sets the real starting pose from inverse kinematics."""
    def leg(s, name):
        x = lambda v: f"{s * v:.4f}"
        return f"""
      <body name="{name}_crankF" pos="{x(X_CRANK)} {K.A:.4f} 0">
        <joint name="{name}_hipF" axis="1 0 0" damping="0.02" armature="0.002"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-K.L1:.4f}" size="0.005" mass="{M['crank']}" rgba="1 .6 0 1"/>
        <body name="{name}_linkA" pos="{x(X_A - X_CRANK)} 0 {-K.L1:.4f}">
          <joint name="{name}_kneeA" axis="1 0 0" damping="0.001"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-K.L2:.4f}" size="0.004" mass="{M['link_a']}" rgba=".55 .8 .9 1"/>
          <site name="{name}_pinA" pos="{x(X_PIN - X_A)} 0 {-K.L2:.4f}" size="0.003"/>
        </body>
      </body>
      <body name="{name}_crankR" pos="{x(X_CRANK)} {-K.A:.4f} 0">
        <joint name="{name}_hipR" axis="1 0 0" damping="0.02" armature="0.002"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-K.L1:.4f}" size="0.005" mass="{M['crank']}" rgba="1 .6 0 1"/>
        <body name="{name}_linkB" pos="{x(X_B - X_CRANK)} 0 {-K.L1:.4f}">
          <joint name="{name}_kneeB" axis="1 0 0" damping="0.001"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-K.LB:.4f}" size="0.004" mass="{M['link_b']}" rgba=".2 .4 .6 1"/>
          <geom type="box" pos="{x(-0.010)} 0 {-K.LB:.4f}" size="0.012 0.006 0.005" mass="{M['motor']}" rgba=".7 .7 .7 1"/>
          <site name="{name}_pinB" pos="{x(X_PIN - X_B)} 0 {-K.L2:.4f}" size="0.003"/>
          <body name="{name}_wheel" pos="{x(X_WHEEL - X_B)} 0 {-K.LB:.4f}">
            <joint name="{name}_wheel" axis="-1 0 0" damping="0.0002" frictionloss="0.002"/>
            <geom name="{name}_tire" type="cylinder" size="{WHEEL_R} 0.005" euler="0 90 0" mass="{M['wheel']}"
                  contype="1" conaffinity="1" friction="1.0 0.01 0.001" rgba=".2 .2 .2 1"/>
          </body>
        </body>
      </body>"""

    servo_geoms = "\n".join(
        f'      <geom type="box" pos="{sx * 0.016:.3f} {sy * K.A:.4f} -0.0102" size="0.0145 0.0099 0.0204" '
        f'mass="{M["servo"]}" rgba=".1 .1 .1 1"/>' for sx in (-1, 1) for sy in (-1, 1))
    z0 = depth + WHEEL_R + 0.002
    return f"""<mujoco model="wheel_biped">
  <compiler angle="degree"/>
  <option timestep="0.0005" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".25 .26 .28" rgb2=".32 .33 .35" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".1"/>
  </asset>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <light pos="0 -1 2" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid" contype="1" conaffinity="1"/>
    <body name="chassis" pos="0 0 {z0:.4f}">
      <freejoint name="root"/>
      <!-- origin = midway between the servo shafts -->
{servo_geoms}
      <geom name="box" type="box" pos="0 0 -0.012" size="0.0315 0.032 0.0295" mass="{M['chassis'] + M['screws']}" rgba=".27 .51 .71 .4"
            contype="1" conaffinity="1"/>
      <geom type="box" pos="0 0 0.026" size="0.052 0.042 0.008" mass="{M['deck']}" rgba=".7 .75 .85 1"/>
      <geom type="box" pos="0 0 0.063" size="0.052 0.042 0.002" mass="{M['top']}" rgba=".7 .75 .85 1"/>
      <geom name="battery" type="box" pos="0 0 0.075" size="0.039 0.0205 0.010" mass="{M['battery']}" rgba="0 .5 0 1"/>
      <site name="imu" pos="0 0.006 0.023"/>
{leg(1, 'R')}
{leg(-1, 'L')}
    </body>
  </worldbody>
  <equality>
    <connect site1="R_pinA" site2="R_pinB" solref="0.002 1"/>
    <connect site1="L_pinA" site2="L_pinB" solref="0.002 1"/>
  </equality>
  <actuator>
    <position name="R_hipF" joint="R_hipF" kp="{SERVO_KP}" forcerange="-{SERVO_FMAX} {SERVO_FMAX}" forcelimited="true"/>
    <position name="R_hipR" joint="R_hipR" kp="{SERVO_KP}" forcerange="-{SERVO_FMAX} {SERVO_FMAX}" forcelimited="true"/>
    <position name="L_hipF" joint="L_hipF" kp="{SERVO_KP}" forcerange="-{SERVO_FMAX} {SERVO_FMAX}" forcelimited="true"/>
    <position name="L_hipR" joint="L_hipR" kp="{SERVO_KP}" forcerange="-{SERVO_FMAX} {SERVO_FMAX}" forcelimited="true"/>
    <motor name="R_wheel" joint="R_wheel"/>
    <motor name="L_wheel" joint="L_wheel"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""


def leg_pose(depth, y=0.0):
    """Joint angles (radians) for one leg: hipF, kneeA, hipR, kneeB."""
    qf, qr, k = K.ik(y, -depth)
    a_abs = math.atan2(k["P"][0] - k["EF"][0], -(k["P"][1] - k["EF"][1]))
    b_abs = math.atan2(k["W"][0] - k["ER"][0], -(k["W"][1] - k["ER"][1]))
    qf, qr = math.radians(qf), math.radians(qr)
    return qf, a_abs - qf, qr, b_abs - qr


class Firmware:
    """Port of controlTask() in firmware/src/biped_main.cpp (balance + speed + legs)."""

    def __init__(self, kp, ki, kd, trim, kv, kvi, alpha=0.98):
        self.kp, self.ki, self.kd, self.trim, self.kv, self.kvi = kp, ki, kd, trim, kv, kvi
        self.alpha = alpha
        self.pitch = None
        self.roll = 0.0
        self.angle_i = self.speed_i = self.speed_est = self.speed_target = 0.0
        self.turn = 0.0
        self.out = 0.0
        self.armed = True
        self.height = 0.110
        self.roll_i = 0.0
        self.lean = 0.0

    def step(self, acc_pitch, gyro_rate, acc_roll, roll_rate, yaw_rate, want_speed, want_turn, want_height, dt,
             lean_gain=1.0, roll_kp=0.0):
        if self.pitch is None:
            self.pitch = acc_pitch
        self.pitch = self.alpha * (self.pitch + gyro_rate * dt) + (1 - self.alpha) * acc_pitch
        self.roll = self.alpha * (self.roll + roll_rate * dt) + (1 - self.alpha) * acc_roll
        angle = self.pitch
        if abs(angle) > 40:
            self.armed = False
        # legs: ramp height, lean into turns (outer leg longer)
        self.height += float(np.clip(want_height - self.height, -0.05 * dt, 0.05 * dt))
        if not self.armed:
            self.out = 0.0
            return 0.0, 0.0, angle, self.trim, self.height, 0.0
        self.speed_target += float(np.clip(want_speed - self.speed_target, -0.6 * dt, 0.6 * dt))
        self.turn += float(np.clip(want_turn - self.turn, -1.5 * dt, 1.5 * dt))
        self.speed_est += (self.out - self.speed_est) * (dt / 0.3)
        speed_err = self.speed_target - self.speed_est
        self.speed_i = float(np.clip(self.speed_i + self.kvi * speed_err * dt, -6, 6))
        target = float(np.clip(self.trim + self.kv * speed_err + self.speed_i, self.trim - 10, self.trim + 10))
        err = angle - target
        self.angle_i += err * dt
        if self.ki > 0:
            lim = 1.0 / self.ki
            self.angle_i = float(np.clip(self.angle_i, -lim, lim))
        out = self.kp * err + self.ki * self.angle_i + self.kd * gyro_rate
        self.out = float(np.clip(out, -1, 1))
        # lean into turns: centripetal accel = v * yaw rate. v is estimated from the wheel command
        # (no encoders), the yaw rate comes straight from the gyro's Z axis (deg/s, + = turning left).
        v = self.speed_est * NO_LOAD_SPEED * WHEEL_R                   # m/s
        lean = math.atan2(-v * math.radians(yaw_rate), 9.81) * lean_gain   # radians, + = lean right
        self.lean += (lean - self.lean) * (dt / 0.25)                  # smooth it (gyro noise, servo jitter)
        dh = math.tan(self.lean) * X_WHEEL                             # +: right leg shorter, left longer
        dh += roll_kp * math.radians(self.roll) * X_WHEEL               # level on slopes
        dh = float(np.clip(dh, -0.03, 0.03))
        return self.out, self.turn, angle, target, self.height, dh


def deadband_comp(u):
    if abs(u) < 0.005:
        return 0.0
    return math.copysign(min(DEADBAND + abs(u) * (1 - DEADBAND), 1.0), u)


def build_model(depth=0.110):
    return mujoco.MjModel.from_xml_string(xml(depth))


def set_pose(m, d, depth_r, depth_l=None):
    depth_l = depth_r if depth_l is None else depth_l
    for side, dep in (("R", depth_r), ("L", depth_l)):
        q = leg_pose(dep)
        for jn, v in zip(("hipF", "kneeA", "hipR", "kneeB"), q):
            d.joint(f"{side}_{jn}").qpos[0] = v


def simulate(gains, T=10.0, depth=0.110, start_tilt=2.0, push_at=(3.0,), push_force=1.2,
             drive=None, turn=None, height=None, motor=1.0, com=0.0, latency_steps=2,
             noise=True, seed=0, viewer=False, lean_gain=0.5):
    rng = np.random.default_rng(seed)
    m = build_model(depth)
    m.geom_pos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "battery")][1] += com
    for j in ("L_wheel", "R_wheel"):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        m.dof_frictionloss[m.jnt_dofadr[jid]] = DEADBAND * STALL_TORQUE * motor * 0.9
    d = mujoco.MjData(m)
    set_pose(m, d, depth)
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([-1.0, 0, 0]), math.radians(start_tilt))
    d.qpos[3:7] = q
    # settle the closed loops with the servos holding the pose
    fw = Firmware(**gains)
    fw.height = depth
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    def servo_targets(hr, hl):
        for side, dep in (("R", hr), ("L", hl)):
            qf, _, qr, _ = leg_pose(float(np.clip(dep, 0.070, 0.145)))
            d.actuator(f"{side}_hipF").ctrl[0] = qf
            d.actuator(f"{side}_hipR").ctrl[0] = qr

    servo_targets(depth, depth)
    mujoco.mj_forward(m, d)
    fw.pitch = start_tilt
    steps_per_ctrl = int(round(1 / (LOOP_HZ * m.opt.timestep)))
    queue = [(0.0, 0.0)] * (latency_steps + 1)
    log = {k: [] for k in ("t", "true", "est", "target", "out", "y", "x", "h", "roll", "z", "yaw")}
    u, tr = 0.0, 0.0
    v = None
    if viewer:
        from mujoco import viewer as mjviewer
        v = mjviewer.launch_passive(m, d)
    n = int(T / m.opt.timestep)
    fell = False
    for i in range(n):
        t = i * m.opt.timestep
        if i % steps_per_ctrl == 0:
            gyro = d.sensor("gyro").data.copy()
            acc = d.sensor("accel").data.copy()
            if noise:
                gyro += rng.normal(0, 0.5 * math.pi / 180, 3) + 0.3 * math.pi / 180
                acc += rng.normal(0, 0.3, 3)
            acc_pitch = math.degrees(math.atan2(-acc[1], acc[2]))
            rate = -math.degrees(gyro[0])
            acc_roll = math.degrees(math.atan2(acc[0], acc[2]))
            roll_rate = math.degrees(gyro[1])
            yaw_rate = math.degrees(gyro[2])
            want = drive(t) if drive else 0.0
            wturn = turn(t) if turn else 0.0
            wh = height(t) if height else depth
            cmd, trn, est, target, h, dh = fw.step(acc_pitch, rate, acc_roll, roll_rate, yaw_rate, want, wturn, wh,
                                                    1 / LOOP_HZ, lean_gain=lean_gain)
            servo_targets(h - dh, h + dh)
            queue.append((cmd, trn))
            c, tr = queue.pop(0)
            u = c
            R = d.xmat[body].reshape(3, 3)
            true_pitch = math.degrees(math.atan2(R[1, 2], R[2, 2]))
            true_roll = math.degrees(math.asin(np.clip(-R[2, 0], -1, 1)))
            log["t"].append(t); log["true"].append(true_pitch); log["est"].append(est)
            log["target"].append(target); log["out"].append(cmd); log["y"].append(d.qpos[1]); log["x"].append(d.qpos[0])
            log["h"].append(h); log["roll"].append(true_roll); log["z"].append(d.qpos[2])
            log["yaw"].append(math.degrees(math.atan2(R[1, 0], R[0, 0])))
            if abs(true_pitch) > 45 or d.qpos[2] < 0.06:
                fell = True
        for side, sgn in (("R", -1), ("L", +1)):
            w = d.joint(f"{side}_wheel").qvel[0]
            cmd_w = deadband_comp(float(np.clip(u + sgn * tr, -1, 1)))
            d.actuator(f"{side}_wheel").ctrl[0] = STALL_TORQUE * motor * (cmd_w - w / NO_LOAD_SPEED)
        d.xfrc_applied[body] = 0
        for pt in push_at:
            if pt <= t < pt + 0.1:
                d.xfrc_applied[body][1] = push_force
        mujoco.mj_step(m, d)
        if v is not None and i % 33 == 0:
            v.sync()
    for k in log:
        log[k] = np.array(log[k])
    return fell, log


DEFAULT = dict(kp=0.12, ki=3.5, kd=0.004, trim=0.0, kv=8.0, kvi=2.0)   # from --search


def robust_score(g):
    cases = [dict(), dict(motor=0.6), dict(motor=1.4), dict(com=0.004), dict(latency_steps=4),
             dict(depth=0.075), dict(depth=0.140), dict(push_force=2.0)]
    score = 0.0
    for c in cases:
        fell, log = simulate(g, T=8, push_at=(3.0,), **c)
        if fell:
            return 1e9
        score += np.mean(np.abs(log["true"][200:])) + 20 * abs(log["y"][-1])
    return score


def search():
    import itertools
    best = (1e9, None)
    for kp, ki, kd in itertools.product([0.05, 0.08, 0.12], [2.0, 3.5, 5.0], [0.004, 0.006, 0.009]):
        g = dict(kp=kp, ki=ki, kd=kd, trim=0.0, kv=8.0, kvi=2.0)
        s = robust_score(g)
        print(f"{g} -> {s:.2f}", flush=True)
        if s < best[0]:
            best = (s, g)
    print("\nBEST:", best)


def plot(log, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(log["t"], log["true"], label="true tilt")
    ax[0].plot(log["t"], log["est"], "--", label="filtered IMU tilt")
    ax[0].plot(log["t"], log["target"], label="target tilt")
    ax[0].set_ylabel("pitch (deg)"); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_title(title)
    ax[1].plot(log["t"], log["out"], color="tab:orange"); ax[1].set_ylabel("wheel cmd"); ax[1].grid(alpha=.3)
    ax[2].plot(log["t"], log["h"] * 1000, label="leg height cmd (mm)")
    ax[2].plot(log["t"], log["roll"], label="roll (deg)")
    ax[2].legend(); ax[2].grid(alpha=.3)
    ax[3].plot(log["t"], log["y"] * 100, color="tab:green", label="forward position (cm)")
    ax[3].plot(log["t"], log["x"] * 100, color="tab:purple", label="sideways position (cm)")
    ax[3].legend(); ax[3].set_xlabel("time (s)"); ax[3].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(path, dpi=100)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    for k, v in DEFAULT.items():
        ap.add_argument(f"--{k}", type=float, default=v)
    ap.add_argument("--depth", type=float, default=110, help="fixed leg height (wheel axle below shafts), mm")
    ap.add_argument("--motor", type=float, default=1.0)
    ap.add_argument("--com", type=float, default=0.0)
    ap.add_argument("--test", choices=["push", "height", "turn", "drive"], default="push")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--search", action="store_true")
    ap.add_argument("--write-xml", action="store_true")
    a = ap.parse_args()
    if a.write_xml:
        open(os.path.join(HERE, "biped.xml"), "w").write(xml())
        print("wrote biped.xml")
    elif a.search:
        search()
    else:
        g = {k: getattr(a, k) for k in DEFAULT}
        kw = dict(depth=a.depth / 1000, motor=a.motor, com=a.com, viewer=a.view)
        if a.test == "push":
            fell, log = simulate(g, T=10, push_at=(3.0, 6.5), **kw)
        elif a.test == "drive":
            fell, log = simulate(g, T=12, push_at=(), drive=lambda t: 0.35 if 2 < t < 5 else (-0.35 if 7 < t < 9 else 0), **kw)
        elif a.test == "height":
            def hcmd(t):
                return 0.110 if t < 2 else (0.075 if t < 5 else (0.140 if t < 8.5 else 0.110))
            fell, log = simulate(g, T=11, push_at=(), height=hcmd, **kw)
        else:
            fell, log = simulate(g, T=14, push_at=(), drive=lambda t: 0.40 if t > 1 else 0,
                                 turn=lambda t: 0.2 if 4 < t < 11 else 0, **kw)
        print("FELL" if fell else "stayed up",
              f"| final position y {log['y'][-1]*100:.1f} cm, x {log['x'][-1]*100:.1f} cm | max |roll| {np.max(np.abs(log['roll'])):.1f} deg")
        out = os.path.join(os.getcwd(), f"sim_{a.test}.png")
        plot(log, out, f"test: {a.test}")
        print("saved", out)
