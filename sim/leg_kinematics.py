"""
Leg kinematics for the wheel-legged biped (one leg, side view).

Each leg is a planar FIVE-BAR linkage. Both servos sit in the body, so the leg itself is light:

      SF (front servo)    SR (rear servo)       y = forward, z = up, origin = midway between the shafts
          o-----2a-----o
           \\           /       crank   L1  (bolted to the servo horn)
            o EF   ER o
             \\       /         link A (front) L2: EF -> P
              \\     /          link B (rear)  Lb: ER -> P -> W  (straight, longer: carries the wheel motor)
               \\   /
                 P              pin joint where link A meets link B
                 |
                 W  (wheel axle, e = Lb - L2 below P along link B)

Functions:
  ik(y, z)       -> crank angles (front, rear) in degrees, measured from straight DOWN, positive = toward the front
  fk(qf, qr)     -> wheel position (y, z)
  torques(y, z, F) -> servo torques for a vertical load F (N) on the wheel

Run it to print the table used in the README and to pick the link lengths.
"""
import math
import numpy as np

A = 0.0135     # half spacing of the two servo shafts (m)
L1 = 0.055     # crank length
L2 = 0.075     # link A length (EF -> P)
LB = 0.100     # link B length (ER -> W)  ->  P sits L2 along it
E = LB - L2


def _circle_x(c0, r0, c1, r1, pick):
    """Intersection of two circles. pick=+1/-1 chooses the side (left/right of c0->c1)."""
    d = np.linalg.norm(c1 - c0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = math.sqrt(max(r0**2 - a**2, 0))
    m = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    return m + pick * h * perp


SF = np.array([A, 0.0])
SR = np.array([-A, 0.0])


def crank_tip(s, q):
    q = math.radians(q)
    return s + L1 * np.array([math.sin(q), -math.cos(q)])


def _both(c0, r0, c1, r1):
    out = [_circle_x(c0, r0, c1, r1, p) for p in (+1, -1)]
    return [o for o in out if o is not None]


def ik(y, z):
    """Elbows-out branch: front crank swings forward, rear crank swings backward."""
    W = np.array([y, z])
    c = _both(SR, L1, W, LB)
    if not c:
        return None
    ER = min(c, key=lambda p: p[0])            # rear elbow: the more backward solution
    P = ER + (W - ER) * (L2 / LB)
    c = _both(SF, L1, P, L2)
    if not c:
        return None
    EF = max(c, key=lambda p: p[0])            # front elbow: the more forward solution
    qr = math.degrees(math.atan2(ER[0] - SR[0], -(ER[1] - SR[1])))
    qf = math.degrees(math.atan2(EF[0] - SF[0], -(EF[1] - SF[1])))
    return qf, qr, dict(ER=ER, EF=EF, P=P, W=W)


def fk(qf, qr):
    """Wheel position (y, z) from the two crank angles (degrees)."""
    EF, ER = crank_tip(SF, qf), crank_tip(SR, qr)
    c = _both(ER, L2, EF, L2)
    if not c:
        return None
    P = min(c, key=lambda p: p[1])             # the knee point below the elbows
    return ER + (P - ER) * (LB / L2)


def torques(y, z, F=4.0, h=1e-5):
    """Servo torques (N m) holding a vertical ground reaction F on the wheel (virtual work)."""
    q = ik(y, z)
    if q is None:
        return None
    qf, qr = math.radians(q[0]), math.radians(q[1])
    def fkr(a, b):
        return fk(math.degrees(a), math.degrees(b))
    J = np.zeros((2, 2))
    for i, dq in enumerate(([h, 0], [0, h])):
        p1 = fkr(qf + dq[0], qr + dq[1]); p0 = fkr(qf - dq[0], qr - dq[1])
        J[:, i] = (p1 - p0) / (2 * h)
    return J.T @ np.array([0.0, F])


if __name__ == "__main__":
    print(f"L1={L1*1000:.0f} L2={L2*1000:.0f} LB={LB*1000:.0f} a={A*1000:.0f} mm")
    print(" wheel depth | front crank | rear crank | torque F/R (N m) at 4 N load")
    for d in range(60, 171, 10):
        r = ik(0.0, -d / 1000)
        if r is None:
            print(f"   {d:4d} mm  | unreachable"); continue
        t = torques(0.0, -d / 1000, 4.0)
        chk = fk(r[0], r[1])
        print(f"   {d:4d} mm  |  {r[0]:7.1f}    | {r[1]:7.1f}    | {t[0]:+.3f} / {t[1]:+.3f}   fk err {np.linalg.norm(chk - r[2]['W'])*1e6:.1f} um")
