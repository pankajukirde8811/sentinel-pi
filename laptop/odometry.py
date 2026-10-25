"""
sentinel-pi | laptop/odometry.py   (Phase 5: wheel odometry)

Differential-drive dead reckoning from the signed encoder counts (pos_l, pos_r)
that the Pi sends in every telemetry message.

    ds     = (dl + dr) / 2                 distance of the robot centre
    dtheta = (dr - dl) / track_width       change of heading
    x     += ds * cos(theta + dtheta / 2)  midpoint integration
    y     += ds * sin(theta + dtheta / 2)

Frame: x forward at start, y to the LEFT, theta counter-clockwise (radians).
Because pos_l / pos_r are cumulative counts, a lost telemetry packet loses no distance.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.robot import METRES_PER_COUNT, TRACK_WIDTH_M  # noqa: E402


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class Odometry:
    def __init__(self, m_per_count=METRES_PER_COUNT, track=TRACK_WIDTH_M):
        self.mpc = m_per_count
        self.track = track
        self.reset()

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.dist = 0.0              # total distance travelled by the centre
        self.turned = 0.0            # total signed rotation (not wrapped)
        self._prev = None

    def update(self, pos_l, pos_r):
        """Feed the newest cumulative counts. Returns (x, y, theta)."""
        if self._prev is None:
            self._prev = (pos_l, pos_r)
            return self.x, self.y, self.theta
        dl = (pos_l - self._prev[0]) * self.mpc
        dr = (pos_r - self._prev[1]) * self.mpc
        self._prev = (pos_l, pos_r)
        ds = (dl + dr) / 2
        dth = (dr - dl) / self.track
        self.x += ds * math.cos(self.theta + dth / 2)
        self.y += ds * math.sin(self.theta + dth / 2)
        self.theta = wrap(self.theta + dth)
        self.dist += abs(ds)
        self.turned += dth
        return self.x, self.y, self.theta


if __name__ == "__main__":
    # self-test with known movements
    o = Odometry(m_per_count=0.001, track=0.10)          # 1 mm per count for easy numbers
    o.update(0, 0)
    o.update(1000, 1000)                                  # 1 m straight
    print(f"straight 1 m      -> x={o.x:.3f} y={o.y:.3f} th={math.degrees(o.theta):6.1f} deg  (expect 1, 0, 0)")
    q = round(math.pi * 0.10 / 4 * 1000)                  # quarter turn on the spot: each wheel pi*b/4
    o.update(1000 - q, 1000 + q)
    print(f"+ spin 90 deg     -> x={o.x:.3f} y={o.y:.3f} th={math.degrees(o.theta):6.1f} deg  (expect 1, 0, 90)")
    o.update(2000 - q, 2000 + q)                          # 1 m forward, now along +y
    print(f"+ 1 m forward     -> x={o.x:.3f} y={o.y:.3f} th={math.degrees(o.theta):6.1f} deg  (expect 1, 1, 90)")
    o.reset(); o.update(0, 0)
    r = 0.5                                               # full circle, radius 0.5 m
    n = 400
    for i in range(1, n + 1):
        a = 2 * math.pi * i / n
        o.update(round((r - 0.05) * a * 1000), round((r + 0.05) * a * 1000))
    print(f"circle r=0.5 m    -> x={o.x:.3f} y={o.y:.3f} th={math.degrees(o.theta):6.1f} deg  (expect 0, 0, 0)")
