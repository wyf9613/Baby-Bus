"""Shared ordered-reference Frenet chart; no candidate generation."""
import math
from .geometry import wrap_angle


def _shadow_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1]+math.dist(a, b))
    return lengths


def _shadow_at(points, lengths, s):
    """Piecewise linear arc chart with explicitly inferred tangent tails."""
    if len(points) == 1:
        return points[0][0]+s, points[0][1]
    if s <= 0:
        i = 0
    elif s >= lengths[-1]:
        i = len(points)-2
    else:
        from bisect import bisect_left
        i = bisect_left(lengths, s)-1
    u = (s-lengths[i])/(lengths[i+1]-lengths[i])
    return tuple(points[i][k]+u*(points[i+1][k]-points[i][k]) for k in (0, 1))


class ShadowFrame:
    """Local two-dimensional Frenet chart; no monotonic-body-x restriction."""
    def __init__(self, points):
        self.points = points
        self.lengths = _shadow_lengths(points)
        self.end = self.lengths[-1]

    def at(self, s):
        p = _shadow_at(self.points, self.lengths, s)
        a = _shadow_at(self.points, self.lengths, s-0.04)
        b = _shadow_at(self.points, self.lengths, s+0.04)
        length = math.dist(a, b)
        tangent = ((b[0]-a[0])/length, (b[1]-a[1])/length) if length > 1e-9 else (1.0, 0.0)
        return p, tangent, (-tangent[1], tangent[0])

    def project(self, point, heading=0.0):
        options = []
        if len(self.points) == 1:
            s = point[0]-self.points[0][0]
            return s, point[1]-self.points[0][1]
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            length = self.lengths[i+1]-self.lengths[i]
            tx, ty = (b[0]-a[0])/length, (b[1]-a[1])/length
            u = ((point[0]-a[0])*tx+(point[1]-a[1])*ty)/length
            # The first/last rays are inferred chart support, not hard boundaries.
            u = max(0.0, u) if i else u
            u = min(1.0, u) if i < len(self.points)-2 else u
            q = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
            angle = wrap_angle(math.atan2(ty, tx)-heading)
            cost = math.dist(q, point)**2+0.02*angle**2
            options.append((cost, self.lengths[i]+u*length))
        s = min(options)[1]
        p, _, n = self.at(s)
        return s, (point[0]-p[0])*n[0]+(point[1]-p[1])*n[1]
