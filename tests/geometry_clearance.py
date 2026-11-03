"""Planar clearance between the actual robot footprint and a rectangular test obstacle."""
import math


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in points]


def clearance(a, b):
    separated = False
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            axis = (-(q[1]-p[1]), q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x, y in a]
            pb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                separated = True
    if not separated:
        return 0.0

    def point_segment(p, q, r):
        dx, dy = r[0]-q[0], r[1]-q[1]
        length2 = dx*dx+dy*dy
        t = max(0., min(1., ((p[0]-q[0])*dx+(p[1]-q[1])*dy)/length2)) if length2 else 0.
        return math.hypot(p[0]-q[0]-t*dx, p[1]-q[1]-t*dy)
    return min(point_segment(p, q, r) for first, second in [(a,b),(b,a)]
               for p in first for q, r in zip(second, second[1:]+second[:1]))


def circle_clearance(polygon, x, y, radius):
    signs = []
    distances = []
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        signs.append(dx*(y-a[1])-dy*(x-a[0]))
        length2 = dx*dx+dy*dy
        t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/length2)) if length2 else 0.
        distances.append(math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
    if all(s >= 0 for s in signs) or all(s <= 0 for s in signs):
        return 0.
    return max(0., min(distances)-radius)


if __name__ == '__main__':
    square = [(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]
    assert abs(clearance(square, transform(square,2,0,0))-1.) < 1e-9
    assert clearance(square, transform(square,1,0,0)) == 0
    assert clearance(square, transform(square,0,0,.3)) == 0
    assert abs(clearance(square,transform(square,2,2,0))-math.sqrt(2)) < 1e-9
    print('PASS: rectangle separation, corner distance, rotated overlap and contact')
