"""Ordered contours representative of organic walls, not traced from a photo."""
import math
from pathlib import Path
from tests.native_arc_benchmarks import path


def organic_points(n=180,reverse=False):
    points=[]
    for i in range(n+1):
        t=2*math.pi*i/n
        r=105+24*math.cos(3*t)+16*math.sin(5*t)
        points.append((250+1.5*r*math.cos(t),220+r*math.sin(t)))
    points[-1]=points[0]
    return list(reversed(points)) if reverse else points


def cases():
    for n in (60,180):
        yield f'organic_{n}_ccw',path(organic_points(n))
    yield 'organic_180_cw',path(organic_points(180,True))
    for n in (120,360):
        points=[]
        for i in range(n+1):
            t=2*math.pi*i/n; r=100+22*math.cos(4*t)
            points.append((220+r*math.cos(t),180+r*math.sin(t)))
        points[-1]=points[0]
        yield f'four_lobes_{n}',path(points)
    yield 'open_variable_curve',path([(i,40*math.sin(i/37)+18*math.sin(i/11)) for i in range(0,361,3)])
    yield 'figure_eight',path([(200+110*math.sin(i*math.pi/90),180+55*math.sin(i*math.pi/45)) for i in range(181)])
    dense=organic_points(180)
    yield 'nonuniform_organic',path([p for i,p in enumerate(dense) if i%5 not in (2,4)])
    pts=organic_points(120)
    first=path(pts)
    second='G1 Z38\nM3\n'+''.join(f'G1 X{x:.9f} Y{y:.9f} F2400\n' for x,y in pts[1:])+'M5\n'
    yield 'organic_two_layers',first+second
    source=Path('pared_senoidal_concreto_3d.gcode')
    if source.exists(): yield 'user_wall',source.read_text(encoding='utf-8-sig')
