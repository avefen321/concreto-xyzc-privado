"""Bounded planar Hermite/Bezier references; no scientific runtime dependency.

The domain is source chord length, not physical arc length. Curve position and
tangent come from the same polynomial. Closed smooth runs use cyclic tangents.
"""
from dataclasses import dataclass
import bisect
import math
from .numerics import value,derivative,roots
from .orientation import nearest_equivalent_angle,wrap180


def poly_add(a,b,scale=1.):
    out=list(a)+[0.]*(max(len(a),len(b))-len(a))
    for i,x in enumerate(b): out[i]+=scale*x
    return out


def poly_mul(a,b):
    out=[0.]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b): out[i+j]+=x*y
    return out


def polynomial_roots(c):
    scale=max(map(abs,c),default=0.)
    return roots([x/scale for x in c]) if scale else []


def polynomial_range(c):
    candidates=[0.,1.]+polynomial_roots(derivative(c))
    values=[value(c,t) for t in candidates]
    margin=2e-12*max(sum(map(abs,c)),1e-20)
    return min(values)-margin,max(values)+margin


def split_controls(points,t):
    levels=[list(points)]
    while len(levels[-1])>1:
        levels.append([tuple(a[j]+(b[j]-a[j])*t for j in range(3))
                       for a,b in zip(levels[-1],levels[-1][1:])])
    return tuple(level[0] for level in levels),tuple(level[-1] for level in reversed(levels))


def restricted_controls(points,a,b):
    if b<1.: points=split_controls(points,b)[0]
    if a>0.: points=split_controls(points,a/b)[1]
    return points


def coefficients(points,center=(0.,0.)):
    # Translate before forming differences, including for machine offsets.
    p=[(q[0]-center[0],q[1]-center[1]) for q in points]
    return tuple([p[0][j],3*(p[1][j]-p[0][j]),
                  3*(p[2][j]-2*p[1][j]+p[0][j]),
                  p[3][j]-3*p[2][j]+3*p[1][j]-p[0][j]] for j in range(2))


@dataclass(frozen=True)
class BezierPiece:
    controls: tuple
    u0: float
    u1: float
    heading_start: float
    source_error_bound: float
    source_index: int

    def xyz(self,t):
        return split_controls(self.controls,t)[0][-1]

    def tangent(self,t):
        v=[tuple(3*(b[j]-a[j]) for j in range(2)) for a,b in zip(self.controls,self.controls[1:])]
        d=tuple((1-t)**2*v[0][j]+2*t*(1-t)*v[1][j]+t*t*v[2][j] for j in range(2))
        if math.hypot(*d)<1e-12: raise ValueError('Tangente paramétrica degenerada')
        return d

    def heading(self,t):
        d=self.tangent(t)
        return nearest_equivalent_angle(math.degrees(math.atan2(d[1],d[0])),self.heading_start,360.)

    def inflections(self):
        x,y=coefficients(self.controls,self.controls[0][:2])
        dx,dy=derivative(x),derivative(y)
        cross=poly_add(poly_mul(dx,derivative(dy)),poly_mul(dy,derivative(dx)),-1.)
        return [self.u0+(self.u1-self.u0)*t for t in polynomial_roots(cross) if 1e-7<t<1-1e-7]


class ParametricReference:
    def __init__(self,stroke,field,start,end,budget):
        self.stroke=stroke; self.start=start; self.end=end; self.pieces=[]
        self.u=[field.s[start]]
        points=stroke.points[start:end+1]
        h=[math.dist(a.xyz[:2],b.xyz[:2]) for a,b in zip(points,points[1:])]
        headings=list(field.point_heading[start:end+1])
        closed=(start==0 and end==len(stroke.points)-1 and len(points)>3 and
                math.dist(points[0].xyz,points[-1].xyz)<1e-8)
        global_closed=(len(stroke.points)>3 and math.dist(stroke.points[0].xyz,stroke.points[-1].xyz)<1e-8)
        if (global_closed and (start==0 or end==len(stroke.points)-1) and
                stroke.points[0].heading_override_deg is None and stroke.points[-1].heading_override_deg is None):
            left,right=field.seg_length[-1],field.seg_length[0]
            a,b,c=stroke.points[-2].xyz,stroke.points[0].xyz,stroke.points[1].xyz
            d=tuple(-right*a[j]/(left*(left+right))+(right-left)*b[j]/(left*right)+
                    left*c[j]/(right*(left+right)) for j in range(2))
            head=math.degrees(math.atan2(d[1],d[0]))
            if start==0: headings[0]=nearest_equivalent_angle(head,headings[0],360.)
            if end==len(stroke.points)-1: headings[-1]=nearest_equivalent_angle(head,headings[-1],360.)
        tangents=[(math.cos(math.radians(a)),math.sin(math.radians(a))) for a in headings]
        scales=[1.]*len(points)
        for i,(a,b,length) in enumerate(zip(points,points[1:],h)):
            direction=((b.xyz[0]-a.xyz[0])/length,(b.xyz[1]-a.xyz[1])/length)
            for k in (i,i+1):
                dot=sum(tangents[k][j]*direction[j] for j in range(2))
                if dot<=1e-8: raise ValueError('Tangente incompatible con el orden local de puntos')
                cross=abs(tangents[k][0]*direction[1]-tangents[k][1]*direction[0])
                scales[k]=min(scales[k],1.5/dot,3*budget/(length*cross) if cross>1e-14 else 1.)
        if closed: scales[0]=scales[-1]=min(scales[0],scales[-1])
        for i,(a,b,length) in enumerate(zip(points,points[1:],h)):
            controls=(a.xyz,tuple(a.xyz[j]+length*scales[i]*tangents[i][j]/3 if j<2 else a.xyz[j] for j in range(3)),
                      tuple(b.xyz[j]-length*scales[i+1]*tangents[i+1][j]/3 if j<2 else b.xyz[j] for j in range(3)),b.xyz)
            direction=((b.xyz[0]-a.xyz[0])/length,(b.xyz[1]-a.xyz[1])/length)
            cx,cy=coefficients(controls,a.xyz[:2])
            normal=poly_add([direction[0]*v for v in cy],[direction[1]*v for v in cx],-1.)
            lo,hi=polynomial_range(normal); bound=max(abs(lo),abs(hi))
            if bound>budget+1e-8: raise ValueError('Reconstrucción excede su presupuesto geométrico')
            self.pieces.append(BezierPiece(controls,self.u[-1],self.u[-1]+length,headings[i],bound,start+i))
            self.u.append(self.u[-1]+length)
        self.closed=closed
        self.source_error_bound=max(p.source_error_bound for p in self.pieces)

    def location(self,u):
        i=max(0,min(len(self.pieces)-1,bisect.bisect_right(self.u,u)-1))
        return i,max(0.,min(1.,(u-self.u[i])/(self.u[i+1]-self.u[i])))

    def xyz(self,u):
        i,t=self.location(u); return self.pieces[i].xyz(t)

    def heading(self,u):
        i,t=self.location(u); return self.pieces[i].heading(t)

    def portions(self,a,b):
        i,_=self.location(a)
        for piece in self.pieces[i:]:
            if piece.u0>=b-1e-12: break
            lo=max(a,piece.u0); hi=min(b,piece.u1)
            if hi>lo+1e-12:
                yield piece,restricted_controls(piece.controls,(lo-piece.u0)/(piece.u1-piece.u0),
                                               (hi-piece.u0)/(piece.u1-piece.u0)),lo,hi

    def cuts(self):
        result=[self.u[0],self.u[-1]]
        for piece in self.pieces: result.extend(piece.inflections())
        # Curvature may change sign across a knot without an interior zero.
        for i in range(1,len(self.pieces)):
            def sign(piece,t):
                x,y=coefficients(piece.controls,piece.controls[0][:2]); dx,dy=derivative(x),derivative(y)
                return value(dx,t)*value(derivative(dy),t)-value(dy,t)*value(derivative(dx),t)
            if sign(self.pieces[i-1],1.)*sign(self.pieces[i],0.)<0: result.append(self.u[i])
        if self.closed:
            result.extend(self.u[0]+(self.u[-1]-self.u[0])*k/4 for k in (1,2,3))
        tolerance=max(1e-9,(self.u[-1]-self.u[0])*1e-12)
        distinct=[]
        for u in sorted(result):
            if not distinct or u-distinct[-1]>tolerance: distinct.append(u)
            elif abs(u-self.u[-1])<tolerance: distinct[-1]=self.u[-1]
        return distinct
