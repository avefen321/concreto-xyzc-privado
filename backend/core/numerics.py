"""Small polynomial helpers for exact extrema of the existing cubic/quintic fields."""
import math


def value(coefficients, x):
    out = 0.0
    for c in reversed(coefficients):
        out = out*x+c
    return out


def derivative(c):
    return [i*c[i] for i in range(1,len(c))]


def roots(c, a=0.0, b=1.0):
    c=list(c)
    while len(c)>1 and abs(c[-1])<1e-13:
        c.pop()
    if len(c)<=1:
        return []
    if len(c)==2:
        root=-c[0]/c[1]
        return [root] if a < root < b else []
    boundaries=[a]+roots(derivative(c),a,b)+[b]
    found=[]
    for left,right in zip(boundaries,boundaries[1:]):
        fl,fr=value(c,left),value(c,right)
        if abs(fl)<1e-10 and a<left<b:
            found.append(left)
        if fl*fr>=0:
            continue
        for _ in range(50):
            mid=(left+right)/2
            fm=value(c,mid)
            if fl*fm<=0:
                right=mid
            else:
                left=mid; fl=fm
        found.append((left+right)/2)
    return sorted(set(found))


def compose_affine(c, offset, scale):
    out=[0.0]*len(c)
    for power,coef in enumerate(c):
        for j in range(power+1):
            out[j]+=coef*math.comb(power,j)*offset**(power-j)*scale**j
    return out


def max_abs(c,a=0.0,b=1.0):
    candidates=[a,b]+roots(derivative(c),a,b)
    return max((abs(value(c,t)),t) for t in candidates)
