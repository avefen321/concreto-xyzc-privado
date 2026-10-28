"""Nominal geometric screening only: no material or real head collision model."""
import math


def rectangle(x,y,physical_c,length=35.,width=15.):
    angle=math.radians(physical_c); co,si=math.cos(angle),math.sin(angle)
    return [(x+i*length/2*co-j*width/2*si,y+i*length/2*si+j*width/2*co)
            for i,j in ((-1,-1),(1,-1),(1,1),(-1,1))]


def inspect_footprint(primitives,profile,max_samples=400):
    poses=[]; accumulated=0.
    for p in primitives:
        if p.process_state!='DEPOSIT': continue
        n=max(1,math.ceil(abs(math.degrees(p.sweep_rad))/10)) if p.type=='ARC' else 1
        shift=p.metadata.get('c_reference_shift_deg',0.)
        for k in range(n+1):
            pose=p.pose(k/n)
            poses.append((pose[0],pose[1],pose[3]-shift,p.stroke_id,p.layer_id,accumulated+p.length_mm*k/n))
        accumulated+=p.length_mm
    if not poses: return {'status':'NOT_APPLICABLE','physical_collision_verified':False}
    if len(poses)>max_samples:
        poses=[poses[round(i*(len(poses)-1)/(max_samples-1))] for i in range(max_samples)]
    polygons=[rectangle(x,y,c,profile.length_mm,profile.width_mm) for x,y,c,_,_,_ in poses]
    corners=[q for polygon in polygons for q in polygon]
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    crossings=0
    for i,(a,b) in enumerate(zip(poses,poses[1:])):
        if a[3:5]!=b[3:5]: continue
        for c,d in zip(poses[i+3:],poses[i+4:]):
            if c[3:5]!=a[3:5] or d[3:5]!=a[3:5]: continue
            if cross(a,b,c)*cross(a,b,d)<-1e-12 and cross(c,d,a)*cross(c,d,b)<-1e-12: crossings+=1
    def overlap(first,second):
        for polygon in (first,second):
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                axis=(b[1]-a[1],a[0]-b[0])
                one=[q[0]*axis[0]+q[1]*axis[1] for q in first]
                two=[q[0]*axis[0]+q[1]*axis[1] for q in second]
                if max(one)<min(two)-1e-9 or max(two)<min(one)-1e-9:return False
        return True
    overlaps=0;examples=[]
    for i,a in enumerate(poses):
        for j in range(i+3,len(poses)):
            b=poses[j]
            if a[4]!=b[4] or abs(a[5]-b[5])<2*profile.length_mm:continue
            if math.dist(a[:2],b[:2])>math.hypot(profile.length_mm,profile.width_mm):continue
            if overlap(polygons[i],polygons[j]):
                overlaps+=1
                if len(examples)<10:examples.append({'sample_pair':[i,j],'xy_a':a[:2],'xy_b':b[:2]})
    return {'status':'APPROXIMATE_NOMINAL_GEOMETRY','samples':len(poses),'sample_cap':max_samples,
            'nozzle_mm':[profile.length_mm,profile.width_mm],
            'nominal_envelope_xy':[min(q[0] for q in corners),max(q[0] for q in corners),
                                    min(q[1] for q in corners),max(q[1] for q in corners)],
            'sampled_centerline_crossings':crossings,'continuous_collision_certificate':False,
            'sampled_nonlocal_footprint_overlaps':overlaps,'overlap_examples':examples,
            'physical_collision_verified':False,'material_deposition_model':False,
            'limitations':'Solapes de huellas muestreadas, separados por al menos dos longitudes de boquilla de recorrido. Pueden ser normales en cierres. No detecta todos los solapes ni colisiones físicas.'}
