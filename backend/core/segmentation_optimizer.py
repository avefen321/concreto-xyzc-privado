"""Bounded DAG segmentation over one frozen reference, without AI dependencies."""
from dataclasses import dataclass, replace
import math
from .arc_fitting import tangent_arc
from .orientation import wrap180


@dataclass(frozen=True)
class SearchParameters:
    candidate_window: int = 4
    split_level: int = 1
    max_sweep_deg: float = 160.

    def to_dict(self):
        return {'candidate_window':self.candidate_window,'split_level':self.split_level,
                'max_sweep_deg':self.max_sweep_deg}

    @classmethod
    def from_dict(cls,value=None):
        p=cls(**(value or {}))
        if type(p.candidate_window) is not int or not 1<=p.candidate_window<=16:
            raise ValueError('candidate_window debe ser entero entre 1 y 16')
        if type(p.split_level) is not int or not 0<=p.split_level<=2:
            raise ValueError('split_level debe ser entero entre 0 y 2')
        if type(p.max_sweep_deg) not in (int,float) or not math.isfinite(p.max_sweep_deg) or not 30<=p.max_sweep_deg<=180:
            raise ValueError('max_sweep_deg debe estar entre 30 y 180')
        return p


@dataclass(frozen=True)
class CandidatePrimitive:
    start: int
    end: int
    points: tuple
    cost: int
    kind: str


def shortest_path(node_count,edges,compatible=None):
    """All edge endpoints share fixed XYZ and reference heading: no hidden DP state.

    Arcs are endpoint-tangent-constrained and lines pass the same angular gate.
    This DP minimizes blocks only, not curvature continuity or motion time.
    """
    incoming={}
    for edge in edges:
        if not 0<=edge.start<edge.end<node_count or edge.cost<=0:
            raise ValueError('Arista DAG inválida')
        incoming.setdefault(edge.end,[]).append(edge)
    # Keep the arriving edge as state when join orientation is constrained.
    # A biarc's interior tangent need not equal the reference tangent exactly.
    distance={}; previous={}; arrivals={}
    for j in range(1,node_count):
        for edge in incoming.get(j,[]):
            key=id(edge)
            options=[(0,None)] if edge.start==0 else [
                (distance[id(prior)],prior) for prior in arrivals.get(edge.start,[])
                if compatible is None or compatible(prior,edge)]
            if not options: continue
            cost,prior=min(options,key=lambda item:item[0])
            distance[key]=cost+edge.cost; previous[key]=prior
            arrivals.setdefault(j,[]).append(edge)
    if not arrivals.get(node_count-1): return None
    edge=min(arrivals[node_count-1],key=lambda item:distance[id(item)])
    chain=[]
    while edge is not None:
        chain.append(edge); edge=previous[id(edge)]
    return list(reversed(chain))


def optimize_interval(fitter,a,b,values,stats,cancel):
    from .curve_reconstruction import ReconstructionFitter
    p=SearchParameters.from_dict(values)
    # The original path is an admissible set of fallback edges. Recomputed on
    # the SAME frozen curve, not on a previously simplified output.
    baseline=fitter.fit(a,b)
    stats['regions']+=1
    if len(baseline)<=2: return baseline
    nodes={a,b}
    nodes.update(q.fit_metadata['reference_u_end'] for q in baseline[1:])
    if p.split_level:
        knots=[a]+[u for u in fitter.ref.u if a<u<b]+[b]
        for left,right in zip(knots,knots[1:]):
            for k in range(1,2**p.split_level):
                nodes.add(left+(right-left)*k/2**p.split_level)
    nodes=sorted(nodes)
    # Bounded cost and explicit fallback; no unbounded all-pairs graph.
    if len(nodes)>256:
        stats['fallback_regions']+=1
        stats.setdefault('fallback_reasons',{}).setdefault('NODE_LIMIT',0)
        stats['fallback_reasons']['NODE_LIMIT']+=1
        return baseline
    cfg=replace(fitter.config,arc_fit=replace(fitter.config.arc_fit,max_sweep_deg=p.max_sweep_deg))
    proposal=ReconstructionFitter(fitter.ref,cfg)
    edges=[]; index={u:i for i,u in enumerate(nodes)}
    for first,last in zip(baseline,baseline[1:]):
        lo=last.fit_metadata['reference_u_start']; hi=last.fit_metadata['reference_u_end']
        edges.append(CandidatePrimitive(index[lo],index[hi],(first,last),1,'BASELINE'))
    for i,left in enumerate(nodes[:-1]):
        for j in range(i+1,min(len(nodes),i+p.candidate_window+1)):
            if cancel(): raise InterruptedError('Búsqueda cancelada')
            right=nodes[j]; stats['edges_attempted']+=1
            before=stats['edges_accepted']; proposal.last_reject=None
            line=proposal.line_candidate(left,right)
            choices=[]
            if line is not None:
                choices.append((proposal.materialize(left,right,line=line),'G1'))
            h=math.radians(proposal.ref.heading(left))
            arc=tangent_arc(proposal.ref.xyz(left),proposal.ref.xyz(right),(math.cos(h),math.sin(h)))
            if arc is not None:
                arc.sa,arc.sb=left,right
                if (abs(wrap180(arc.heading+math.degrees(arc.sweep)-proposal.ref.heading(right)))<1e-9
                        and proposal.validate_arc(arc)[0]):
                    choices.append((proposal.materialize(left,right,arcs=[arc]),'ARC'))
            if not choices and cfg.arc_fit.biarc_enabled:
                arcs=proposal.on_curve_biarcs(left,right)
                if arcs: choices.append((proposal.materialize(left,right,arcs=arcs),'BIARC'))
            for points,kind in choices:
                # G1 with angular sampling could cost more than one block.
                # Require the existing sampler's endpoint-angle limit here.
                if any(abs(q.heading_override_deg-r.heading_override_deg)>cfg.sampling.max_delta_c_deg+1e-8
                       for r,q in zip(points,points[1:]) if q.arc_center is None): continue
                stats['edges_accepted']+=1
                edges.append(CandidatePrimitive(i,j,tuple(points),len(points)-1,kind))
            if stats['edges_accepted']==before:
                reason=proposal.last_reject or 'NO_ADMISSIBLE_GEOMETRY_OR_ANGLE_STEP'
                reasons=stats.setdefault('rejected_by_reason',{})
                reasons[reason]=reasons.get(reason,0)+1
                examples=stats.setdefault('rejection_examples',[])
                if len(examples)<30: examples.append({'u_start':left,'u_end':right,'reason':reason})
    def compatible(left,right):
        return abs(wrap180(left.points[-1].heading_override_deg-right.points[0].heading_override_deg))<1e-6
    chain=shortest_path(len(nodes),edges,compatible)
    if chain is None: return baseline
    output=[]
    for edge in chain:
        if output:
            output[-1]=replace(output[-1],heading_override_deg=edge.points[0].heading_override_deg)
            output.extend(edge.points[1:])
        else: output.extend(edge.points)
    # Keep tie cases deterministic and retain the established fallback.
    return output if len(output)<len(baseline) else baseline
