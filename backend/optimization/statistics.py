"""Descriptive computational statistics; physical measurements stay absent."""
import math
import statistics as stats


def reduction(base,new):
    return 100*(base-new)/base if base and base>0 and new is not None else None


def wilson(successes,total,z=1.6448536269514722):
    if type(total) is not int or total<=0 or not 0<=successes<=total: return None
    p=successes/total; den=1+z*z/total
    center=(p+z*z/(2*total))/den
    margin=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/den
    return [max(0.,center-margin),min(1.,center+margin)]


def paired_summary(differences,*,representative=False):
    values=[float(x) for x in differences if x is not None]
    n=len(values)
    if not n: return {'n_families':0,'mean':None,'median':None,'sd':None,'t_score':None,'t_ic90':None}
    mean=stats.mean(values); sd=stats.stdev(values) if n>1 else None
    result={'n_families':n,'mean':mean,'median':stats.median(values),'sd':sd,
            't_score':None,'t_ic90':None,'inference':'DESCRIPTIVE_PILOT',
            'reason':'Pocas familias elegidas por conveniencia; no se acredita normalidad ni población universal'}
    if representative and n>=8 and sd and sd>0:
        from scipy.stats import t
        se=sd/math.sqrt(n); margin=float(t.ppf(.95,n-1))*se
        result.update(t_score=mean/se,t_ic90=[mean-margin,mean+margin],inference='EXPLORATORY_PAIRED_T')
    if n>1 and sd==0: result['reason']='Diferencias idénticas; t no aplicable'
    return result
