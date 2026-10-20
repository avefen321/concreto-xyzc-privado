"""Immutable stage records around the established converter, not a second pipeline."""
from dataclasses import dataclass
import hashlib
import json

BACKEND_VERSION='3.6.0'


def digest(value):
    encoded=value.encode('utf-8') if isinstance(value,str) else json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class Stage:
    id: str
    name: str
    input_hash: str
    output_hash: str
    count: int


class StageTrace:
    names=('RawProgram','ParsedProgram','CanonicalMotionProgram','ClassifiedGeometry',
           'PrintableGeometry','OrientationPlan','SampledMotionPlan','ProcessPlan','SerializedProgram')

    def __init__(self): self.stages=[]

    def add(self,data,count):
        from dataclasses import asdict
        index=len(self.stages)
        record=Stage(f'S{index}',self.names[index],self.stages[-1]['output_hash'] if index else '',digest(data),count)
        self.stages.append(asdict(record))


@dataclass(frozen=True)
class FinalMotionPlan:
    conversion_id: str
    samples: tuple
    design_path: tuple
    printable_path: tuple
    primitives: tuple = ()
