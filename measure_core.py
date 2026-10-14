import ctypes as C
import json
from pathlib import Path
import platform
import statistics
from time import perf_counter_ns
from branchguard.native import Controller, State, Observation, Output
from branchguard.simulator import Course, render


if __name__=='__main__':
    ctl=Controller()
    frame=render(Course('fork_left',101),'mixed',(1.7,0.,0.),0.,0)
    for _ in range(100):
        ctl.step(frame,route=1)
    timings=[]
    for _ in range(3000):
        t=perf_counter_ns()
        ctl.step(frame,route=1)
        timings.append((perf_counter_ns()-t)/1000.)
    report={'platform':platform.platform(),'processor':platform.processor(),
            'samples':len(timings),'median_us':statistics.median(timings),
            'p95_us':sorted(timings)[int(.95*len(timings))],
            'state_bytes':C.sizeof(State),'observation_bytes':C.sizeof(Observation),
            'output_bytes':C.sizeof(Output),
            'beam_arrays_bytes':2*24*60,
            'scope':'Windows host, Python validation/ctypes + C image extraction and decision; one fixed fork frame',
            'not_measured':'RT1064 runtime, total compiler stack usage, real-time worst case'}
    dst=Path(__file__).resolve().parent/'results'/'host_timing.json'
    dst.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
