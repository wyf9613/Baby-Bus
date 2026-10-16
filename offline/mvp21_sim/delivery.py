"""Causal discrete-event planner service; simulated time advances during delivery.

CPU work runs synchronously, but its result is hidden until the modelled service
completion. This tests delayed frames without claiming executor real-time proof.
"""
from copy import deepcopy
import math
import time

from mvp21_policy.lattice import ShadowLatticePlanner, ShadowSettings


class SimulatedPlannerWorker:
    def __init__(self, *, delay_s=.1, measured=False):
        if not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError('Nonnegative delivery delay required')
        self.delay_s, self.measured = delay_s, measured
        self.now_s = 0.
        self.pending = self.waiting = None
        self.completed = []

    def submit(self, job):
        job = deepcopy(job)
        if self.pending is None:
            self._start(job,self.now_s)
        else:
            self.waiting = job  # Same latest-only waiting slot as ShadowWorker.

    def _start(self, job, start_s):
        started = time.perf_counter()
        try:
            result = ShadowLatticePlanner(ShadowSettings(**job['settings'])).plan(job)
        except Exception as error:
            result = dict(status='WORKER_ERROR',error=str(error),selected=None)
        wall = time.perf_counter()-started
        service = wall+self.delay_s if self.measured else self.delay_s
        result.update(generation=job['generation'],job_id=job['job_id'],
            execution_anchor=dict(pose=job.get('shadow_pose'),odometry_epoch=job.get('shadow_odometry_epoch')),
            shadow_only=not bool(job.get('planner_control_enabled')),
            submitted_monotonic_s=job['submitted_monotonic_s'])
        self.pending = dict(result=result,job=job,ready_s=start_s+service,
            started_s=start_s,service_s=service,planner_wall_s=wall)

    def poll(self, now_s):
        self.now_s = now_s
        if self.pending is None or now_s+1e-9 < self.pending['ready_s']:
            return None
        packet, self.pending = self.pending, None
        packet['delivered_s'] = now_s
        packet['round_trip_s'] = now_s-packet['job']['submitted_monotonic_s']
        self.completed.append(packet)
        if self.waiting is not None:
            job,self.waiting = self.waiting,None
            self._start(job,packet['ready_s'])
        return packet
