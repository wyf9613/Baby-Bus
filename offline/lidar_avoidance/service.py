"""Snapshot adapter for the existing causal native planner service."""
from mvp21_policy.avoidance import filter_candidates
from offline.mvp21_sim.delivery import SimulatedPlannerWorker


class AvoidancePlannerWorker(SimulatedPlannerWorker):
    def __init__(self, guard, **kwargs):
        super().__init__(candidate_planner=filter_candidates, **kwargs)
        self.guard = guard

    def submit(self, job):
        job = dict(job, lidar_avoidance=self.guard.context())
        super().submit(job)
