"""MVP2.1 transport; relocated from pinned f11428b, without algorithm tuning."""

from copy import deepcopy
import json
import math
from .geometry import finite_number
import queue
import subprocess
import sys
import threading
import time
from . import worker


class ShadowJobSource:
    """Pure temporal collection and job handoff shared by ROS and simulation."""
    def _shadow_collect_road(self, road, stamp):
        """Bounded 20 Hz collection; planner execution also calls before control.

        Accumulate short read-only motion intervals; no per-frame fitting and no
        changes to the estimator's motion history. Alignment faults clear only
        shadow memory. Original source epochs are not renewed by reconstruction.
        """
        previous = getattr(self, "shadow_pose_stamp_s", None)
        pose = getattr(self, "shadow_pose", [0.0, 0.0, 0.0])
        epoch = getattr(self, "shadow_odometry_epoch", 0)
        history = getattr(self, "shadow_road_history", [])
        reason, reset = ("bootstrap" if previous is None else "aligned"), False
        if previous is not None and stamp != previous:
            try:
                motion, motion_reason = self.motion_history.pose_between(previous, stamp)
            except Exception as error:
                motion, motion_reason = None, "motion_exception: "+str(error)
            if motion is None:
                history, pose, epoch = [], [0.0, 0.0, 0.0], epoch+1
                self.shadow_previous_reference = None
                reason, reset = motion_reason, True
                self.shadow_collection_reset_id = getattr(self, "shadow_collection_reset_id", 0)+1
                events = getattr(self, "shadow_collection_events", [])
                events.append({"id": self.shadow_collection_reset_id, "stamp_s": stamp,
                               "reason": motion_reason, "odometry_epoch": epoch})
                self.shadow_collection_events = events[-64:]
                if hasattr(self, "_warn"):
                    self._warn("shadow_alignment", "Shadow temporal history reset: "+str(motion_reason))
            else:
                c, s = math.cos(pose[2]), math.sin(pose[2])
                pose = [pose[0]+c*motion["dx_m"]-s*motion["dy_m"],
                        pose[1]+s*motion["dx_m"]+c*motion["dy_m"], pose[2]+motion["yaw_rad"]]
        measurement = road.get("measurement_timestamp_s")
        key = measurement if finite_number(measurement) else stamp
        history = [h for h in history if stamp-h["stamp_s"] <= self.shadow_settings.fusion_window_s
                   and h.get("measurement_key") != key]
        keys = ("centerline_xy", "left_boundary_xy", "right_boundary_xy", "boundary_source",
                "lane_width_m", "lane_width_source", "inward_offset_m", "status", "visibility",
                "timestamp_s", "measurement_timestamp_s", "source_age_s", "time_aligned", "degraded", "frame_id")
        snapshot = {"road": {key: deepcopy(road.get(key)) for key in keys}, "stamp_s": stamp,
                    "pose": list(pose), "odometry_epoch": epoch, "measurement_key": key}
        state = (getattr(self, "estimation_output", None) or {}).get("state") or {}
        snapshot["state"] = {key: state.get(key) for key in
            ("timestamp_s", "frame_id", "speed_mps", "yaw_rate_rps", "speed_valid", "yaw_rate_valid")}
        history.append(snapshot)
        self.shadow_road_history = history[-self.shadow_settings.fusion_max_samples:]
        self.shadow_pose, self.shadow_pose_stamp_s, self.shadow_odometry_epoch = pose, stamp, epoch
        self.shadow_collection_diagnostics = {"status": "RESET" if reset else "BOOTSTRAP" if previous is None else "ALIGNED",
            "reason": reason, "reset": reset, "buffer_samples": len(self.shadow_road_history), "odometry_epoch": epoch,
            "events": deepcopy(getattr(self, "shadow_collection_events", [])),
            "reset_events_total": getattr(self, "shadow_collection_reset_id", 0)}
        return snapshot["road"]

    def _shadow_offer(self):
        now = self._monotonic()
        estimates = self.estimation_output
        if not estimates:
            return
        road = estimates.get("road") or {}
        forward = road.get("forward_geometry") or {}
        if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
            road = forward
        stamp = estimates.get("state", {}).get("timestamp_s")
        stamp = stamp if finite_number(stamp) else now
        if self.shadow_settings.fusion_enabled or getattr(getattr(self, "planner_control_settings", None), "enabled", False):
            road = self._shadow_collect_road(road, stamp)
        if now-self.shadow_last_submit_s+1e-9 < 1/self.shadow_settings.rate_hz:
            return
        self.shadow_job_id += 1
        job = {"settings": vars(self.shadow_settings).copy(),
            "road": deepcopy(road),
            "state": {key: estimates.get("state", {}).get(key) for key in
                      ("speed_mps", "yaw_rate_rps", "timestamp_s", "speed_valid", "yaw_rate_valid")},
            "stamp_s": stamp,
            "generation": self.shadow_generation, "job_id": self.shadow_job_id,
            "submitted_monotonic_s": now, "preview_m": self.planning_settings.preview_m,
            "handoff_estimate_s": self.shadow_handoff_estimate_s,
            "shadow_pose": list(getattr(self, "shadow_pose", [0.0, 0.0, 0.0])),
            "shadow_odometry_epoch": getattr(self, "shadow_odometry_epoch", 0),
            "planner_control_enabled": bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False)),
            "shadow_only": not bool(getattr(getattr(self, "planner_control_settings", None), "enabled", False))}
        if self.shadow_settings.fusion_enabled:
            job.update(road_history=deepcopy(getattr(self, "shadow_road_history", [])),
                shadow_pose=list(getattr(self, "shadow_pose", [0.0, 0.0, 0.0])),
                shadow_odometry_epoch=getattr(self, "shadow_odometry_epoch", 0),
                temporal_collection=getattr(self, "shadow_collection_diagnostics", None),
                previous_reference=deepcopy(getattr(self, "shadow_previous_reference", None)))
        guard = getattr(self, 'lidar_avoidance_guard', None)
        if guard is not None:
            job['lidar_avoidance'] = guard.context()
        self.shadow_worker.submit(job)
        self.shadow_last_submit_s = now


class ShadowWorker:
    """Latest-only transport thread and separate CPU process; never block control."""
    def __init__(self):
        self.source_file = worker.__file__
        self.jobs, self.results = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self.closed = threading.Event()
        self.process = None
        self.thread = threading.Thread(target=self._serve, name="shadow-transport", daemon=True)
        self.thread.start()

    @staticmethod
    def _replace(mailbox, value):
        try:
            mailbox.get_nowait()
        except queue.Empty:
            pass
        try:
            mailbox.put_nowait(value)
        except queue.Full:
            pass

    def submit(self, job):
        if not self.closed.is_set():
            self._replace(self.jobs, job)

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def _serve(self):
        try:
            process = subprocess.Popen([sys.executable, self.source_file, "--shadow-worker"],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, encoding="utf-8", bufsize=1,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            self.process = process
            if self.closed.is_set():
                process.terminate()
                return
            while not self.closed.is_set():
                try:
                    job = self.jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                process.stdin.write(json.dumps(job, allow_nan=False)+"\n")
                process.stdin.flush()
                line = process.stdout.readline()
                if not line:
                    raise RuntimeError("shadow process exited")
                result = json.loads(line)
                result["wire_json"] = line.rstrip()
                result["received_monotonic_s"] = time.monotonic()
                result["submitted_monotonic_s"] = job["submitted_monotonic_s"]
                self._replace(self.results, result)
        except Exception as error:
            self._replace(self.results, {"status": "TRANSPORT_ERROR", "error": str(error)})
        finally:
            if self.process is not None:
                if self.process.poll() is None:
                    self.process.terminate()
                try:
                    self.process.communicate(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.communicate()

    def close(self):
        self.closed.set()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
