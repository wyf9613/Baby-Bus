"""MVP2.1 transport; relocated from pinned f11428b, without algorithm tuning."""

from copy import deepcopy
import json
import queue
import subprocess
import sys
import threading
import time
from . import worker


class ShadowJobSource:
    """Pure job handoff shared by ROS and offline simulation."""
    def _shadow_offer(self):
        now = self._monotonic()
        if now-self.shadow_last_submit_s+1e-9 < 1/self.shadow_settings.rate_hz:
            return
        estimates = self.estimation_output
        if not estimates:
            return
        road = estimates.get("road") or {}
        forward = road.get("forward_geometry") or {}
        if not any(road.get(k) for k in ("centerline_xy", "left_boundary_xy", "right_boundary_xy")):
            road = forward
        keys = ("centerline_xy", "left_boundary_xy", "right_boundary_xy", "boundary_source",
                "lane_width_m", "lane_width_source", "inward_offset_m", "status", "visibility",
                "timestamp_s", "measurement_timestamp_s", "source_age_s", "time_aligned", "degraded")
        self.shadow_job_id += 1
        job = {"settings": vars(self.shadow_settings).copy(),
            "road": {key: deepcopy(road.get(key)) for key in keys},
            "state": {key: estimates.get("state", {}).get(key) for key in
                      ("speed_mps", "yaw_rate_rps", "timestamp_s", "speed_valid", "yaw_rate_valid")},
            "stamp_s": estimates.get("state", {}).get("timestamp_s"),
            "generation": self.shadow_generation, "job_id": self.shadow_job_id,
            "submitted_monotonic_s": now, "preview_m": self.planning_settings.preview_m,
            "handoff_estimate_s": self.shadow_handoff_estimate_s,
            "shadow_only": True}
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
