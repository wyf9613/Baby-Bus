from pathlib import Path
from collections import deque
from dataclasses import replace
import json, math, unittest
import yaml
import test_mpc as m
import test_mpc_real_frames as frames
R=Path(__file__).resolve().parents[1]
P=yaml.safe_load((R/'config/ai4r_policy_speed_feedback.yaml').read_text(encoding='utf-8'))['/**/ai4r_policy']['ros__parameters']
TARGET=P['planning']['cruise_speed_mps']
def controller():return m.Controller(m.Settings(**P['mpc']),m.Vehicle(**P['vehicle']))
class SpeedFeedback(unittest.TestCase):
    def test_profile_and_explicit_overspeed_switch(self):
        self.assertEqual(TARGET,.25)
        self.assertEqual(P['mpc']['v_exec_max_mps'],.25)
        self.assertTrue(P['mpc']['speed_feedback_continuous_enabled'])
        self.assertFalse(P['mpc']['overspeed_stop_enabled'])
        self.assertTrue(m.Settings().overspeed_stop_enabled)
        with self.assertRaises(ValueError):m.Settings(overspeed_stop_enabled=1)
    def test_start_then_feedback_without_startup_lock(self):
        c=controller()
        for t in (0,.1,.3,.49,.5,1,2,5):
            c.clock_s=t;u=c.speed_feedback_drive(0,TARGET,0 if t==0 else .05)
            self.assertGreater(u,0);self.assertLessEqual(u,.36)
    def test_first_motion_sample_does_not_cut_drive(self):
        c=controller();c.speed_feedback_drive(0,TARGET,0);c.clock_s=.05
        u=c.speed_feedback_drive(.03,TARGET,.05)
        self.assertGreaterEqual(u,.30);self.assertFalse(c.feedback_coasting)
    def test_prediction_reduces_effort_without_full_coast(self):
        c=controller();c.feedback_started_s=0;c.clock_s=1;c.feedback_previous_speed=0
        u=c.speed_feedback_drive(.1,TARGET,.05)
        self.assertGreater(u,0);self.assertLess(u,.36);self.assertFalse(c.feedback_coasting)
    def test_integral_uses_actual_error(self):
        c=controller();c.feedback_started_s=0;c.clock_s=1;c.feedback_previous_speed=.05
        c.speed_feedback_drive(.1,TARGET,.05)
        self.assertGreater(c.feedback_integral,0)
    def test_crossing_old_coast_threshold_no_longer_cuts_or_latches(self):
        c=controller();c.feedback_started_s=0;c.clock_s=1
        c.feedback_previous_speed=TARGET;c.feedback_command=.30
        for speed in (TARGET+.01,TARGET,TARGET-.01,TARGET+.02,TARGET-.02):
            c.clock_s+=.05;u=c.speed_feedback_drive(speed,TARGET,.05)
            self.assertGreater(u,0);self.assertFalse(c.feedback_coasting)
        self.assertEqual(c.speed_feedback_drive(TARGET,0,.05),0)
    def test_legacy_mode_still_supports_its_explicit_coast_latch(self):
        c=m.Controller(replace(m.Settings(**P['mpc']),speed_feedback_continuous_enabled=False),m.Vehicle(**P['vehicle']))
        c.feedback_started_s=0;c.clock_s=1;c.feedback_previous_speed=TARGET+.02
        self.assertEqual(c.speed_feedback_drive(TARGET+.02,TARGET,.05),0)
        c.clock_s=1.05;self.assertEqual(c.speed_feedback_drive(TARGET,TARGET,.05),0)
        c.clock_s=1.1;self.assertGreater(c.speed_feedback_drive(TARGET-.03,TARGET,.05),0)
    def test_slew_bounds_regular_adjustments_but_stop_is_immediate(self):
        c=controller();c.feedback_started_s=0;c.clock_s=1;c.feedback_command=.30
        c.feedback_previous_speed=TARGET
        for speed in (TARGET+.05,TARGET+.1,TARGET-.05,TARGET+.2):
            previous=c.feedback_command;c.clock_s+=.05
            u=c.speed_feedback_drive(speed,TARGET,.05)
            self.assertLessEqual(abs(u-previous),P['mpc']['speed_feedback_drive_slew_per_s']*.05+1e-9)
        self.assertEqual(c.speed_feedback_drive(TARGET,0,.05),0)
    def test_persistent_excess_speed_can_still_reduce_drive_to_zero(self):
        c=controller();c.feedback_started_s=0;c.feedback_previous_speed=.6;c.feedback_command=.30
        for i in range(80):
            c.clock_s=1+i*.05;u=c.speed_feedback_drive(.6,TARGET,.05)
        self.assertEqual(u,0)

    def test_loss_of_motion_still_stops(self):
        c=controller();c.feedback_started_s=0;c.feedback_moving=True
        c.clock_s=1;c.speed_feedback_drive(0,TARGET,.05);c.clock_s=3.01
        with self.assertRaisesRegex(m.Stop,'motion_lost'):c.speed_feedback_drive(0,TARGET,.05)
    def test_overspeed_lock_is_configurable(self):
        ref=m.adapt(frames.ordered_centerline_reference(3.,speed=TARGET))
        c=controller()
        try:c.step(ref,P["mpc"]["v_exec_max_mps"]+P["mpc"]["overspeed_margin_mps"]+.01,0.,10.,True)
        except m.Stop:pass
        self.assertNotEqual(c.last_debug['reject_reason'],'overspeed')
        c=m.Controller(replace(m.Settings(**P['mpc']),overspeed_stop_enabled=True),m.Vehicle(**P['vehicle']))
        with self.assertRaises(m.Stop):c.step(ref,P["mpc"]["v_exec_max_mps"]+P["mpc"]["overspeed_margin_mps"]+.01,0.,10.,True)
        self.assertEqual(c.last_debug['reject_reason'],'overspeed')
    def test_history_matches_applied_feedback(self):
        c=controller();ref=m.adapt(frames.ordered_centerline_reference(3.,speed=TARGET))
        for i in range(12):
            ref['timestamp_s']=10+i*.05
            result=c.step(ref,0.,0. if i==0 else .05,10+i*.05,i==0)
            self.assertGreater(result['drive'],0);self.assertLessEqual(result['drive'],.36)
            self.assertAlmostEqual(c.history[-1][1],result['drive'])
    def test_delayed_plants_keep_moving_with_valid_reference_assumed(self):
        results=[]
        for plant in ('linear','stiction'):
            for gain,delay in ((1.,.26),(.8,.32),(1.2,.2)):
                c=controller();q=deque();speed=0.;trace=[];drives=[]
                for i in range(1200):
                    t=i*.05;c.clock_s=t
                    u=c.speed_feedback_drive(speed,TARGET,0 if i==0 else .05)
                    self.assertGreaterEqual(u,0);self.assertLessEqual(u,.36)
                    q.append((t,u));applied=0
                    for at,v in q:
                        if at<=t-delay:applied=v
                        else:break
                    while len(q)>1 and q[1][0]<=t-delay:q.popleft()
                    if plant=='linear':speed=max(0,speed+.05*(3.725*gain*applied-speed)/1.33)
                    else:
                        desired=2*gain*max(0,applied-.28) if speed>=.005 or applied>=.33 else 0
                        speed=max(0,speed+.05*(desired-speed)/.8)
                    trace.append(speed);drives.append(u)
                self.assertGreater(min(trace[100:]),.02)
                self.assertTrue(all(u>0 for u in drives[-400:]),"steady feedback must not cycle into zero drive in these assumed plants")
                results.append(dict(plant=plant,gain=gain,delay_s=delay,peak_mps=max(trace),tail_min_mps=min(trace[-400:]),tail_rmse_mps=math.sqrt(sum((v-TARGET)**2 for v in trace[-400:])/400),zero_drive_fraction=sum(u==0 for u in drives)/len(drives)))
        out=R/'.verification/smooth-speed-feedback-study.json';out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(results,indent=2),encoding='utf-8')
if __name__=='__main__':unittest.main()
