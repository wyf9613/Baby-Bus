"""Replay immutable executed prefixes; never re-solve or emit their labels."""
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
from .dataset import read_decisions
from .features import scene_features


def assert_equal(actual, expected, name):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(actual) != set(expected):
            raise ValueError('Recovery mapping mismatch: '+name)
        for key in expected:
            assert_equal(actual[key], expected[key], name+'.'+key)
    elif isinstance(expected, (list, tuple)):
        if len(actual) != len(expected):
            raise ValueError('Recovery length mismatch: '+name)
        for i, value in enumerate(expected):
            assert_equal(actual[i], value, name+':'+str(i))
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if not np.isclose(actual, expected, atol=1e-10, rtol=0):
            raise ValueError('Recovery numeric mismatch: '+name)
    elif actual != expected:
        raise ValueError('Recovery value mismatch: '+name)


class CollectionRecovery:
    def __init__(self, directories):
        self.completed=set(); self.prefixes=defaultdict(list); self.saved_groups=set()
        records=[]
        for directory in map(Path, directories):
            report=json.loads((directory/'decisions.collection.json').read_text(encoding='utf-8'))
            for scene in report['scenes']:
                if scene['episode_id'] in self.completed:
                    raise ValueError('Duplicate completed episode')
                self.completed.add(scene['episode_id'])
            for record in read_decisions(directory/'decisions.jsonl'):
                if record['group_id'] in self.saved_groups:
                    raise ValueError('Duplicate saved decision')
                self.saved_groups.add(record['group_id']); records.append(record)
        for record in records:
            if record['episode']['id'] not in self.completed:
                self.prefixes[record['episode']['id']].append(record)
        for identity, prefix in self.prefixes.items():
            prefix.sort(key=lambda row:row['episode']['segment'])
            if [row['episode']['segment'] for row in prefix] != list(range(len(prefix))):
                raise ValueError('Noncontiguous saved prefix: '+identity)

    def replay(self, environment, observation, choices, chooser, saved, previous_action, probability):
        """Restore sensors, dynamics history, native call order and chooser RNG."""
        episode=saved['episode']; successor=saved['successor']
        assert_equal(list(environment._backend.car._state_values()),episode['start_state'],'start_state')
        assert_equal(list(previous_action),episode['initial_control'],'previous_action')
        assert_equal(observation['timestamp_s'],saved['observation']['timestamp_s'],'timestamp')
        assert_equal(scene_features(observation),scene_features(saved['observation']),'observed_scene')
        previous=[entry['candidate'] for entry in saved['candidates']]
        assert_equal([c['candidate_id'] for c in choices],[c['candidate_id'] for c in previous],'candidate_ids')
        for current, old in zip(choices,previous):
            assert_equal(current['trajectory_xy_m'],old['trajectory_xy_m'],'candidate_geometry')
            assert_equal(current['reference']['speed_profile'],old['reference']['speed_profile'],'candidate_speed_rule')
        behavior='random' if probability==1. or chooser.random()<probability else 'planner'
        assert_equal(behavior,successor['behavior'],'behavior')
        candidates=previous if behavior=='random' else previous[:1]
        excluded=set(successor['solver_excluded_candidate_ids'])
        eligible=[c['candidate_id'] for c in candidates if c['candidate_id'] not in excluded]
        assert_equal(len(eligible),successor['eligible_candidates'],'eligible_count')
        if eligible:
            order=[eligible[int(i)] for i in chooser.permutation(len(eligible))]
            assert_equal([a['candidate_id'] for a in successor['attempts']],order[:len(successor['attempts'])],'attempt_order')
        if not successor['accepted']:
            return observation,tuple(previous_action)
        attempt=successor['attempts'][-1]
        assert_equal(attempt['candidate_id'],episode['selected_candidate_id'],'executed_candidate')
        for tick in attempt['trace']:
            observation,_,terminated,truncated,info=environment.step(tick['action'])
            if terminated or truncated:
                raise ValueError('Saved accepted execution terminated during recovery')
            raw=environment._backend.car._state_values()
            assert_equal(list(raw[:3]),[tick['x_m'],tick['y_m'],tick['body_heading_rad']],'executed_pose')
            assert_equal(observation['timestamp_s'],tick['timestamp_s'],'executed_time')
        assert_equal(list(environment._backend.car._state_values()),attempt['end_state'],'end_state')
        return observation,tuple(attempt['final_control'])
