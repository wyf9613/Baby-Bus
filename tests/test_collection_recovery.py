from copy import deepcopy
import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from offline.lattice_training.collection_recovery import CollectionRecovery
from offline.lattice_training.pipeline import PipelineSettings,collect_records,collection_episode_id
from offline.lattice_training.objective import CStarTrackingSettings
from offline.lattice_training.cstar import SolverSettings
from tests.test_lattice_training import decision_records


class RecoveryChecks(unittest.TestCase):
    def test_completed_episodes_are_not_reinitialized_or_emitted(self):
        cfg=PipelineSettings(kinds=('straight',),train_seeds=(0,),validation_seeds=(),test_seeds=(),
            train_only=True,starts_per_road=1,planner_profiles=('standard',))
        identity=collection_episode_id('straight',0,0,'standard',cfg)
        def forbidden(*args,**kwargs):raise AssertionError('Completed episode initialized again')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'new.jsonl'
            report=collect_records(path,cfg,CStarTrackingSettings(),SolverSettings(),
                environment_factory=forbidden,recovery=SimpleNamespace(completed={identity},prefixes={}))
            self.assertEqual(path.read_bytes(),b'')
            self.assertEqual(report['scenes'],[])
            self.assertEqual(report['decisions'],0)

    def fixture(self,directory,rows):
        directory=Path(directory)
        (directory/'decisions.collection.json').write_text(json.dumps(dict(scenes=[])),encoding='utf-8')
        (directory/'decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')

    def test_prefix_with_missing_first_decision_is_rejected(self):
        row=deepcopy(decision_records()[0]);row['episode']=dict(id='interrupted',segment=1)
        with tempfile.TemporaryDirectory() as directory:
            self.fixture(directory,[row])
            with self.assertRaisesRegex(ValueError,'Noncontiguous'):CollectionRecovery([directory])

    def test_duplicate_saved_rows_are_rejected(self):
        row=deepcopy(decision_records()[0]);row['episode']=dict(id='interrupted',segment=0)
        with tempfile.TemporaryDirectory() as directory:
            self.fixture(directory,[row,row])
            with self.assertRaisesRegex(ValueError,'Duplicate saved decision'):CollectionRecovery([directory])


if __name__=='__main__':unittest.main()
