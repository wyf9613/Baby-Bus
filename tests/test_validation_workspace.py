import json
from pathlib import Path
import tempfile
import unittest

from offline.mvp21_sim.validation_workspace import initialize,ingest,roads,run,index,write,read
from offline.lattice_training.scenarios import expanded_road_spec


class ValidationWorkspaceTests(unittest.TestCase):
    def test_add_raw_road_run_and_index_immutable_records(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'workspace';initialize(root,seed_roads=False,import_history=False)
            source=Path(tmp)/'road.json';write(source,expanded_road_spec('suite_straight',710))
            path=ingest(root,source);fixture=read(path)
            records=run(root,selected=[fixture['fixture_id']],max_steps=4)
            first=index(root);self.assertEqual(len(first['cases']),1)
            payload=root/first['cases'][0]['data_url'].lstrip('/');before=payload.read_bytes()
            run(root,selected=[fixture['fixture_id']],max_steps=4)
            second=index(root)
            self.assertEqual(len(second['cases']),1)
            self.assertEqual(second['history_count'],2)
            self.assertNotEqual(first['cases'][0]['data_url'],second['cases'][0]['data_url'])
            self.assertEqual(before,payload.read_bytes())
            self.assertEqual(second['latest_case_name'],second['cases'][0]['name'])
            self.assertTrue(second['cases'][0]['current_source'])

    def test_invalid_input_is_indexed_without_fabricating_trajectory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);initialize(root,seed_roads=False,import_history=False)
            (root/'roads/broken.json').write_text('{')
            result=index(root)
            self.assertEqual(result['cases'],[])
            self.assertEqual(len(result['errors']),1)
            self.assertFalse(list((root/'runs').rglob('case.json')))


if __name__=='__main__':unittest.main()
