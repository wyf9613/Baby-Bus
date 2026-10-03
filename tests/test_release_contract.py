"""Release identity and source-package regressions using isolated Git fixtures."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
import check_history as history  # noqa: E402
import package_source as bundle  # noqa: E402


class ReleaseContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'repo'
        self.root.mkdir()
        self.git('init', '-b', 'main')
        self.git('config', 'user.name', 'Release fixture')
        self.git('config', 'user.email', 'fixture@example.invalid')
        self.git('config', 'commit.gpgsign', 'false')
        self.git('config', 'tag.gpgsign', 'false')
        for name in bundle.REQUIRED:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture\n', encoding='utf-8')
        (self.root / 'package.xml').write_text(
            '<package><name>fixture</name><version>0.1.0</version></package>\n',
            encoding='utf-8')
        (self.root / 'ci/dependencies.repos').write_text(
            'repositories:\n  dream_interfaces:\n    type: git\n'
            '    url: https://example.invalid/dream_interfaces.git\n'
            '    version: 9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd\n',
            encoding='utf-8')
        self.git('add', '.')
        self.git('commit', '-m', 'bootstrap')
        self.git('checkout', '-b', 'dev')
        (self.root / 'payload').write_text('candidate\n', encoding='utf-8')
        self.git('add', '.')
        self.git('commit', '-m', 'candidate')
        self.candidate = self.git('rev-parse', 'HEAD')
        self.git('update-ref', 'refs/remotes/origin/dev', self.candidate)
        self.git('checkout', 'main')
        self.git('merge', '--no-ff', 'dev', '-m', 'release: v0.1.0')
        self.promotion = self.git('rev-parse', 'HEAD')
        self.git('update-ref', 'refs/remotes/origin/main', self.promotion)

    def git(self, *args):
        return subprocess.check_output(
            ['git', '-C', str(self.root), *args], text=True,
            stderr=subprocess.PIPE).strip()

    def tag(self, version='v0.1.0', annotated=True):
        args = ['tag']
        if annotated:
            args += ['-a', '-m', 'fixture release']
        self.git(*args, version)
        return {'CI_COMMIT_TAG': version}

    def test_final_tag_exact_tree_and_reproducible_dependency_archive(self):
        environment = self.tag()
        result = history.check(self.root, environment)
        self.assertEqual(result['candidate_commit'], self.candidate)
        self.assertEqual(self.git('rev-parse', 'HEAD^{tree}'),
                         self.git('rev-parse', self.candidate + '^{tree}'))
        first = Path(self.temp.name) / 'first'
        second = Path(self.temp.name) / 'second'
        bundle.package(self.root, first, environment)
        bundle.package(self.root, second, environment)
        self.assertEqual({p.name: p.read_bytes() for p in first.iterdir()},
                         {p.name: p.read_bytes() for p in second.iterdir()})
        release = json.loads((first / 'release.json').read_text())
        dependency = release['dependencies']['dream_interfaces']
        self.assertNotIn('release_tag', dependency)
        self.assertEqual(dependency['commit'],
                         '9b6ef917c0b8bc31efe6ca07b8a3d25f29c35fdd')

    def test_rejects_wrong_tag_identity_and_changed_promotion_tree(self):
        for version, annotated in [('v0.1.0', False), ('v0.2.0', True)]:
            with self.subTest(version=version):
                with self.assertRaises(ValueError):
                    history.check(self.root, self.tag(version, annotated))
        self.git('tag', '-d', 'v0.2.0')
        (self.root / 'payload').write_text('changed promotion\n',
                                           encoding='utf-8')
        self.git('add', '.')
        self.git('commit', '-m', 'direct main change')
        with self.assertRaises(ValueError):
            history.check(self.root, {'CI_COMMIT_BRANCH': 'main'})

    def test_candidate_tag_requires_frozen_dev(self):
        self.git('checkout', 'dev')
        self.git('update-ref', 'refs/remotes/origin/main', 'main^1')
        environment = self.tag('v0.1.0-rc.1')
        result = history.check(self.root, environment)
        self.assertEqual(result['kind'], 'candidate')
        (self.root / 'payload').write_text('new candidate\n', encoding='utf-8')
        self.git('add', '.')
        self.git('commit', '-m', 'unfrozen candidate')
        self.tag('v0.1.0-rc.2')
        with self.assertRaises(ValueError):
            history.check(self.root, {'CI_COMMIT_TAG': 'v0.1.0-rc.2'})

        self.git('update-ref', 'refs/remotes/origin/dev', 'HEAD')
        self.git('update-ref', 'refs/remotes/origin/main', self.promotion)
        with self.assertRaises(subprocess.CalledProcessError):
            history.check(self.root, {'CI_COMMIT_TAG': 'v0.1.0-rc.2'})

    def test_sync_requires_ancestry_and_equal_tree(self):
        environment = {'CI_MERGE_REQUEST_SOURCE_BRANCH_NAME': 'main',
                       'CI_MERGE_REQUEST_TARGET_BRANCH_NAME': 'dev'}
        history.check(self.root, environment)
        (self.root / 'payload').write_text('reconciliation\n', encoding='utf-8')
        self.git('add', '.')
        self.git('commit', '-m', 'changed sync')
        with self.assertRaises(ValueError):
            history.check(self.root, environment)


if __name__ == '__main__':
    unittest.main()
