"""Fingerprint a working-tree snapshot, including local evidence, without Git mutations."""
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from zipfile import ZipFile,ZIP_DEFLATED

from .validation_workspace import ROOT,write


def package(output):
    output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    excluded={'.git','.verification','.codex','__pycache__','.pytest_cache','build','node_modules','.venv','venv'}
    paths=[]
    for parent,dirs,names in os.walk(ROOT):
        dirs[:]=[name for name in dirs if name not in excluded]
        for name in names:
            path=Path(parent)/name
            if path!=output and name not in excluded and name!='.env' and not path.is_symlink() and not name.endswith(('.pyc','.pyo','.tmp')):paths.append(path)
    def git(*args):return subprocess.check_output(['git','-c','safe.directory='+ROOT.as_posix(),'-C',str(ROOT),*args],text=True,encoding='utf-8').strip()
    manifest=dict(format='validation_working_tree_bundle_v1',created_utc=datetime.now(timezone.utc).isoformat(),
        head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),working_tree_included=True,
        code_and_evidence_included=True,environment_included=False,committed_or_pushed=False,files={})
    with ZipFile(output,'w',compression=ZIP_DEFLATED,compresslevel=4) as archive:
        for path in sorted(paths):
            data=path.read_bytes();name=path.relative_to(ROOT).as_posix()
            manifest['files'][name]=hashlib.sha256(data).hexdigest();archive.writestr(name,data)
        archive.writestr('BUNDLE_MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2))
        archive.writestr('WORKING_TREE.diff',git('diff','HEAD','--binary'))
    with ZipFile(output) as archive:
        if archive.testzip() is not None:raise AssertionError('Bundle CRC verification failed')
    result=dict(path=str(output),bytes=output.stat().st_size,sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        files=len(paths),all_crc_verified=True)
    write(output.with_suffix('.json'),result);return result
