"""Fingerprint the actual module sources, separately from the upstream snapshot."""
import hashlib
from pathlib import Path


def source_fingerprint():
    root = Path(__file__).resolve().parent
    files = {path.name: hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest()
             for path in sorted(root.glob('*.py'))}
    aggregate = ''.join(name+'\0'+digest+'\n' for name, digest in files.items()).encode()
    return dict(format='mvp21_modules_sha256_v1', sha256=hashlib.sha256(aggregate).hexdigest(), files=files)
