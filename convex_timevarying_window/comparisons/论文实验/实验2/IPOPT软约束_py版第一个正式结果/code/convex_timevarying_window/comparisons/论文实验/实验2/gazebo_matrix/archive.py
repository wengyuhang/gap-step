"""Lossless result-local native-log storage with raw-content hashes."""
from contextlib import contextmanager
import gzip
import hashlib
import json
from pathlib import Path
import shutil


def stream_digest(stream):
    value = hashlib.sha256()
    for block in iter(lambda: stream.read(1024*1024), b''):
        value.update(block)
    return value.hexdigest()


def pack(folder):
    folder = Path(folder)
    files = [*folder.glob('*.pbtxt'), *folder.glob('*.ulg')]
    records = []
    for path in files:
        archive = path.with_suffix(path.suffix+'.gz')
        if archive.exists():
            raise FileExistsError(archive)
        with path.open('rb') as source:
            raw_digest = stream_digest(source)
        with path.open('rb') as source, archive.open('wb') as output:
            with gzip.GzipFile(filename='', mode='wb', fileobj=output, mtime=0) as target:
                shutil.copyfileobj(source, target)
        with gzip.open(archive, 'rb') as source:
            assert stream_digest(source) == raw_digest
        with archive.open('rb') as source:
            compressed_digest = stream_digest(source)
        records.append(dict(file=path.name, raw_sha256=raw_digest, raw_bytes=path.stat().st_size,
            archive=archive.name, archive_sha256=compressed_digest, compressed_bytes=archive.stat().st_size))
        path.unlink()
    (folder/'compression_manifest.json').write_text(json.dumps(records, indent=2)+'\n')
    return records


@contextmanager
def restored(folder):
    folder = Path(folder)
    records = json.loads((folder/'compression_manifest.json').read_text())
    created = []
    try:
        for row in records:
            path, archive = folder/row['file'], folder/row['archive']
            with archive.open('rb') as source:
                assert stream_digest(source) == row['archive_sha256']
            if not path.exists():
                partial = path.with_suffix(path.suffix+'.restore')
                with gzip.open(archive, 'rb') as source, partial.open('wb') as target:
                    shutil.copyfileobj(source, target)
                with partial.open('rb') as source:
                    assert stream_digest(source) == row['raw_sha256']
                partial.rename(path)
                created.append(path)
            with path.open('rb') as source:
                assert stream_digest(source) == row['raw_sha256']
        yield
    finally:
        for path in created:
            path.unlink()
