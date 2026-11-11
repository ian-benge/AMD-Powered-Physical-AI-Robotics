"""Display-only telemetry from the native capture guard; never authorizes motion."""
import json
import math
import os
from pathlib import Path
import tempfile
import time


def write_temperature(path, health, boot_id, case):
    path = Path(path)
    payload = dict(source='native_capture_guard', boot_id=boot_id, case=Path(case).name,
                   utc=health['utc'], wall_monotonic=time.monotonic(), tctl_c=health['tctl_c'])
    encoded = json.dumps(payload, allow_nan=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.demo-temperature-',
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded+'\n')
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def read_temperature(path, boot_id, now=None):
    """Discard stale, foreign-boot or malformed readings instead of freezing a value."""
    now = time.monotonic() if now is None else now
    try:
        path = Path(path)
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 4096:
            return None
        value = json.loads(path.read_text())
        temperature, stamp = value['tctl_c'], value['wall_monotonic']
        if (value['source'] != 'native_capture_guard' or value['boot_id'] != boot_id or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in (temperature, stamp)) or not 0 <= temperature <= 125 or
                not -.05 <= now-stamp <= 2.):
            return None
        return dict(tctl_c=temperature, age_seconds=max(0., now-stamp), utc=value['utc'],
                    source=value['source'], qualification_stop_c=87, native_stop_c=90)
    except (OSError, ValueError, KeyError, TypeError):
        return None
