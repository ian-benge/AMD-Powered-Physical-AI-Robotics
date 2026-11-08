"""Portable canonical hashes, also usable by a standard-library-only supervisor."""
import hashlib
import json


def stable_numbers(value):
    # Sub-nanometre differences between Windows/Linux BLAS must not change IDs.
    if isinstance(value,float):
        rounded=round(value,10)
        return 0.0 if rounded==0 else rounded
    if isinstance(value,dict):return {k:stable_numbers(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [stable_numbers(v) for v in value]
    return value


def canonical(value):
    return json.dumps(stable_numbers(value),sort_keys=True,separators=(',',':'),allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()
