#!/usr/bin/env python3
"""Bounded attended environment-v2 qualification, with read-only RGB-D recording."""
from phase2_supervise_live import main

if __name__=='__main__':
    raise SystemExit(main(phase='environment-v2',record_terrain=True))
