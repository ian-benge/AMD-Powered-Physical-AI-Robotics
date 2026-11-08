#!/usr/bin/env bash
# Historical interface retired: the old post-trial grep could miss an incident
# until ten minutes later and had no live thermal/capture gates.
set -euo pipefail
echo 'Use phase1_run_fixture.py once per trial with fresh output, --seconds 600 and the current capture/kit.' >&2
exit 2
