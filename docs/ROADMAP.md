# Roadmap

## Delivered through Phase Three

- Mapping diagnostics distinguish unknown ground, slope and height-change causes.
- Fresh-camera coverage and guarded browser manual driving are implemented.
- Four-class NPU inference and GPU terrain geometry have measured deployment evidence.
- Confirmed rock tracking gathers independent depth-supported views and recognizes revisits.

## Next: Phase Four sample-site navigation

The intended workflow is **manual mapping → detected soil patch → evidence checks → operator approval → outward/return preview → approach beside the patch → stopped waiting → explicit Return home**.

All terrain-fixture gates come first, including the recorded flat/edge drift and ramp startup failures. Current ramp/drop-off fixtures also require an actually clear launch disk before operator confirmation can be valid.

Planned additions:

1. **Revisable terrain evidence.** Better, independent observations may overturn weak hazard measurements only after positively observing safe supporting ground. Missing depth, elapsed time or a missing detector box cannot clear a hazard. Clearing terrain will not automatically release a safety stop.
2. **Soil-specific evidence.** A flat soil patch cannot pass a raised-rock geometry test. Its validation needs repeated observations, ground support, uncertainty-aware identity and explicit operator review. Visual classes do not certify material composition.
3. **A stopping pose beside the patch.** Preview a full-footprint approach and return through known ground, initially keeping the padded rover front 1 m outside the estimated patch boundary. Avoid intervening rocks using depth geometry.
4. **Stopped waiting and explicit return.** Preserve the session's original home, wait for operator Return, and retain Stop/takeover/late-response protections.
5. **Recorded acceptance.** Qualify fixtures and repeated scene-50000 missions, with evaluator-only truth, contact evidence, bounded durations and retained failures.

These capabilities are planned, not implemented in this publication. The existing waypoint/exploration APIs are not evidence of accepted autonomous sampling.

## Later milestones

Physical sensor/actuator integration, wheel/IMU fusion, additional scenes, frontier-exploration acceptance, moving-object handling, arm integration and actual sample collection require separate work and evidence.
