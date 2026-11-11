#!/usr/bin/env python3
"""Reserve the unseen 100-image diagnostic evaluation, without rendering/training."""
import argparse
import hashlib
import json
from pathlib import Path
from bulk10000_plan import split_for_seed,reservation


def reserve():
    used={s['seed'] for b in reservation('not-generated')['batches'] for s in b['scenes']}
    seeds=[];seed=6500000
    while len(seeds)<5:
        if seed not in used and split_for_seed(seed)=='test':seeds.append(seed)
        seed+=1
    return dict(schema_version=1,purpose='manual_demo_diagnostic_evaluation_only',
        captured=False,reviewed=False,training_permitted=False,detectors_frozen=True,
        image_count=100,scene_count=5,source_annotations='legacy_three_class',
        export_annotations='metadata_based_four_class_including_mudstone_slab',
        asset_pool='held_out_test_assets_only',baseline_and_candidate_comparison_required=True,
        scenes=[dict(seed=s,scene_group=f'seed_{s}',split='test',views=20,background=i==4,
            density=('sparse','standard','cluttered','sparse','standard')[i],
            sunlight=('low','medium','high','medium','low')[i],
            appearance_family=f'test:seed_{s}',
            primary_target_views={} if i==4 else {c:5 for c in ('rock','pale_soil_patch','yellow_crystal_rock','mudstone_slab')})
            for i,s in enumerate(seeds)],
        review_fields=['image_sha256','labels_sha256','all_visible_eligible_targets_labeled','class_conversion_correct',
            'occlusion_and_clipping_correct','background_has_no_eligible_target','human_reviewer','review_utc','decision'],
        eligibility='Validate against all training/capture manifests and held-out asset hashes before generation; no reuse of inspected scenes')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);a=p.parse_args()
    value=reserve();body=json.dumps(value,indent=2,sort_keys=True)+'\n'
    with a.output.open('x') as f:f.write(body)
    print(hashlib.sha256(body.encode()).hexdigest(),str(a.output))
