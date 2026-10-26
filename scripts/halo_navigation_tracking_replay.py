#!/usr/bin/env python3
"""Replay recorded tracker observations; reference labels are evaluator-only.

This validates temporal tracking on saved projected RGB-D evidence. It does not
rerun the detector or replace the synchronized raw-depth geometry tests/audits.
"""
import argparse
import json
from pathlib import Path

from rover_navigation.sample_sites import DEFAULTS, SiteStore
from rover_navigation.tracking_evaluation import reference_matches


def replay(events, trial, scene):
    store, frames = None, 0
    positive, negative, rejected_negative = 0, 0, 0
    configuration = None
    false_events, duplicate_events, seen_false, seen_duplicate = [], [], set(), set()
    for event in events:
        if event['kind'] == 'configuration':
            configuration = event
            if event['defaults'] != DEFAULTS:
                raise ValueError('Recorded thresholds differ from this replay implementation')
        if event['kind'] != 'frame':
            continue
        if store is None or store.epoch != event['epoch']:
            store = SiteStore(epoch=event['epoch'])
        store.update(event['observations'], event['stamp'], event['observer'], event['camera'][:3],
                     event['eligible_visible_misses'])
        current = [store.export(item) for item in store.objects if item['state'] == 'confirmed']
        current_score = reference_matches(current, trial['start'], scene)
        for item in current_score['false_confirmed']:
            identity = (event['epoch'], item['track_id'])
            if identity not in seen_false:
                seen_false.add(identity)
                false_events.append(dict(stamp=event['stamp'], **item))
        for name in current_score['duplicate_references']:
            identity = (event['epoch'], name)
            if identity not in seen_duplicate:
                seen_duplicate.add(identity)
                duplicate_events.append(dict(stamp=event['stamp'], reference=name))
        for observation in event['observations']:
            if observation['class_name'] not in DEFAULTS['automatic_classes']:
                continue
            labels = reference_matches([dict(observation, track_id='observation')], trial['start'], scene)
            if labels['matches']:
                positive += 1
            else:
                negative += 1
                rejected_negative += not observation['geometry']['supported']
        frames += 1
    if configuration is None or store is None:
        raise ValueError('No configuration and replayable frames')
    confirmed = [store.export(item) for item in store.objects if item['state'] == 'confirmed']
    scoring = reference_matches(confirmed, trial['start'], scene)
    return dict(test='phase3_recorded_tracking_replay', passed=bool(confirmed) and not scoring['false_confirmed']
                and not scoring['duplicate_references'] and not false_events and not duplicate_events
                and positive > 0 and rejected_negative > 0,
                frames=frames, positive_observations=positive, negative_observations=negative,
                negative_observations_without_raised_support=rejected_negative,
                confirmed=confirmed, reference_evaluation=scoring, configuration=configuration,
                false_confirmation_events=false_events, duplicate_confirmation_events=duplicate_events,
                scope='Recorded projected evidence; simulator reference labels only used after tracking',
                tentative_count=sum(item['state'] in ('tentative', 'unverified') for item in store.objects))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events', type=Path, required=True)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.events.open() as stream:
        result = replay((json.loads(line) for line in stream), json.loads(args.trial.read_text()),
                        json.loads(args.scene.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ('passed', 'frames', 'positive_observations',
                                                'negative_observations_without_raised_support')}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
