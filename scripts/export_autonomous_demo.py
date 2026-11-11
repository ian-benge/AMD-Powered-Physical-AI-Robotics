#!/usr/bin/env python3
"""Export preserved demo records. Missing live gates stay incomplete."""
import argparse
import base64
import csv
import hashlib
import html
import json
from pathlib import Path


COLORS={'exploring':'#26b8b8','exploration_return':'#2970da','sample_outbound':'#b76a00','sample_return':'#9455d3'}


def read_rows(path):return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def export(events_path,observer_path,output,final_path=None,map_path=None):
    output.mkdir(parents=True,exist_ok=False)
    events=read_rows(events_path);rows=read_rows(observer_path)
    snapshots=[r for r in rows if r.get('status',{}).get('demo_status')]
    if not snapshots:raise ValueError('No synchronized mission snapshots; no report can be inferred')
    ids={r['status']['demo_status']['mission_id'] for r in snapshots}
    if len(ids)!=1:raise ValueError('Export one continuous mission identity at a time; preserve restarts separately')
    final=json.loads(final_path.read_text()) if final_path else snapshots[-1]['status']
    d=final['demo_status']
    if d['mission_id'] not in ids:raise ValueError('Final snapshot belongs to another mission')
    with (output/'route.csv').open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(['mission_id','phase','wall_monotonic_seconds','simulation_seconds','frame','x_m','y_m','yaw_rad'])
        for r in events:
            if r.get('kind')=='trajectory':writer.writerow([r['mission_id'],r['phase'],r['wall'],r['sim_time'],'odom',*r['odom_pose']])
    (output/'mission-events.json').write_text(json.dumps(events,indent=2)+'\n')
    (output/'sample.json').write_text(json.dumps(d.get('sample'),indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    def wall(row):return row.get('wall_monotonic',row.get('windows_monotonic_ms',0)/1000)
    zero=wall(snapshots[0]);times=[wall(r)-zero for r in snapshots]
    metrics=[('Depth-supported coverage','m²',lambda s:(s.get('demo_status',{}).get('coverage') or {}).get('measured_area_m2')),
             ('Map publication p95','ms',lambda s:s.get('mapping',{}).get('publication_latency_p95_seconds')),
             ('Tctl temperature','°C',lambda s:(s.get('platform_temperature') or {}).get('tctl_c')),
             ('Measured real-time factor','ratio',lambda s:s.get('demo_status',{}).get('real_time_factor'))]
    fig,axes=plt.subplots(4,1,figsize=(10,9),sharex=True,layout='constrained')
    for ax,(title,unit,value) in zip(axes,metrics):
        y=[value(r['status']) for r in snapshots];scale=1000 if unit=='ms' else 1
        ax.plot(times,[float('nan') if v is None else v*scale for v in y],lw=1.4)
        ax.set_ylabel(unit);ax.set_title(title,loc='left',fontsize=11);ax.grid(alpha=.2)
        if unit=='ms':ax.axhline(200,color='#c33',ls='--',lw=1)
        if unit=='°C':ax.axhline(87,color='#c33',ls='--',lw=1,label='Qualification stop');ax.legend(loc='upper left')
    axes[-1].set_xlabel('Wall seconds since first recorded snapshot')
    for suffix in ('png','svg','pdf'):fig.savefig(output/('mission-telemetry.'+suffix),dpi=180)
    plt.close(fig)
    latency=[r['status'].get('mapping',{}).get('publication_latency_p95_seconds') for r in snapshots]
    rtf=[r['status']['demo_status'].get('real_time_factor') for r in snapshots if r['status']['demo_status'].get('elapsed_seconds',0)>=10]
    temps=[(r['status'].get('platform_temperature') or {}).get('tctl_c') for r in snapshots]
    valid=lambda a:[x for x in a if x is not None and np.isfinite(x)]
    summary=dict(mission_id=d['mission_id'],phase=d['phase'],reason=d['reason'],sample=d.get('sample'),
        recorded_wall_seconds=times[-1],coverage=d.get('coverage'),actual_distance_m=d.get('actual_distance_m'),
        largest_reported_rolling_publication_p95_ms=max(valid(latency),default=float('nan'))*1000,
        median_reported_rolling_rtf=float(np.median(valid(rtf))) if valid(rtf) else None,
        peak_tctl_c=max(valid(temps),default=None),
        acceptance='PENDING_INDEPENDENT_LIVE_REVIEW',
        incomplete_gates=['matched_20m_baseline','contact_fixture','rendered_ground_15mm','localization_drift_0.10m_5deg',
            'home_return_truth_review','100_image_manual_diagnostic_review','three_primary_and_two_variation_cycles','30_minute_continuous_recording'],
        caveat='A completed state machine is not independent mission acceptance. Rolling RTF is not a per-sample median baseline test.')
    if not valid(latency):summary['largest_reported_rolling_publication_p95_ms']=None
    (output/'run-report.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    m=final.get('map')
    if m and map_path:
        width,height=m['width'],m['height'];res=m['resolution'];origin=m['origin']
        xy=lambda p:((p[0]-origin[0])/res,height-(p[1]-origin[1])/res)
        image=base64.b64encode(map_path.read_bytes()).decode()
        body=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height+70}"><rect width="100%" height="100%" fill="#172633"/>',
              f'<image width="{width}" height="{height}" href="data:image/png;base64,{image}"/>']
        for phase,color in COLORS.items():
            points=' '.join(f'{x:.3f},{y:.3f}' for x,y in (xy(t['pose']) for t in d.get('trails',[]) if t['phase']==phase))
            if points:body.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
        for point,label in ((d.get('home'),'Home'),(d.get('target_map'),'Simulated sample')):
            if point:
                x,y=xy(point);body.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#fff"/><text x="{x+6}" y="{y-6}" fill="#fff" font-size="12">{label}</text>')
        body.append(f'<text x="10" y="{height+22}" fill="white" font-size="12">{html.escape(d["mission_id"])} | {html.escape(d["phase"])}</text>')
        body.append(f'<text x="10" y="{height+42}" fill="white" font-size="11">Solid trails: exploration teal / first return blue / sample outward amber / final return purple.</text>')
        body.append(f'<text x="10" y="{height+60}" fill="white" font-size="11">Estimated map coordinates. Gray unknown; pale known; red hazard. Collection is simulated.</text></svg>')
        (output/'annotated-map.svg').write_text(''.join(body))
    inputs=[events_path,observer_path]+[p for p in (final_path,map_path) if p]
    manifest={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    manifest.update({str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()})
    (output/'SHA256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('events','observer','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--final',type=Path);p.add_argument('--map',type=Path)
    a=p.parse_args();print(json.dumps(export(a.events,a.observer,a.output,a.final,a.map),indent=2))
