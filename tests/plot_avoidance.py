#!/usr/bin/env python3
"""Export the measured Gazebo trajectories; not planner path visualizations."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=Path(os.environ.get('INDOOR_AVOIDANCE_OUTPUT_DIR', ROOT/'results/isolated_avoidance'))
reports=[]
for path in sorted(OUTPUT.glob('navigation_*.json')):
    report=json.loads(path.read_text())
    if report.get('success') and report.get('results'):
        reports.append((path,report['results'][0]))
cases={tuple(case['goal']):(path,case) for path,case in reports}
assert all(goal in cases for goal in [(0.,4.),(0.,0.),(6.,0.)]), 'all three live tests must pass first'
fig,axes=plt.subplots(1,3,figsize=(16,5),constrained_layout=True)
world=ET.parse(ROOT/'src/indoor_bringup/worlds/indoor_obstacle.world').getroot().find('world')
settings=[((0.,4.),'1 m box: long approach','#2468b4',(-2.1,9.5),(-1.5,5)),
          ((0.,0.),'1 m box: reverse','#ca5a19',(-2.1,2.2),(-1.5,5)),
          ((6.,0.),'0.5 m box: passing','#2b8e54',(-1,6.5),(-1.5,3))]
for ax,(goal,label,color,xlimits,ylimits) in zip(axes,settings):
    for model in world.findall('model'):
        box=model.find('link/collision/geometry/box')
        if box is None:continue
        x,y,*_=map(float,model.findtext('pose','0 0 0 0 0 0').split())
        sx,sy,_=map(float,box.findtext('size').split())
        ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,
                              facecolor='#555b65' if model.get('name')!='unit_box' else '#bb3a3a',alpha=.7))
    if goal==(6.,0.):
        ax.add_patch(Rectangle((2.75,-.25),.5,.5,facecolor='#d7aa29',alpha=.7))
    _,case=cases[goal]
    trajectory=case['trajectory']
    ax.plot([p[0] for p in trajectory],[p[1] for p in trajectory],color=color,lw=1.8)
    ax.scatter(trajectory[0][0],trajectory[0][1],marker='o',s=25,color=color,zorder=4)
    ax.scatter(*goal,marker='x',s=45,color=color,zorder=4)
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
    ax.set(xlim=xlimits,ylim=ylimits,title=label+f'\nMin sampled body clearance: {case["minimum_box_clearance_m"]:.3f} m')
fig.savefig(OUTPUT/'trajectories.png',dpi=160)
manifest={str(goal):dict(report=str(path.relative_to(ROOT)),
                        seconds=case['elapsed_wall_s'],
                        position_error_m=case['gazebo_truth_error_m'],
                        minimum_box_clearance_m=case['minimum_box_clearance_m'])
          for goal,(path,case) in cases.items()}
(OUTPUT/'summary.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(OUTPUT/'trajectories.png')
print(json.dumps(manifest,indent=2))
