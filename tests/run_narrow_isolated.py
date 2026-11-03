#!/usr/bin/env python3
"""Replay a measured doorway in an isolated Gazebo, checking both posts and all scene obstacles."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--world',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--goal',type=float,nargs=2,default=[12.,8.])
parser.add_argument('--timeout',type=float,default=120.)
parser.add_argument('--expect-blocked',action='store_true')
parser.add_argument('--domain',type=int,default=190)
parser.add_argument('--port',type=int,default=11357)
args = parser.parse_args()
world = args.world.resolve(); output = args.output.resolve();output.mkdir(parents=True,exist_ok=True)
env = dict(os.environ,INDOOR_ROS_DOMAIN_ID=str(args.domain),ROS_DOMAIN_ID=str(args.domain),
           INDOOR_GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}',
           INDOOR_RESULTS_DIR=str(output),INDOOR_RUN_LOG=str(output/'simulation.log'),
           INDOOR_ROS_LOG_DIR=str(output/'ros_logs'))
with socket.socket() as probe:probe.bind(('127.0.0.1',args.port))
simulation = subprocess.Popen([str(ROOT/'scripts/run_sim.sh'),'world:='+str(world),
    'x:=8','y:=0','yaw:=0','gui:=false','use_rviz:=false'],env=env,
    stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
result = 1
try:
    subprocess.run([sys.executable,str(ROOT/'tests/lidar_gate_geometry.py'),
        '--world',str(world),'--output',str(output/'lidar_geometry.json')],env=env,check=True)
    command = [sys.executable,str(ROOT/'tests/navigation_smoke.py'),
        '--obstacle','--existing-obstacle','unit_box_1','--box-size','1',
        '--start-near','8','0','--obstacle-goal',*map(str,args.goal),
        '--timeout',str(args.timeout),'--obstacle-world',str(world),'--report-dir',str(output)]
    if not args.expect_blocked:command.append('--require-door-servo')
    result = subprocess.call(command,env=env)
    if args.expect_blocked:
        reports = sorted(output.glob('navigation_*.json'))
        report = json.loads(reports[-1].read_text()) if reports else {}
        cases = report.get('results',[])
        assert cases and not report['success'], 'blocked doorway must not report success'
        case = cases[0]
        assert min(case['scene_clearances_m'].values()) > .02, 'blocked test must remain collision free'
        assert all(p[0]<9.5 for p in case['trajectory']), 'robot must stay before the blocked gate'
        assert not case['door_phase_counts'].get('3',0), 'undersized gate must not trigger CROSS'
        result = 0
        print('PASS: physically blocked gate rejected; robot stayed before the doorway without contact',flush=True)
finally:
    if simulation.poll() is None:
        simulation.send_signal(signal.SIGINT)
        try:simulation.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(simulation.pid,signal.SIGTERM);simulation.wait(timeout=5)
    if result:
        log = output/'simulation.log'
        if log.exists():print(log.read_text(errors='replace')[-6000:],flush=True)
sys.exit(result)
