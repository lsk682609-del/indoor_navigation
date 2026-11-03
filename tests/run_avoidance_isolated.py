#!/usr/bin/env python3
"""Run actual Gazebo avoidance regression independently of the user's GUI instance."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path(os.environ.get(
    'INDOOR_AVOIDANCE_OUTPUT_DIR', ROOT / 'results/isolated_avoidance')))
parser.add_argument('--upper-corridor', action='store_true')
parser.add_argument('--world', type=Path)
args = parser.parse_args()
if args.upper_corridor and args.world is None:
    parser.error('--upper-corridor requires --world containing unit_box_0')
OUTPUT = args.output.resolve()
OUTPUT.mkdir(parents=True, exist_ok=True)
env = dict(os.environ,
           INDOOR_ROS_DOMAIN_ID='189', ROS_DOMAIN_ID='189',
           INDOOR_GAZEBO_MASTER_URI='http://127.0.0.1:11356',
           INDOOR_RESULTS_DIR=str(OUTPUT), INDOOR_TEST_REPORT_DIR=str(OUTPUT),
           INDOOR_RUN_LOG=str(OUTPUT/'simulation.log'),
           INDOOR_ROS_LOG_DIR=str(OUTPUT/'ros_logs'))
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 11356))
launch = [str(ROOT/'scripts/run_sim.sh'), 'x:=0', 'y:=8', 'yaw:=0'] if args.upper_corridor else [str(ROOT/'scripts/run_avoidance.sh')]
if args.world:
    launch.append('world:='+str(args.world.resolve()))
simulation = subprocess.Popen(launch + ['gui:=false', 'use_rviz:=false'],
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                              start_new_session=True)
result = 1
try:
    print('Isolated Gazebo: ROS domain 189, master port 11356; user GUI domain unchanged', flush=True)
    if args.upper_corridor:
        result = subprocess.call([
            sys.executable, str(ROOT/'tests/navigation_smoke.py'), '--obstacle',
            '--existing-obstacle', 'unit_box_0', '--box-size', '1',
            '--start-near', '0', '8', '--obstacle-goal', '12', '8',
            '--timeout', '60', '--max-translation-pause', '5',
            '--report-dir', str(OUTPUT)], env=env)
    else:
        result = subprocess.call([str(ROOT/'scripts/validate.sh'), '--avoidance-regression'], env=env)
finally:
    if simulation.poll() is None:
        simulation.send_signal(signal.SIGINT)
        try:
            simulation.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(simulation.pid, signal.SIGTERM)
            simulation.wait(timeout=5)
    if result:
        print((OUTPUT/'simulation.log').read_text()[-8000:], flush=True)
print('Avoidance results:', OUTPUT, flush=True)
sys.exit(result)
