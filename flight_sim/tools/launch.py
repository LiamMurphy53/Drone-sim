#!/usr/bin/env python3
"""Own and clean up only the Betaflight process launched for this simulator."""
import argparse, json, socket, subprocess, sys, time
from pathlib import Path
from configure_betaflight import BIN, ROOT, RUNTIME, CONFIG_MARKER, configure

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--scenario', choices=['flight', 'throttle', 'steering'], default='flight',
                        help='Live Betaflight scenario to run with --test.')
    parser.add_argument('--test-fps', type=int, choices=[30, 60, 120], default=60,
                        help='Frame rate for --test; physics stays at 500 Hz.')
    profiles=json.loads((ROOT/'config/aircraft_catalog.json').read_text())
    parser.add_argument('--aircraft', choices=[p['id'] for p in profiles], default='gopro_drone',
                        help='Aircraft for --test. Interactive flight uses the saved in-app selection.')
    args=parser.parse_args()
    godot=ROOT/'.tools/Godot.app/Contents/MacOS/Godot'
    if not godot.exists() or not BIN.exists():
        raise SystemExit('Run tools/setup.sh first to download Godot and build Betaflight.')
    if not CONFIG_MARKER.exists(): configure()
    for port,kind in [(5761,socket.SOCK_STREAM),(9001,socket.SOCK_DGRAM),(9003,socket.SOCK_DGRAM),(9004,socket.SOCK_DGRAM)]:
        try:
            with socket.socket(socket.AF_INET,kind) as sock: sock.bind(('127.0.0.1',port))
        except OSError:
            raise SystemExit(f'Port {port} is already in use. Close the other simulator instance and retry.')
    with (RUNTIME/'betaflight.log').open('w') as log:
        bf=subprocess.Popen([str(BIN)],cwd=RUNTIME,stdout=log,stderr=log)
        game=None
        try:
            time.sleep(1)
            if bf.poll() is not None: raise RuntimeError('Betaflight exited; see runtime/betaflight.log')
            command=[str(godot),'--path',str(ROOT)]
            if args.test:
                script = {'flight':'test_integration.gd', 'throttle':'test_throttle.gd',
                          'steering':'test_steering.gd'}[args.scenario]
                command += ['--headless','--script','res://tests/'+script,'--',args.aircraft,str(args.test_fps)]
            game=subprocess.Popen(command,cwd=ROOT)
            while game.poll() is None:
                if bf.poll() is not None:
                    game.terminate()
                    raise RuntimeError('Betaflight stopped unexpectedly. See runtime/betaflight.log')
                time.sleep(.1)
            return game.returncode
        finally:
            if game and game.poll() is None:
                game.terminate()
                try: game.wait(3)
                except subprocess.TimeoutExpired: game.kill(); game.wait()
            if bf.poll() is None:
                bf.terminate()
                try: bf.wait(3)
                except subprocess.TimeoutExpired: bf.kill(); bf.wait()
if __name__=='__main__':
    try: sys.exit(main())
    except KeyboardInterrupt: pass
