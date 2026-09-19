#!/usr/bin/env python3
"""Launch the installed Mac game with this project's isolated mod loader."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GAME = Path.home() / 'Library/Application Support/Steam/steamapps/common/Liftoff/Liftoff.app'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--game', type=Path, default=DEFAULT_GAME)
    args = parser.parse_args()
    if sys.platform != 'darwin':
        parser.error('This launcher is for macOS. Windows needs its own loader and compatibility check.')
    game = args.game.resolve()
    exe = game / 'Contents/MacOS/Liftoff'
    loader = ROOT / '.tools/bepinex'
    for required in [exe, loader / 'libdoorstop.dylib', loader / 'BepInEx/core/BepInEx.Preloader.dll', loader / 'BepInEx/plugins/GoProGeometry/GoProGeometry.dll']:
        if not required.is_file():
            parser.error(f'Missing {required}. Run setup/build first.')
    processes = subprocess.run(['/bin/ps', '-axo', 'comm='], capture_output=True, text=True)
    if processes.returncode != 0:
        parser.error('Cannot check running games. Run this launcher from Terminal or Finder.')
    if str(exe) in [line.strip() for line in processes.stdout.splitlines()]:
        parser.error('Liftoff is already running. Quit it before launching Geometry Lab.')
    if subprocess.run(['/usr/bin/pgrep', '-x', 'steam_osx'], stdout=subprocess.DEVNULL).returncode != 0:
        subprocess.run(['/usr/bin/open', '-a', 'Steam'], check=True)
        parser.error('Steam is starting. Sign in and launch Geometry Lab again when Steam is ready.')
    if subprocess.run(['/usr/bin/arch', '-x86_64', '/usr/bin/true'], capture_output=True).returncode:
        parser.error('The mod uses Intel-mode Liftoff. Install Apple Rosetta, then retry.')
    runtime = ROOT / 'runtime'
    runtime.mkdir(exist_ok=True)
    env = os.environ.copy()
    env.update(DOORSTOP_ENABLED='1', DOORSTOP_TARGET_ASSEMBLY=str(loader / 'BepInEx/core/BepInEx.Preloader.dll'),
               SteamAppId='410340', SteamGameId='410340')
    cmd = ['/usr/bin/arch', '-x86_64', '-e', 'DYLD_INSERT_LIBRARIES=' + str(loader / 'libdoorstop.dylib'),
           str(exe), '-logFile', str(runtime / 'player.log')]
    with (runtime / 'launcher.log').open('w') as log:
        proc = subprocess.Popen(cmd, cwd=game.parent, env=env, stdout=log, stderr=log, start_new_session=True)
    (runtime / 'game.pid').write_text(str(proc.pid) + '\n')
    print(f'Liftoff launched with Geometry Lab (PID {proc.pid}).')
    print(f'Log: {runtime / "player.log"}')

if __name__ == '__main__':
    main()
