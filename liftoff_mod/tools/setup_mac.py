#!/usr/bin/env python3
"""Install pinned build tools privately, build, test, and stage the plugin.

Does not modify Liftoff.app, Steam settings, or global runtimes.
"""
import argparse
import hashlib
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SDK = '8.0.425'
SDK_HASHES = {
    'arm64': 'd42156be70b489236479a415536013a89a9fd6eaca57d5716a7017612fcc65bdb459068973d7cffd6fe710f2799af8bee95b2046ba032762d47df176f5c4a7d6',
    'x64': '3b27552358905f1bde600dd0315a23051ef1cbd055a50eb64c675ee25a04e658c2ed735177125436c5bec38dfcd958fee338299611348299cd34a9bea8b8011f',
}
BEP_VERSION = '5.4.23.5'
BEP_HASH = '01c2ae782eb016dfd6c345a18dbd2dcafffb3d9d318449d6486689f426b4a323'

def download(url, target, expected, algorithm):
    if not target.exists():
        partial = target.with_suffix(target.suffix + '.part')
        subprocess.run(['curl', '--fail', '--location', '--retry', '2', '--output', str(partial), url], check=True)
        partial.replace(target)
    digest = hashlib.new(algorithm)
    with target.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != expected:
        raise RuntimeError(f'Checksum mismatch: {target}. Remove this download and retry.')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--game', type=Path, default=Path.home() / 'Library/Application Support/Steam/steamapps/common/Liftoff/Liftoff.app')
    parser.add_argument('--build-only', action='store_true', help='Use existing local tools; do not download.')
    args = parser.parse_args()
    if sys.platform != 'darwin':
        parser.error('Mac setup only. Shared C# sources do not yet have a tested Windows integration.')
    managed = args.game.resolve() / 'Contents/Resources/Data/Managed'
    if not (managed / 'Assembly-CSharp.dll').is_file():
        parser.error('Install Liftoff through Steam first, or pass --game /path/to/Liftoff.app.')
    local = ROOT / '.tools'
    local.mkdir(exist_ok=True)
    if not args.build_only:
        arch = 'arm64' if platform.machine() == 'arm64' else 'x64'
        sdk_archive = local / 'dotnet-sdk.tar.gz'
        download(f'https://dotnetcli.blob.core.windows.net/dotnet/Sdk/{SDK}/dotnet-sdk-{SDK}-osx-{arch}.tar.gz', sdk_archive, SDK_HASHES[arch], 'sha512')
        if not (local / 'dotnet/dotnet').is_file():
            (local / 'dotnet').mkdir(exist_ok=True)
            with tarfile.open(sdk_archive) as archive:
                archive.extractall(local / 'dotnet')
        bep_archive = local / 'bepinex-mac.zip'
        download(f'https://github.com/BepInEx/BepInEx/releases/download/v{BEP_VERSION}/BepInEx_macos_universal_{BEP_VERSION}.zip', bep_archive, BEP_HASH, 'sha256')
        if not (local / 'bepinex/BepInEx/core/BepInEx.dll').is_file():
            with zipfile.ZipFile(bep_archive) as archive:
                archive.extractall(local / 'bepinex')
    dotnet = local / 'dotnet/dotnet'
    core = local / 'bepinex/BepInEx/core'
    if not dotnet.is_file() or not (core / 'BepInEx.dll').is_file():
        parser.error('Local tools are missing. Run setup without --build-only.')
    env = os.environ.copy()
    env.update(DOTNET_CLI_HOME=str(local / 'cli-home'), DOTNET_CLI_TELEMETRY_OPTOUT='1',
               DOTNET_SKIP_FIRST_TIME_EXPERIENCE='1', DOTNET_GENERATE_ASPNET_CERTIFICATE='false')
    subprocess.run([str(dotnet), 'build', str(ROOT / 'GoProGeometry.csproj'), '-c', 'Release',
                    '-p:UseSharedCompilation=false', f'-p:LiftoffManagedDir={managed}', '--nologo'], env=env, check=True, cwd=ROOT)
    subprocess.run([str(dotnet), 'run', '--project', str(ROOT / 'tests/GeometryChecks.csproj'),
                    '-p:UseSharedCompilation=false', '--', str(ROOT / 'profiles/gopro-drone.json')], env=env, check=True, cwd=ROOT)
    config = local / 'bepinex/BepInEx/config/BepInEx.cfg'
    config.parent.mkdir(parents=True, exist_ok=True)
    if config.exists():
        text = config.read_text().replace('HideManagerGameObject = false', 'HideManagerGameObject = true').replace('EnableAssemblyCache = true', 'EnableAssemblyCache = false')
    else:
        text = '[Caching]\nEnableAssemblyCache = false\n\n[Chainloader]\nHideManagerGameObject = true\n'
    config.write_text(text)
    plugin = local / 'bepinex/BepInEx/plugins/GoProGeometry'
    plugin.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / 'bin/Release/netstandard2.1/GoProGeometry.dll', plugin)
    shutil.copytree(ROOT / 'profiles', plugin / 'profiles', dirs_exist_ok=True)
    print('\nGeometry Lab built and staged. Open Steam, then launch Liftoff Geometry Lab.command.')

if __name__ == '__main__':
    main()
