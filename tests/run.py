"""Run host regressions; GCC, Clang or TinyCC must be installed separately."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default=shutil.which('cc') or shutil.which('gcc') or shutil.which('clang') or shutil.which('tcc'))
args = parser.parse_args()
if not args.cc:
    parser.error('No C compiler found. Supply --cc /path/to/compiler')
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='meic-tests-') as directory:
    binary = Path(directory) / 'test_control.exe'
    subprocess.run([args.cc, '-std=c99', '-Wall', '-Werror', '-I'+str(root/'tests'),
                    '-I'+str(root/'meic_car_basic_code/Hardware'),
                    str(root/'tests/test_control.c'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
