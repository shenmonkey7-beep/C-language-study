"""Check Keil source paths, duplicate application files, and entry points."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
project = ET.parse(root / 'Project.uvprojx')
paths = [node.text for node in project.findall('.//FilePath')]
missing = [name for name in paths if not (root / name.replace('\\', '/')).is_file()]
assert not missing, 'Missing Keil inputs: ' + ', '.join(missing)
assert len(paths) == len(set(name.lower() for name in paths)), 'Duplicate Keil inputs'
app = list((root / 'meic_car_basic_code').rglob('*.c'))
assert len(app) == 5, 'Unexpected application translation units'
assert not list(root.glob('*.c')), 'Root application copies must not return'
by_hash = {}
for path in (root / 'meic_car_basic_code').rglob('*'):
    if path.suffix in {'.c', '.h'}:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest not in by_hash, f'Duplicate application files: {path}, {by_hash.get(digest)}'
        by_hash[digest] = path
assert sum('int main(void)' in p.read_text(encoding='utf-8') for p in app) == 1
assert sum('void TIM1_UP_IRQHandler(void)' in p.read_text(encoding='utf-8') for p in app) == 1
print(f'PASS: {len(paths)} Keil inputs exist; 5 application C files; no duplicate app sources')
