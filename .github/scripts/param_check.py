import pathlib
import re
import subprocess
import sys

import yaml

YAML = 'main_executor/config/main_params.yaml'
READ = re.compile(r'''(?:get_parameter\s*\(\s*"|launch_params\[')([\w.]+)["']\s*[)\]](?:\s*\.\s*as_(\w+))?''')
DYNAMIC = re.compile(r'get_parameter\s*\(\s*[^"\s]')
NODE = re.compile(r'Node\s*\(\s*"(\w+)"')
TYPES = {'bool': bool, 'int': int, 'integer': int, 'double': float, 'string': str}


def flatten(params, prefix=''):
    flat = {}
    for key, value in params.items():
        if isinstance(value, dict):
            flat.update(flatten(value, f'{prefix}{key}.'))
        else:
            flat[prefix + key] = value
    return flat


def type_ok(value, name):
    base = TYPES.get(name.removesuffix('_array'))
    if base is None:
        return True
    if name.endswith('_array'):
        return type(value) is list and all(type(v) is base for v in value)
    return type(value) is base


sections = {k: flatten(v['ros__parameters']) for k, v in yaml.safe_load(open(YAML)).items()}
unused = {(s, k) for s, keys in sections.items() for k in keys}
no_node = set(sections) - {'/**'}
errors = []

tracked = subprocess.run(['git', 'ls-files'], capture_output=True, text=True, check=True).stdout.split()
for package in [pathlib.Path(f).parent for f in tracked if f.endswith('package.xml')]:
    sources = [f for p in ('**/*.cpp', '**/*.hpp', 'launch/*.py') for f in package.glob(p)]
    texts = {f: f.read_text() for f in sources}
    nodes = {n for t in texts.values() for n in NODE.findall(t)} & set(sections)
    no_node -= nodes
    for file, text in texts.items():
        if DYNAMIC.search(text):
            errors.append(f'{file}: パラメータ名が文字列リテラルでない')
        for key, as_type in READ.findall(text):
            found = [s for s in (*nodes, '/**') if key in sections[s]]
            if not found:
                errors.append(f'{file}: {key} が yaml に無い')
            for s in found:
                unused.discard((s, key))
                if not type_ok(sections[s][key], as_type):
                    errors.append(f'{file}: {s}.{key} = {sections[s][key]!r} は as_{as_type} と型が合わない')

errors += [f'{YAML}: {s} に対応するノードが無い' for s in sorted(no_node)]
errors += [f'{YAML}: {s}.{k} はどこからも読まれていない' for s, k in sorted(unused)]
print('\n'.join(errors) or 'ok')
sys.exit(bool(errors))
