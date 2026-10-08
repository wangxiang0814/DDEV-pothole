import ast
from pathlib import Path


def test_strategies_and_shared_layers_have_no_forbidden_imports():
    root = Path(__file__).resolve().parents[2] / 'src/ddevsim'
    for folder in ('runtime', 'scenarios', 'vehicle_profiles', 'strategies'):
        for path in (root / folder).rglob('*.py'):
            parts = path.relative_to(root).parts
            tree = ast.parse(path.read_text(encoding='utf-8'))
            names = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    names.append('.' * node.level + (node.module or ''))
            for name in names:
                assert not name.startswith('scripts') and not name.startswith('run_'), (path, name)
                if folder != 'strategies':
                    assert 'strategies.' not in name and 'static_wheel_lift' not in name, (path, name)
                elif parts[1] == 'wheel_lift':
                    assert 'shallow_press' not in name, (path, name)
                elif parts[1] == 'shallow_press':
                    assert 'wheel_lift' not in name and 'static_wheel_lift' not in name, (path, name)


def test_selected_strategy_does_not_import_the_other_strategy():
    import os
    import subprocess
    import sys
    root = Path(__file__).resolve().parents[2]
    script = "from ddevsim.runtime.registry import load_strategy; import sys; load_strategy('shallow_press'); assert not any(k.startswith('ddevsim.strategies.wheel_lift') for k in sys.modules)"
    result = subprocess.run([sys.executable, '-c', script], cwd=root,
                            env={**os.environ, 'PYTHONPATH': str(root / 'src')}, capture_output=True)
    assert result.returncode == 0, result.stderr
