"""Run the explicit functional suite independently of ROS collection plugins."""
import sys
from pathlib import Path

import pytest


def main():
    root = Path(__file__).resolve().parents[1]
    excluded = {'test_copyright.py', 'test_flake8.py', 'test_pep257.py'}
    suites = ('smart_delivery_core/test', 'smart_carrier_api/tests', 'hailo_vision/test', 'power_monitor/test')
    files = sorted(path for suite in suites
                   for path in (root/'packages'/suite).rglob('test_*.py')
                   if path.name not in excluded)
    if not files or not all(path.is_file() for path in files):
        raise RuntimeError('Functional test inventory is incomplete')
    print(f'Functional suite: {len(files)} files; pytest={pytest.__version__}', flush=True)
    # Ruff runs separately. Existing ament style suites are not functional tests.
    return pytest.main(['-q', '-p', 'no:cacheprovider', *map(str, files), *sys.argv[1:]])


if __name__ == '__main__':
    raise SystemExit(main())
