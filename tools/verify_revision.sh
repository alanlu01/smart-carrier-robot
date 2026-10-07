#!/usr/bin/env bash
# Static/mocked verification ONLY: never launch nodes or send robot commands.
set -e
source /opt/ros/jazzy/setup.bash
source /home/kj0921/dev_ws/install/setup.bash
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD/packages/smart_delivery_core:$PWD/packages/hailo_vision:$PWD/packages/power_monitor:$PWD/packages/smart_carrier_api:$PYTHONPATH"
python3 -m compileall -q packages/smart_delivery_core/smart_delivery_core \
  packages/hailo_vision/hailo_vision packages/smart_carrier_api/smart_carrier_api
python3 -m pytest --import-mode=importlib -q \
  packages/smart_delivery_core/test packages/hailo_vision/test \
  packages/power_monitor/test packages/smart_carrier_api/tests \
  --ignore-glob='*/test_copyright.py' --ignore-glob='*/test_flake8.py' \
  --ignore-glob='*/test_pep257.py'
