"""Real launch I/O callbacks with a temporary shell helper, never Nav2/ROS."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from launch import LaunchDescription,LaunchService
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessIO,OnProcessExit
from smart_delivery_core.navigation_transcript import NavigationTranscript

with TemporaryDirectory(prefix='codex_nav2_transcript_') as directory:
    root=Path(directory)
    helper=root/'component_container_isolated'
    helper.symlink_to('/bin/sh')
    writer=NavigationTranscript(root/'logs')
    service=LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessIO(on_stdout=writer.output,on_stderr=writer.output)),
        RegisterEventHandler(OnProcessExit(on_exit=writer.exited)),
        ExecuteProcess(cmd=[str(helper),'-c',
            'printf "synthetic stdout\\n"; printf "synthetic fatal stderr\\n" >&2; exit 7',
            '__node:=nav2_container'],output='screen')]))
    service.run()
    writer.close()
    assert next((root/'logs').glob('*.stdout.log')).read_text()=='synthetic stdout\n'
    assert next((root/'logs').glob('*.stderr.log')).read_text()=='synthetic fatal stderr\n'
    assert json.loads(next((root/'logs').glob('*.exit.json')).read_text())['returncode']==7
    print('Real launch stdout/stderr/exit capture passed; no ROS nodes or Nav2 processes started.')
