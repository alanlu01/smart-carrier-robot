"""Container diagnostics must neither capture API credentials nor control nodes."""
import json
from types import SimpleNamespace as S
from smart_delivery_core.navigation_transcript import NavigationTranscript


def test_only_nav2_container_output_and_exit_are_saved(tmp_path):
    writer=NavigationTranscript(tmp_path)
    event=S(cmd=['/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated',
                 '--ros-args','__node:=nav2_container'],pid=123,text=b'C++ fatal detail\n',from_stderr=True,returncode=-6)
    writer.output(S(cmd=['api_bridge','SECRET'],pid=99,text=b'credential',from_stderr=True))
    writer.output(event)
    writer.exited(event)
    assert (tmp_path/'nav2_container_123.stderr.log').read_bytes()==event.text
    result=json.loads((tmp_path/'nav2_container_123.exit.json').read_text())
    assert result['returncode']==-6 and result['dropped_bytes']==0
    assert len(list(tmp_path.iterdir()))==2
