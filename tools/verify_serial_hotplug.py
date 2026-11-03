"""Domain 231, PTY emulator only: real ROS/pyserial reconnect, zero commands only."""
import json,os,pty,tempfile,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from smart_delivery_core.serial_to_ros import SerialToROS
from smart_delivery_core.cmd_vel_to_serial import CmdVelToSerialNode,ZERO
from smart_delivery_core.mecanum_odom_real import MecanumOdomReal
from smart_delivery_core.localization_manager import LocalizationManager


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    with tempfile.TemporaryDirectory(prefix='codex_fake_stm32_') as temp:
        path=Path(temp)/'fake_stm32'
        master,slave=pty.openpty();path.symlink_to(os.ttyname(slave));os.set_blocking(master,False)
        rclpy.init(args=['--ros-args','-p','serial_port:='+str(path),'-p','auto_initialize:=false',
                        '-p','automatic_motion_recovery:=false','-p','pose_cache_path:='+str(Path(temp)/'pose.json')])
        rx,tx=SerialToROS(),CmdVelToSerialNode()
        odom,manager=MecanumOdomReal(),LocalizationManager()
        probe=rclpy.create_node('codex_serial_zero_probe')
        pub=probe.create_publisher(Twist,'/chassis_cmd_vel',1)
        executor=SingleThreadedExecutor()
        for node in (rx,tx,odom,manager,probe):executor.add_node(node)
        observed=bytearray();frames=0;masters=[master];slaves=[slave]
        def step(seconds,feedback=True):
            nonlocal frames
            end=time.monotonic()+seconds;send=next_command=0.
            while time.monotonic()<end:
                if (feedback and rx.link.port is not None and tx.link.port is not None
                        and time.monotonic()>=send):
                    os.write(masters[-1],b'A_RT: 0\nB_RT: 0\nC_RT: 0\nD_RT: 0\n')
                    send=time.monotonic()+.50;frames+=1
                if time.monotonic()>=next_command:
                    pub.publish(Twist());next_command=time.monotonic()+.05
                executor.spin_once(timeout_sec=.01)
                if masters[-1] is not None:
                    try:observed.extend(os.read(masters[-1],8192))
                    except (BlockingIOError,OSError):pass
        try:
            step(3.)
            assert tx.armed and tx.accepted_commands>0 and rx.feedback.snapshot(time.monotonic())['healthy']
            assert odom.feedback_assembler.complete_sample_count>=3
            assert manager.recovery_feedback.complete_sample_count>=3
            assert odom.atomic_wheels.sequence==manager.atomic_wheels.sequence
            assert odom.x==0. and odom.y==0. and odom.th==0.
            first=rx.link.session
            os.close(masters[-1]);masters[-1]=None;path.unlink()
            step(1.,False)
            assert not tx.armed and rx.link.port is None and tx.link.port is None
            master,slave=pty.openpty();masters.append(master);slaves.append(slave)
            os.set_blocking(master,False);path.symlink_to(os.ttyname(slave))
            step(3.5)
            assert tx.armed and rx.link.session!=first and rx.link.open_count>=2 and tx.link.open_count>=2
            assert tx.arming_count==2,'Normal 2 Hz feedback must not repeatedly disarm the writer'
            assert odom.atomic_wheels.session==rx.link.session==manager.atomic_wheels.session
            assert odom.atomic_wheels.sequence==manager.atomic_wheels.sequence
            assert observed and all(line+b'\n'==ZERO for line in observed.splitlines())
            print(json.dumps(dict(real_pty_reconnect=True,real_ros_feedback_gate=True,
                 complete_groups=frames,rx_opens=rx.link.open_count,tx_opens=tx.link.open_count,
                 accepted_zero_commands=tx.accepted_commands,atomic_odom_and_manager=True,
                 nonzero_hardware_commands=0,goals=0)))
        finally:
            executor.shutdown()
            manager.pose_cache_writer.close()
            manager.destroy_node();odom.destroy_node();tx.destroy_node();rx.destroy_node();probe.destroy_node();rclpy.shutdown()
            for fd in masters+slaves:
                if fd is not None:
                    try:os.close(fd)
                    except OSError:pass


if __name__=='__main__':main()
