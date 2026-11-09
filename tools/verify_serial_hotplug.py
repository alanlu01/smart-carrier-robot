"""Domain 231, PTY emulator only: real ROS/pyserial reconnect, zero commands only."""
import json,os,pty,tempfile,time,signal,threading,tty
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from smart_delivery_core.serial_to_ros import SerialToROS
from smart_delivery_core.cmd_vel_to_serial import CmdVelToSerialNode,ZERO
from smart_delivery_core.mecanum_odom_real import MecanumOdomReal
from smart_delivery_core.localization_manager import LocalizationManager
from smart_delivery_core.odom_transport import LOCAL_ODOM_QOS
from std_msgs.msg import String


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
    assert not os.environ.get('ROS_STATIC_PEERS')
    with tempfile.TemporaryDirectory(prefix='codex_fake_stm32_') as temp:
        path=Path(temp)/'fake_stm32'
        master,slave=pty.openpty();tty.setraw(slave);path.symlink_to(os.ttyname(slave));os.set_blocking(master,False)
        rclpy.init(args=['--ros-args','-p','serial_port:='+str(path),'-p','auto_initialize:=false',
                        '-p','automatic_motion_recovery:=false','-p','pose_cache_path:='+str(Path(temp)/'pose.json')])
        rx,tx=SerialToROS(),CmdVelToSerialNode()
        odom,manager=MecanumOdomReal(),LocalizationManager()
        probe=rclpy.create_node('codex_serial_zero_probe')
        pub=probe.create_publisher(Twist,'/chassis_cmd_vel',1)
        control_ages=[]
        def observe_control(message):
            p=json.loads(message.data)
            control_ages.append((time.time_ns()-p['sample_ros_stamp_ns'])/1e9)
        probe.create_subscription(String,'/chassis/serial_feedback',observe_control,LOCAL_ODOM_QOS)
        executor=SingleThreadedExecutor()
        for node in (rx,tx,odom,manager,probe):executor.add_node(node)
        observed=bytearray();frames=0;masters=[master];slaves=[slave]
        feeding=[True];feed_stop=threading.Event()
        health_cache={}
        def health(node):
            item=node.health.get()
            now=time.monotonic()
            if item:health_cache[node]=(now,item[1])
            cached=health_cache.get(node)
            # The mailbox intentionally skips contended reads. Keep the last
            # snapshot briefly, never confuse a skipped read with empty data.
            return cached[1] if cached and now-cached[0]<=.35 else {}
        def feed():
            nonlocal frames
            while not feed_stop.wait(.5):
                if feeding[0] and masters[-1] is not None and health(rx).get('connected') and health(tx).get('connected'):
                    try:
                        os.write(masters[-1],b'A_RT: 0\nB_RT: 0\nC_RT: 0\nD_RT: 0\n')
                        frames+=1
                    except (OSError,BlockingIOError):pass
        feeder=threading.Thread(target=feed,daemon=True);feeder.start()
        def step(seconds,feedback=True):
            feeding[0]=feedback
            end=time.monotonic()+seconds;next_command=0.
            while time.monotonic()<end:
                if time.monotonic()>=next_command:
                    pub.publish(Twist());next_command=time.monotonic()+.05
                executor.spin_once(timeout_sec=.01)
                if masters[-1] is not None:
                    try:observed.extend(os.read(masters[-1],8192))
                    except (BlockingIOError,OSError):pass
        try:
            step(8.)
            assert health(tx).get('armed') and health(tx).get('accepted_commands',0)>0
            assert health(rx).get('healthy')
            assert odom.feedback_assembler.complete_sample_count>=3
            assert manager.recovery_feedback.complete_sample_count>=3
            assert abs(odom.atomic_wheels.sequence-manager.atomic_wheels.sequence)<=1
            assert odom.x==0. and odom.y==0. and odom.th==0.
            assert len(control_ages)>20
            regular_feedback_max_age=max(control_ages)
            assert regular_feedback_max_age < .35
            # The old large health channel is diagnostic only. Losing its
            # publications must not make TX consume a stale wheel snapshot.
            original_health_publish=rx.health_pub.publish
            rx.health_pub.publish=lambda _:None
            arming=health(tx)['arming_count']
            step(2.)
            rx.health_pub.publish=original_health_publish
            assert health(tx)['armed'] and health(tx)['arming_count']==arming
            first=health(rx)['session']
            def usage():
                result={}
                for label,pid in [('rx_io',rx.workers[0].pid),('rx_diag',rx.workers[1].pid),
                                  ('tx_io',tx.workers[0].pid),('tx_diag',tx.workers[1].pid)]:
                    stat=Path(f'/proc/{pid}/stat').read_text();fields=stat[stat.rfind(')')+2:].split()
                    result[label]=dict(ticks=int(fields[11])+int(fields[12]),
                        rss_mib=int(Path(f'/proc/{pid}/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/2**20)
                return result
            used_before=usage();usage_at=time.monotonic();step(5.);used_after=usage()
            usage_seconds=time.monotonic()-usage_at
            resource_usage={label:dict(cpu_one_core_percent=(end['ticks']-used_before[label]['ticks'])
                /os.sysconf('SC_CLK_TCK')/usage_seconds*100,rss_mib=end['rss_mib'])
                for label,end in used_after.items()}

            # Hard-stall both diagnostic publishers. Source reads/control must continue.
            for owner in (rx,tx):os.kill(owner.workers[1].pid,signal.SIGSTOP)
            before=health(rx)['complete_sample_count'];arming=health(tx)['arming_count']
            step(3.)
            assert health(rx)['complete_sample_count']>=before+5
            assert health(tx)['armed'] and health(tx)['arming_count']==arming
            for owner in (rx,tx):os.kill(owner.workers[1].pid,signal.SIGCONT)
            step(1.)
            # Stall the actual control publish call while independent telemetry
            # feeding and both hardware I/O loops keep running.
            real_publish=rx.sample_pub.publish;blocked=[False]
            def delayed_publish(message):
                if not blocked[0]:
                    blocked[0]=True;time.sleep(2.)
                real_publish(message)
            before=health(rx)['complete_sample_count']
            rx.sample_pub.publish=delayed_publish
            step(4.)
            rx.sample_pub.publish=real_publish
            assert blocked[0] and health(rx)['complete_sample_count']>=before+6
            assert health(rx)['io_tick_max_gap_sec']<.65
            assert health(tx)['wall_durations']['zero_write']['count']>0
            step(2.)
            assert health(tx)['armed']
            os.close(masters[-1]);masters[-1]=None;path.unlink()
            step(1.,False)
            assert not health(tx).get('armed') and not health(rx).get('connected')
            master,slave=pty.openpty();tty.setraw(slave);masters.append(master);slaves.append(slave)
            os.set_blocking(master,False);path.symlink_to(os.ttyname(slave))
            step(7.)
            assert health(tx)['armed'] and health(rx)['session']!=first
            assert health(rx)['open_count']>=2 and health(tx)['open_count']>=2
            assert odom.atomic_wheels.session==health(rx)['session']==manager.atomic_wheels.session
            assert abs(odom.atomic_wheels.sequence-manager.atomic_wheels.sequence)<=1
            assert observed and all(line+b'\n'==ZERO for line in observed.splitlines())
            print(json.dumps(dict(real_pty_reconnect=True,real_ros_feedback_gate=True,
                 diagnostic_process_stall_sec=3.,control_publish_stall_sec=2.,
                 separate_control_feedback_verified=True,
                 diagnostic_health_muted_sec=2.,regular_feedback_max_age_sec=regular_feedback_max_age,
                 worker_resource_usage=resource_usage,complete_groups=frames,rx_opens=health(rx)['open_count'],tx_opens=health(tx)['open_count'],
                 rx_io_max_gap_sec=health(rx)['io_tick_max_gap_sec'],
                 tx_io_max_gap_sec=health(tx)['io_tick_max_gap_sec'],
                 accepted_zero_commands=health(tx)['accepted_commands'],atomic_odom_and_manager=True,
                 nonzero_hardware_commands=0,goals=0)))
        finally:
            feed_stop.set();feeder.join(1.)
            for owner in (rx,tx):
                if owner.workers[1].is_alive():os.kill(owner.workers[1].pid,signal.SIGCONT)
            executor.shutdown()
            manager.pose_cache_writer.close()
            manager.destroy_node();odom.destroy_node();tx.destroy_node();rx.destroy_node();probe.destroy_node();rclpy.shutdown()
            for fd in masters+slaves:
                if fd is not None:
                    try:os.close(fd)
                    except OSError:pass


if __name__=='__main__':main()
