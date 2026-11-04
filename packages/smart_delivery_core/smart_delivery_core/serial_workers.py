"""Process boundaries for hardware I/O, control DDS, and diagnostic DDS."""
import ctypes
import json
import os
import signal
import time

import serial

from smart_delivery_core.serial_io import SerialReceiver, SerialTransmitter, ZERO


def protect_parent(stop, name):
    """Leave no serial writer running after an abruptly killed ROS parent."""
    parent = os.getppid()
    libc = ctypes.CDLL(None)
    libc.prctl(15, name.encode(), 0, 0, 0)
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    libc.prctl(1, signal.SIGTERM, 0, 0, 0)
    if os.getppid() != parent or parent == 1:
        stop.set()


def receive_worker(path, baud, stop, wheel, health, raw):
    protect_parent(stop, 'serial_rx_io')
    engine = SerialReceiver(path, baud)
    last_health = last_raw = 0.
    try:
        while not stop.is_set():
            started = time.monotonic()
            try:
                latest = engine.step(started)
            except (OSError, serial.SerialException) as exc:
                engine.link.fail(exc, time.monotonic())
                engine.pending.clear()
                engine.complete_ros_ns = 0
                latest = None
            if latest is not None:
                with engine.metrics.measure('wheel_ipc'):
                    wheel.put(latest)
            now = time.monotonic()
            if latest is not None or now-last_health >= .2:
                health.put(engine.health(now))
                last_health = now
            if engine.frames != last_raw:
                raw.put(dict(session=engine.session, frames=list(engine.raw)))
                last_raw = engine.frames
            stop.wait(max(0., .05-(time.monotonic()-started)))
    finally:
        engine.link.close()


def transmit_worker(path, baud, stop, commands, feedback, health):
    protect_parent(stop, 'serial_tx_io')
    engine = SerialTransmitter(path, baud)
    last_command = last_feedback = last_health = 0
    try:
        while not stop.is_set():
            started = time.monotonic()
            item = feedback.get(last_feedback)
            if item:
                last_feedback, p = item
                engine.feedback, engine.feedback_at = p['payload'], p['received_at']
            engine.poll(started)
            item = commands.get(last_command)
            if item:
                last_command, payload = item
                engine.command(payload)  # Consumed even if rejected; never replay on rearm.
            if started-last_health >= .2:
                health.put(engine.health(time.monotonic()))
                last_health = started
            stop.wait(max(0., .05-(time.monotonic()-started)))
    finally:
        engine.link.write(ZERO, time.monotonic())
        engine.link.close()


def diagnostic_worker(role, stop, health, relay_metrics, raw=None, summary_period=5.):
    """A blocked reliable diagnostic publisher cannot block either I/O owner."""
    protect_parent(stop, 'serial_'+role+'_diag')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    from smart_delivery_core.pipeline_health import DurationMetrics

    rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('serial_'+role+'_diagnostics')
    diagnostic = node.create_publisher(String, '/chassis/serial_io_diagnostics', 1)
    raw_pub = node.create_publisher(String, '/stm32_data', 10) if role == 'rx' else None
    health_pub = node.create_publisher(String, '/chassis/serial_tx_health', 1) if role == 'tx' else None
    metrics = DurationMetrics()
    raw_version = last_sequence = log_sequence = 0
    raw_session = None
    dropped = 0
    last_diag = last_summary = 0.
    previous_frames = 0
    try:
        while rclpy.ok() and not stop.is_set():
            started = time.monotonic()
            captured = health.get()
            h = captured[1] if captured else {}
            if role == 'rx':
                item = raw.get(raw_version)
                if item:
                    raw_version, batch = item
                    if raw_session != batch['session']:
                        raw_session = batch['session']
                    for frame in batch['frames']:
                        if frame['sequence'] <= last_sequence:
                            continue
                        dropped += max(0, frame['sequence']-last_sequence-1)
                        last_sequence = frame['sequence']
                        if not -.1 <= (time.time_ns()-frame['stamp_ns'])/1e9 <= 1.:
                            dropped += 1
                            continue
                        with metrics.measure('raw_publish'):
                            raw_pub.publish(String(data=frame['text']))
                if started-last_summary >= summary_period and h:
                    frames = h.get('frames', 0)
                    rate = max(0, frames-previous_frames)/max(.001, started-last_summary) if last_summary else 0.
                    node.get_logger().info(f'📡 STM32 回授：{rate:.1f} Hz')
                    previous_frames, last_summary = frames, started
            elif h:
                with metrics.measure('tx_health_publish'):
                    health_pub.publish(String(data=json.dumps(h)))  # Retain capture time.
            for seq, level, text in h.get('logs', []):
                if seq > log_sequence:
                    log_sequence = seq
                    if level == 'warning':
                        node.get_logger().warning(text)
                    else:
                        node.get_logger().info(text)
            if started-last_diag >= 1.:
                relay = relay_metrics.get()
                payload = dict(role=role, sample_ros_stamp_ns=time.time_ns(),
                               io=h, relay=relay[1] if relay else {},
                               diagnostics=metrics.snapshot(time.monotonic()), raw_dropped=dropped,
                               ipc_write_skips=dict(health=health.skipped.value, relay=relay_metrics.skipped.value))
                diagnostic.publish(String(data=json.dumps(payload)))
                last_diag = started
            stop.wait(max(0., .2-(time.monotonic()-started)))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def stop_workers(stop, workers):
    stop.set()
    for worker in workers:
        worker.join(2.)
    for worker in workers:
        if worker.is_alive():
            worker.terminate()
            worker.join(1.)
        if worker.is_alive():
            worker.kill()
            worker.join(1.)
