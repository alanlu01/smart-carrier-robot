"""Bounded asynchronous container output capture; never controls lifecycle."""
from collections import deque
import atexit
import json
from pathlib import Path
import threading
import time


class NavigationTranscript:
    MAX_PENDING = 2*1024*1024

    def __init__(self, directory, multi_process=False, action_health_file=None):
        self.directory=Path(directory)
        self.action_health_file = Path(action_health_file) if action_health_file else None
        self.action_sequence = 0
        self.fragments = {}
        self.multi_process = multi_process
        if multi_process:
            atexit.register(self.close)
        self.condition=threading.Condition()
        self.pending=deque()
        self.pending_bytes=self.dropped_bytes=0
        self.error=''
        self.closed=False
        self.thread=threading.Thread(target=self._run,name='nav2-transcript',daemon=True)
        self.thread.start()

    @staticmethod
    def matches(event):
        return (event.cmd and (
            Path(event.cmd[0]).name in ('component_container','component_container_isolated')
            and '__node:=nav2_container' in event.cmd
            or Path(event.cmd[0]).name in ('map_server','amcl','controller_server','planner_server',
                'smoother_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother',
                'collision_monitor','docking_server','opennav_docking','route_server','lifecycle_manager')))

    @staticmethod
    def prefix(event):
        name=Path(event.cmd[0]).name
        return 'nav2_container' if name.startswith('component_container') else 'nav2_'+name

    def _put(self, suffix, data):
        with self.condition:
            if self.closed or self.pending_bytes+len(data)>self.MAX_PENDING:
                self.dropped_bytes+=len(data)
                return
            self.pending.append((suffix,data))
            self.pending_bytes+=len(data)
            self.condition.notify()

    def output(self,event):
        if self.matches(event):
            stream='stderr' if event.from_stderr else 'stdout'
            self._put(f'{self.prefix(event)}_{event.pid}.{stream}.log',bytes(event.text))

    def exited(self,event,_context=None):
        if self.matches(event):
            with self.condition:
                payload=dict(pid=event.pid,returncode=event.returncode,wall_time=time.time(),
                             dropped_bytes=self.dropped_bytes,write_error=self.error)
            self._put(f'{self.prefix(event)}_{event.pid}.exit.json',json.dumps(payload).encode()+b'\n')
            if not self.multi_process:
                self.close()

    def close(self):
        with self.condition:
            self.closed=True
            self.condition.notify()
        self.thread.join(timeout=.5)

    def _run(self):
        streams = {}
        try:
            self._write_pending(streams)
        finally:
            for stream in streams.values():
                try:
                    stream.close()
                except OSError as exc:
                    with self.condition:
                        self.error = str(exc)[:300]

    def _write_pending(self, streams):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or self.pending)
                if not self.pending:
                    return
                suffix,data=self.pending.popleft()
                self.pending_bytes-=len(data)
            try:
                if self.action_health_file is not None and suffix.endswith(('.stderr.log','.stdout.log')):
                    self._observe_action(suffix,data)
                if suffix.endswith('.exit.json'):
                    payload=json.loads(data)
                    with self.condition:
                        payload.update(dropped_bytes=self.dropped_bytes,write_error=self.error)
                    data=json.dumps(payload).encode()+b'\n'
                self.directory.mkdir(parents=True,exist_ok=True)
                if suffix not in streams:
                    streams[suffix] = (self.directory/suffix).open('ab')
                streams[suffix].write(data)
                streams[suffix].flush()
            except OSError as exc:
                with self.condition:
                    self.error=str(exc)[:300]
                    self.dropped_bytes+=len(data)

    def _observe_action(self, suffix, data):
        text = self.fragments.get(suffix,'')+data.decode(errors='replace')
        lines = text.split('\n')
        self.fragments[suffix] = lines.pop()[-8192:]
        for line in lines:
            if ('Timed out while waiting for action server to acknowledge goal request' not in line
                    and 'Failed to cancel action server for' not in line
                    and 'Delivery policy did not match the stamped goal' not in line):
                continue
            self.action_sequence += 1
            payload=dict(sequence=self.action_sequence,wall_time=time.time(),
                         reason=line[-1000:],source=suffix)
            self.action_health_file.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.action_health_file.with_suffix('.tmp')
            temporary.write_text(json.dumps(payload))
            temporary.replace(self.action_health_file)
