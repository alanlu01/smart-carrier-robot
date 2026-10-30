"""Bounded asynchronous container output capture; never controls lifecycle."""
from collections import deque
import json
from pathlib import Path
import threading
import time


class NavigationTranscript:
    MAX_PENDING = 2*1024*1024

    def __init__(self, directory):
        self.directory=Path(directory)
        self.condition=threading.Condition()
        self.pending=deque()
        self.pending_bytes=self.dropped_bytes=0
        self.error=''
        self.closed=False
        self.thread=threading.Thread(target=self._run,name='nav2-transcript',daemon=True)
        self.thread.start()

    @staticmethod
    def matches(event):
        return (event.cmd and Path(event.cmd[0]).name in ('component_container','component_container_isolated')
                and '__node:=nav2_container' in event.cmd)

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
            self._put(f'nav2_container_{event.pid}.{stream}.log',bytes(event.text))

    def exited(self,event,_context=None):
        if self.matches(event):
            with self.condition:
                payload=dict(pid=event.pid,returncode=event.returncode,wall_time=time.time(),
                             dropped_bytes=self.dropped_bytes,write_error=self.error)
            self._put(f'nav2_container_{event.pid}.exit.json',json.dumps(payload).encode()+b'\n')
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
