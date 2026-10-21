"""Host/domain-scoped launch lease; never kills or activates existing nodes."""

import fcntl
import json
import os
from pathlib import Path


class NavigationLaunchLease:
    """Hold an advisory lock for the launch lifetime and detect unguarded Nav2."""

    def __init__(self, directory=None, proc_root='/proc', domain=None):
        self.directory = Path(directory or (Path.home()/'.ros/smart_carrier_launch'))
        self.proc_root = Path(proc_root)
        self.domain = str(os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else domain)
        if not self.domain.isdecimal() or not 0 <= int(self.domain) <= 232:
            raise RuntimeError('Invalid ROS_DOMAIN_ID; refusing ambiguous navigation launch')
        self.stream = None

    def existing_processes(self):
        """Inspect only command lines and domain; do not log process environments."""
        found = []
        for item in self.proc_root.iterdir():
            if not item.name.isdecimal() or int(item.name) == os.getpid():
                continue
            try:
                words = item.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
                executable = Path(words[0]).name
                is_container = (executable.startswith('component_container')
                                and any('__node:=nav2_container' in word for word in words))
                is_server = executable in ('bt_navigator', 'controller_server', 'amcl', 'map_server')
                if not (is_container or is_server):
                    continue
                environment = item.joinpath('environ').read_bytes().split(b'\0')
                domain = next((entry.split(b'=', 1)[1].decode() for entry in environment
                               if entry.startswith(b'ROS_DOMAIN_ID=')), '0')
                if domain == self.domain:
                    found.append({'pid': int(item.name), 'executable': executable})
            except FileNotFoundError:
                continue  # Process exited during inspection.
            except PermissionError as exc:
                raise RuntimeError(f'Cannot inspect existing Nav2 PID {item.name}') from exc
        return found

    def acquire(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory/f'navigation_domain_{self.domain}.lock'
        self.stream = path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.stream.seek(0)
            owner = self.stream.read(2048)
            self.close()
            raise RuntimeError('start_nav 已在執行，拒絕重複啟動；現有啟動資訊：'+owner) from exc
        try:
            found = self.existing_processes()
            if found:
                raise RuntimeError('發現尚未結束的 Nav2，未啟動新程序、未強制關閉：'+json.dumps(found))
            self.stream.seek(0)
            self.stream.truncate()
            json.dump({'pid': os.getpid(), 'domain': self.domain}, self.stream)
            self.stream.flush()
        except Exception:
            self.close()
            raise
        return self

    def close(self):
        if self.stream is not None:
            # Never unlink a flock file: another launcher may already have it open.
            self.stream.close()
            self.stream = None
