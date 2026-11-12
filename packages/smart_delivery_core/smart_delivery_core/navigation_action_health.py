"""Bound navigation-only lifecycle recovery after a diagnosed action fault."""
import json
import uuid
from pathlib import Path


class ActionRecovery:
    """No reset is allowed without an explicit request and fresh stop proof."""
    def __init__(self, path):
        self.path = Path(path) if path else None
        self.session = uuid.uuid4().hex
        self.sequence = self.recovered_generation = self.attempts = 0
        self.state, self.reason = 'healthy', ''
        self.requested = False
        self.deadline = self.stop_since = None
        self.last_stop_check = None
        self.future = None
        self.observed_size = None
        self.fault_wall_time = None

    def observe(self):
        if self.path is None:
            return
        try:
            stat=self.path.stat(); signature=(stat.st_mtime_ns,stat.st_size)
            if signature==self.observed_size:return
            payload=json.loads(self.path.read_text())
            sequence=int(payload['sequence'])
            self.observed_size=signature
            if sequence<=self.sequence:return
            self.sequence=sequence
            self.fault_wall_time=float(payload.get('wall_time',0.))
            if self.state not in ('waiting_stop','resetting','waiting_start','starting','verifying','manual_required'):
                self.state='fault'; self.requested=False
            self.reason=str(payload['reason'])[:1000]
        except FileNotFoundError:
            pass
        except (OSError,ValueError,KeyError,TypeError):
            self.state,self.reason='manual_required','action_diagnostic_unreadable'

    def request(self, now):
        if self.state=='fault' and self.attempts>=2:
            self.state,self.reason='manual_required','navigation_recovery_limit_reached'
        if self.state!='fault':
            return False
        self.requested=True
        self.deadline=now+10.
        self.state,self.reason='waiting_stop','waiting_fresh_physical_stop'
        self.stop_since=None
        self.last_stop_check=None
        return True

    def tick(self, now, *, stopped, lifecycle_ready, service_ready, call):
        self.observe()
        if (self.state in ('waiting_stop','resetting','waiting_start','starting','verifying')
                and now>=self.deadline):
            self.state,self.reason='manual_required','navigation_recovery_phase_timeout'
            return
        if self.state=='waiting_stop':
            if self.last_stop_check is not None and now-self.last_stop_check>.35:
                self.stop_since=None
            self.last_stop_check=now
            if not stopped:self.stop_since=None
            elif self.stop_since is None:self.stop_since=now
            if self.stop_since is not None and now-self.stop_since>=1. and service_ready:
                self.attempts+=1
                self._call(call,3,now,'resetting','navigation_reset',10.)
            elif now>=self.deadline:
                self.state,self.reason='manual_required','stop_or_service_not_verified'
        elif self.state in ('resetting','starting'):
            if self.future.done():
                try:success=self.future.result().success
                except Exception:success=False
                if not success:
                    self.state,self.reason='manual_required','navigation_lifecycle_request_failed'
                elif self.state=='resetting':
                    self.state,self.reason,self.deadline='waiting_start','navigation_stopped_verify',now+5.
                else:
                    self.state,self.reason,self.deadline='verifying','navigation_verify_active',now+5.
                    self.verification_since=now
            elif now>=self.deadline:
                # A timed-out lifecycle RPC may still execute. Never send a new
                # startup/reset or declare healthy without its response.
                self.state,self.reason='manual_required','navigation_lifecycle_response_timeout'
        elif self.state=='waiting_start':
            # Stop proof must also remain valid AFTER RESET. Reconfiguration
            # never starts while another Spin or an external goal is moving.
            if stopped and service_ready:
                self._call(call,0,now,'starting','navigation_startup',15.)
            elif now>=self.deadline:
                self.state,self.reason='manual_required','stop_not_verified_after_reset'
        elif self.state=='verifying':
            if lifecycle_ready and stopped:
                self.recovered_generation+=1
                self.state,self.reason='healthy','navigation_recovered'
                self.requested=False
            elif now>=self.deadline:
                self.state,self.reason='manual_required','navigation_recovery_not_verified'

    def _call(self, call, command, now, state, reason, timeout):
        try:
            self.future=call(command)
            self.state,self.reason,self.deadline=state,reason,now+timeout
        except Exception:
            self.state,self.reason='manual_required','navigation_lifecycle_request_exception'

    def snapshot(self):
        return dict(state=self.state,reason=self.reason,session=self.session,
            fault_sequence=self.sequence,recovered_generation=self.recovered_generation,
            recovery_attempts=self.attempts,recovery_limit=2)
