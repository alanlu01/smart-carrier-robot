"""One-shot startup reconciliation, independent of later cloud result sync."""

from dataclasses import dataclass


@dataclass
class StartupDispatchGate:
    """Hold initial parking until work is received or a fresh empty is known."""

    confirmed: bool = False

    def observe(self, *, has_work, sync_state):
        """Latch once; a later outage or pending cloud ACK cannot close it."""
        if has_work or sync_state == 'empty':
            self.confirmed = True

    @property
    def waiting(self):
        """Whether initial reconciliation still prevents a parking goal."""
        return not self.confirmed
