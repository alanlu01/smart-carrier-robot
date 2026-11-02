"""Invalidate action evidence on every bad sample, including legacy payloads."""
import json
from power_monitor.power_status import payload_to_slots
from power_monitor.supply_health import supply_failure


class PowerActionHealth:
    def __init__(self):
        self.valid = False
        self.bad_since = None
        self.received_at = None
        self.note = ''
        self.confirmed_fault = False

    def observe(self, text, now):
        self.received_at = now
        self.note = ''
        self.confirmed_fault = False
        try:
            payload = json.loads(text)
            self.note = supply_failure(payload)
            supply = payload.get('slot_supply', {})
            self.confirmed_fault = supply.get('fault_confirmed') is True
            slots = payload_to_slots(text, require_healthy=True)
            if self.note:
                raise ValueError(self.note)
        except (ValueError, TypeError, AttributeError) as exc:
            self.valid = False
            self.bad_since = now if self.bad_since is None else self.bad_since
            self.note = self.note or '槽位感測無效，暫停租還確認'
            raise ValueError(self.note) from exc
        self.valid = True
        self.bad_since = None
        self.confirmed_fault = False
        return slots

    def failure(self, now):
        if self.valid or self.received_at is None or not 0 <= now-self.received_at <= 2.:
            return ''
        if self.confirmed_fault:
            return '槽位供電不足，無法確認實體取還，任務已安全停止'
        if self.bad_since is not None and now-self.bad_since >= 5.:
            return '槽位供電／感測持續無效，無法確認實體取還'
        return ''
