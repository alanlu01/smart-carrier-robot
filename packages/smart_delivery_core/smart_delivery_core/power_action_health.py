"""Invalidate action evidence on every bad sample, including legacy payloads."""
import json
import math
from power_monitor.power_status import payload_to_slots
from power_monitor.supply_health import supply_failure


class PowerActionHealth:
    def __init__(self):
        self.valid = False
        self.bad_since = None
        self.received_at = None
        self.note = ''
        self.confirmed_fault = False
        self.recovery_expected = False

    def observe(self, text, now):
        self.received_at = now
        self.note = ''
        self.confirmed_fault = False
        try:
            payload = json.loads(text)
            self.note = supply_failure(payload)
            supply = payload.get('slot_supply', {})
            self.confirmed_fault = supply.get('fault_confirmed') is True
            if supply.get('state') == 'recovering':
                self.recovery_expected = True
            elif supply.get('state') != 'healthy':
                self.recovery_expected = False
            slots = payload_to_slots(text)
            # Uncertain presence in a different slot must not block travel to
            # service a confirmed slot. Electrical validity remains global;
            # SlotActionVerifier still checks physical action evidence.
            if any(slot.get('enabled',True) and (not slot.get('sensor_ok',True)
                    or slot.get('current') is None or not math.isfinite(slot['current'])
                    or (slot.get('voltage') is not None and not math.isfinite(slot['voltage'])))
                   for slot in slots):
                raise ValueError('槽位感測資料無效')
            if not any(slot.get('status') not in {'unknown','disabled'} for slot in slots):
                raise ValueError('尚無已確認槽位')
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
        self.recovery_expected = False
        return slots

    @staticmethod
    def action_ready(slots):
        # Navigation can use known inventory while other presence is uncertain.
        # Physical wrong-slot verification needs a trustworthy baseline first.
        return all(not slot.get('enabled',True) or
                   (slot.get('sensor_ok',True) and slot.get('status') not in {'unknown','disabled'})
                   for slot in slots)

    def failure(self, now):
        if self.valid or self.received_at is None or not 0 <= now-self.received_at <= 2.:
            return ''
        if self.confirmed_fault:
            return '槽位供電不足，無法確認實體取還，任務已安全停止'
        # Allow the monitor's 5 s recovery plus slot debounce to finish. This
        # remains bounded; fresh but perpetually recovering data cannot wait forever.
        limit = 10. if self.recovery_expected else 5.
        if self.bad_since is not None and now-self.bad_since >= limit:
            return '槽位供電／感測持續無效，無法確認實體取還'
        return ''
