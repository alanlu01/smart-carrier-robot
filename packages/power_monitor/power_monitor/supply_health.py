"""Charging-bus health is independent of I2C health and chassis home policy."""
import math


class SupplyHealth:
    def __init__(self, minimum=9.5, recovered=9.8, recovery_sec=5., fault_sec=2.):
        if (not all(math.isfinite(v) for v in (minimum, recovered, recovery_sec, fault_sec))
                or not 0 < minimum < recovered or min(recovery_sec,fault_sec)<=0):
            raise ValueError('Invalid charging supply thresholds')
        self.minimum,self.recovered=minimum,recovered
        self.recovery_sec,self.fault_sec=recovery_sec,fault_sec
        self.healthy=False;self.seen=False;self.bad_since=self.good_since=None
        self.last_at=None
        self.communication_since=None
        self.communication_samples=0
        self.communication_eligible=False

    def observe(self, voltages, now, sensor_ok=True, enabled=(True,True,True)):
        if self.last_at is not None and not 0 <= now-self.last_at <= 2.:
            self.healthy=False;self.good_since=self.bad_since=None
            self.communication_eligible=False
        advanced=self.last_at is None or now>self.last_at
        self.last_at=now
        values=[(i+1,v) for i,(v,on) in enumerate(zip(voltages,enabled)) if on]
        valid=(sensor_ok and len(voltages)==len(enabled)==3
               and all(v is not None and math.isfinite(v) and v>=0 for _,v in values))
        failed=[i for i,v in values if v is not None and math.isfinite(v) and v<self.minimum]
        if not valid:
            if self.healthy:
                self.communication_since=now
                self.communication_eligible=True
            if (self.communication_since is None or now-self.communication_since>2.):
                self.communication_eligible=False
            self.communication_samples=0
            self.healthy=False;self.bad_since=self.good_since=None;state='sensor_unknown'
        elif failed:
            self.communication_eligible=False;self.communication_samples=0
            self.healthy=False;self.good_since=None
            self.bad_since=now if self.bad_since is None else self.bad_since
            state='supply_fault' if now-self.bad_since>=self.fault_sec else 'supply_low'
        elif not self.seen:
            self.healthy=True;self.bad_since=None;state='healthy'
        elif not self.healthy:
            self.bad_since=None
            if all(v>=self.recovered for _,v in values):
                self.good_since=now if self.good_since is None else self.good_since
                self.communication_samples+=int(advanced)
                self.healthy=(self.communication_samples>=3 if self.communication_eligible
                              else now-self.good_since>=self.recovery_sec)
            else:
                self.good_since=None;self.communication_samples=0
                self.communication_eligible=False
            state='healthy' if self.healthy else 'recovering'
        else:
            self.bad_since=self.good_since=None;state='healthy'
        self.seen=True
        return dict(healthy=self.healthy,state=state,fault_confirmed=state=='supply_fault',
                    failed_channels=failed,minimum_voltage_v=self.minimum,recovery_voltage_v=self.recovered,
                    recovery_mode='communication' if self.communication_eligible and not self.healthy else 'supply',
                    valid_recovery_samples=self.communication_samples)


def supply_failure(payload):
    """Read optional new metadata; legacy payloads remain supported."""
    health=payload.get('slot_supply')
    if not isinstance(health,dict) or health.get('healthy') is True:return ''
    if health.get('fault_confirmed') is True:return '槽位供電不足，無法確認實體取還，任務已安全停止'
    return '槽位供電／感測更新中，暫停租還確認'
