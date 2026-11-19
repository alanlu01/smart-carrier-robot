"""Separate bounded moving, stopped-clearance and absolute station budgets."""
import math


class TranslationBudget:
    def __init__(self, now, sequence_deadline, moving=25., waiting=30.):
        self.moving, self.waiting = moving, waiting
        self.absolute = min(sequence_deadline, now+moving+waiting+5.)
        self.last, self.phase = now, 'moving'

    def transition(self, now, phase):
        if not math.isfinite(now) or now < self.last:
            self.moving = self.waiting = 0.
        else:
            elapsed = now-self.last
            if self.phase == 'moving':
                self.moving = max(0., self.moving-elapsed)
            else:
                self.waiting = max(0., self.waiting-elapsed)
        self.last, self.phase = now, phase

    def deadline(self, now):
        self.transition(now, self.phase)
        if self.waiting<=0.:
            return min(self.absolute,now)
        return min(self.absolute, now+(self.moving if self.phase == 'moving' else self.waiting))

    def snapshot(self, now):
        self.transition(now, self.phase)
        return dict(phase=self.phase, moving_remaining_sec=round(self.moving,3),
                    waiting_remaining_sec=round(self.waiting,3),
                    absolute_remaining_sec=round(max(0.,self.absolute-now),3))
