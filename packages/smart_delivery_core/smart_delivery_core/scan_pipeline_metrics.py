"""Bounded scan timing evidence, independent of the navigation/control path."""

from collections import OrderedDict

from smart_delivery_core.pipeline_health import ReceiptMetrics


class ScanPipelineMetrics:
    """Separate acquisition, DDS writer/RMW receive and callback timestamps."""

    def __init__(self):
        self.receipts = ReceiptMetrics()
        self.pending = OrderedDict()
        self.timings = {}
        self.last = {}
        self.matched = self.evicted = self.invalid_stamps = 0

    def _timing(self, name, value, now):
        if value is None:
            return
        row = self.timings.setdefault(name, {'count': 0, 'last_sec': value,
            'min_sec': value, 'max_sec': value, 'peak_at': now})
        row['count'] += 1
        row['last_sec'] = value
        row['min_sec'] = min(row['min_sec'], value)
        if value > row['max_sec']:
            row['max_sec'], row['peak_at'] = value, now

    def observe(self, name, stamp_ns, now, ros_ns, source_ns=0, received_ns=0):
        """Never substitute a scan header for unavailable DDS timestamps."""
        if stamp_ns <= 0:
            self.invalid_stamps += 1
            return
        age = (ros_ns-stamp_ns)/1e9
        self.receipts.observe(name, now, age)
        source_valid = source_ns > 0 and -.1 <= (ros_ns-source_ns)/1e9 <= 60.
        received_valid = received_ns > 0 and -.1 <= (ros_ns-received_ns)/1e9 <= 60.
        fields = {
            'scan_source_age_sec': age,
            'writer_to_callback_sec': (ros_ns-source_ns)/1e9 if source_valid else None,
            'rmw_to_callback_sec': (ros_ns-received_ns)/1e9 if received_valid else None,
            'writer_to_rmw_sec': (received_ns-source_ns)/1e9
                if source_valid and received_valid else None,
            'acquisition_to_writer_sec': (source_ns-stamp_ns)/1e9 if source_valid else None,
        }
        self.last[name] = {**fields, 'dds_source_timestamp_available': source_valid,
                          'rmw_receive_timestamp_available': received_valid}
        for key, value in fields.items():
            self._timing(name+'.'+key, value, now)
        pair = self.pending.setdefault(stamp_ns, {})
        pair.setdefault(name, (now, source_ns if source_valid else None,
                              received_ns if received_valid else None))
        if 'raw' in pair and 'filtered' in pair:
            raw, filtered = pair['raw'], pair['filtered']
            self.matched += 1
            self._timing('pair.callback_skew_sec', filtered[0]-raw[0], now)
            for index, key in ((1, 'writer_pipeline_sec'), (2, 'rmw_receive_skew_sec')):
                value = None if raw[index] is None or filtered[index] is None else (
                    filtered[index]-raw[index])/1e9
                self._timing('pair.'+key, value, now)
            del self.pending[stamp_ns]
        while len(self.pending) > 128:
            self.pending.popitem(last=False)
            self.evicted += 1

    def snapshot(self, now):
        """Do not call end-to-end writer timing filter CPU/compute duration."""
        streams = self.receipts.snapshot(now)
        raw_fresh = streams.get('raw', {}).get('receive_age_sec', 1e9) <= .30
        filtered_fresh = streams.get('filtered', {}).get('receive_age_sec', 1e9) <= .30
        state = ('healthy' if raw_fresh and filtered_fresh else 'filtered_receive_gap'
                 if raw_fresh else 'raw_and_or_filtered_receive_gap')
        if state == 'healthy':
            for name in ('raw', 'filtered'):
                stream = streams[name]
                age = stream['receive_age_sec']+stream['source_age_at_receive_sec']
                if age < -.1:
                    state = 'source_clock_invalid'
                    break
                if age > .30:
                    state = name+'_source_stale'
                    break
        return {'diagnostic_only': True, 'state': state, 'receipts': streams,
                'last': {name: {k: round(v, 6) if isinstance(v, float) else v
                                for k, v in row.items()} for name, row in self.last.items()},
                'timings': {name: {k: round(v, 6) if isinstance(v, float) else v
                                  for k, v in row.items() if k != 'peak_at'} |
                            {'peak_age_sec': round(max(0., now-row['peak_at']), 3)}
                            for name, row in self.timings.items()},
                'matched_pairs': self.matched, 'pending_pairs': len(self.pending),
                'evicted_pairs': self.evicted, 'invalid_stamps': self.invalid_stamps,
                'writer_pipeline_definition': 'filtered DDS source minus same-stamp raw DDS source; includes input transport/TF/filter path, NOT filter CPU or publish-call duration'}
