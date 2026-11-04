"""Bounded latest-value IPC. Neither the I/O owner nor a reader waits on a lock."""
import json


class LatestMailbox:
    def __init__(self, context, capacity=65536):
        self.capacity = capacity
        self.buffer = context.RawArray('B', capacity)
        self.length = context.RawValue('I', 0)
        self.version = context.RawValue('Q', 0)
        self.skipped = context.RawValue('Q', 0)
        self.lock = context.Lock()

    def put(self, value):
        encoded = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
        if len(encoded) > self.capacity:
            raise ValueError('serial IPC snapshot exceeds bounded capacity')
        if not self.lock.acquire(False):
            self.skipped.value += 1
            return False
        try:
            memoryview(self.buffer).cast('B')[:len(encoded)] = encoded
            self.length.value = len(encoded)
            self.version.value += 1
        finally:
            self.lock.release()
        return True

    def get(self, after=None):
        if not self.lock.acquire(False):
            return None
        try:
            version = self.version.value
            if not version or after == version:
                return None
            encoded = bytes(memoryview(self.buffer).cast('B')[:self.length.value])
        finally:
            self.lock.release()
        return version, json.loads(encoded)
