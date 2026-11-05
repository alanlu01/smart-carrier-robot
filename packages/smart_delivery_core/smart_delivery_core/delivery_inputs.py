"""Single subscription per delivery input; fan out without re-decoding JSON."""
import json
from types import SimpleNamespace
from std_msgs.msg import String


def message_json(message):
    return message.parsed if hasattr(message, 'parsed') else json.loads(message.data)


class SharedInputs:
    def __init__(self, node):
        self.node, self.handlers, self.subscriptions = node, {}, {}

    def subscribe(self, kind, topic, callback, qos):
        key = '/'+topic.strip('/')
        if key not in self.handlers:
            self.handlers[key] = []
            def receive(message):
                if kind is String:
                    try:message = SimpleNamespace(data=message.data, parsed=json.loads(message.data))
                    except (ValueError,TypeError):pass
                for handler in tuple(self.handlers[key]):handler(message)
            self.subscriptions[key] = self.node.create_subscription(kind,topic,receive,qos)
        self.handlers[key].append(callback)


def shared_inputs(node):
    if not hasattr(node, '_delivery_inputs'):
        node._delivery_inputs = SharedInputs(node)
    return node._delivery_inputs


class PolicyCadence:
    """Throttle policy work, not subscriptions, safety timers or urgent events."""
    def __init__(self, period=.1):
        self.period, self.next_at, self.urgent = period, 0., True

    def wake(self):self.urgent = True

    def due(self, now):
        if not self.urgent and now < self.next_at:return False
        self.urgent = False
        self.next_at = now+self.period
        return True
