# /// script
# dependencies = ["rosbags>=0.10", "mcap>=1.2", "numpy"]
# ///
"""Decodes every message of a ROS 2 MCAP with rosbags, from the definitions embedded in the file, and compares it with
what the recorder says it wrote (`<file>.expected.json` next to it, from examples/reference_fixture.rs)."""

import json
import math
import sys

import numpy
from mcap.reader import make_reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def plain(value):
    if hasattr(value, "__dataclass_fields__"):
        # rosbags exposes a message's constants as fields too
        return {name: plain(getattr(value, name)) for name in value.__dataclass_fields__ if name not in ("__msgtype__", "structure_needs_at_least_one_member") and not name.isupper()}
    if isinstance(value, numpy.ndarray):
        return [plain(item) for item in value.tolist()]
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, numpy.generic):
        return value.item()
    return value


def same(actual, expected, path):
    if isinstance(expected, dict):
        assert isinstance(actual, dict) and set(actual) == set(expected), f"{path}: fields {sorted(actual)} != {sorted(expected)}"
        for key in expected:
            same(actual[key], expected[key], f"{path}.{key}")
    elif isinstance(expected, list):
        assert len(actual) == len(expected), f"{path}: {len(actual)} items, expected {len(expected)}"
        for index, (left, right) in enumerate(zip(actual, expected)):
            same(left, right, f"{path}[{index}]")
    elif isinstance(expected, float):
        assert math.isclose(float(actual), expected, rel_tol=1e-12, abs_tol=1e-12), f"{path}: {actual} != {expected}"
    elif isinstance(expected, bool):
        assert bool(actual) == expected, f"{path}: {actual} != {expected}"
    elif isinstance(actual, int) and -128 <= actual < 0 and actual + 256 == expected:
        pass  # rosbags reads ROS 2 `byte` as int8; the octet on the wire is the same
    else:
        assert actual == expected, f"{path}: {actual!r} != {expected!r}"


path = sys.argv[1]
expected = json.load(open(path.removesuffix(".mcap") + ".expected.json"))
typestore = get_typestore(Stores.EMPTY)
seen = {}
with open(path, "rb") as stream:
    reader = make_reader(stream)
    assert reader.get_header().profile == "ros2", reader.get_header().profile
    for schema, channel, message in reader.iter_messages():
        assert channel.message_encoding == "cdr" and schema.encoding == "ros2msg", (channel.topic, channel.message_encoding)
        if schema.name not in typestore.fielddefs:
            typestore.register(get_types_from_msg(schema.data.decode(), schema.name))
        decoded = plain(typestore.deserialize_cdr(message.data, schema.name))
        index = seen.get(channel.topic, 0)
        seen[channel.topic] = index + 1
        want = expected[channel.topic]
        assert schema.name == want["schema"], (schema.name, want["schema"])
        same(decoded, want["messages"][index], f"{channel.topic}#{index}")
for topic, want in expected.items():
    assert seen.get(topic, 0) == len(want["messages"]), f"{topic}: read {seen.get(topic, 0)} of {len(want['messages'])}"
print(f"rosbags decoded {sum(seen.values())} messages on {len(seen)} topics, all as recorded")
