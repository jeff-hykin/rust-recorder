"""Opens the recorder's dimos-layout files with dimos' own stores (run with a dimos checkout's Python on the path) and
checks every stream decodes with the expected types, counts and source times (`<file>.expected.json`)."""

import json
import math
import sys

from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image


def check(store, expected, label):
    names = set(store.list_streams())
    assert names == set(expected), f"{label}: streams {sorted(names)} != {sorted(expected)}"
    for name, want in expected.items():
        observations = store.stream(name).to_list()
        assert len(observations) == len(want["ts"]), f"{label} {name}: {len(observations)} observations, expected {len(want['ts'])}"
        for observation, ts in zip(observations, want["ts"]):
            assert math.isclose(observation.ts, ts, abs_tol=1e-6), f"{label} {name}: ts {observation.ts} != {ts}"
            data = observation.data
            module = f"{type(data).__module__}.{type(data).__qualname__}"
            assert module == want["payload_type"], f"{label} {name}: decoded {module}"
            if isinstance(data, Image):
                assert (data.width, data.height) == (want["width"], want["height"]), f"{label} {name}: {data.width}x{data.height}"
    print(f"{label}: {len(expected)} streams read back through dimos")


directory = sys.argv[1]
expected = json.load(open(f"{directory}/dimos.db.expected.json"))
with SqliteStore(path=f"{directory}/dimos.db", must_exist=True) as store:
    check(store, expected, "SqliteStore(dimos.db)")

if len(sys.argv) > 2 and sys.argv[2] == "--mcap":
    from dimos.memory.store.mcap import McapStore
    from dimos.memory.codecs.base import resolve_payload_type

    expected = json.load(open(f"{directory}/dimos.mcap.expected.json"))
    lz4_streams = {"imu", "lidar", "depth_image"}
    codecs = {}
    for name, want in expected.items():
        if name == "color_image":
            continue  # a jpeg channel decodes itself
        codec = LcmCodec(resolve_payload_type(want["payload_type"]))
        codecs[name] = Lz4Codec(codec) if name in lz4_streams else codec
    with McapStore(path=f"{directory}/dimos.mcap", codecs=codecs) as store:
        check(store, expected, "McapStore(dimos.mcap)")
