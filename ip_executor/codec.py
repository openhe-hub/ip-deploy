"""Wire-compatible duplicate of ip_runner.codec for the franka-nuc side.

Both ends are Python so we use pickle. Keeping a separate file here means the
NUC executor doesn't need ip_runner on PYTHONPATH.

If you change one, change the other.
"""

from __future__ import annotations

import pickle


def encode(msg: dict) -> bytes:
    return pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)


def decode(buf: bytes) -> dict:
    return pickle.loads(buf)
