"""Time alignment between the ADMA ego clock and the annotation clock.

Both files carry a field called "chunktime" (ADTF_CHUNK_TIME in the ADMA CSV,
the `chunktime` attribute on every annotation timestamp element), which in a
clean recording session are the same ADTF pipeline clock and can be used
directly with zero offset. In practice the two loggers can start slightly
apart or drift, which is exactly the "not fully synced" bottleneck this tool
exists to work around -- so we treat 0 as the default/starting hypothesis
and expose an adjustable `sync_offset_us` (applied to the annotation clock
before it is used to sample the ego trace) that the GUI lets a user nudge
while watching the replay, rather than guessing a "correct" automatic value
from a single trace with no ground truth to validate against.
"""
from __future__ import annotations

DEFAULT_SYNC_OFFSET_US = 0


def apply_offset(annotation_t_us: int, sync_offset_us: int) -> int:
    """Maps an annotation-clock timestamp onto the ADMA ego clock."""
    return annotation_t_us + sync_offset_us
