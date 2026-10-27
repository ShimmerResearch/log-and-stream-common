"""Which optional sensors a Shimmer3R actually has fitted.

The ".1" board revisions stopped placing the LIS3MDL alternative magnetometer
and, except on the IMU board, the ADXL371 high-g accelerometer
(docs/SHIMMER3_BOARD_REVISIONS.md). The firmware knows - these are ports of
ShimBrd_isAdxl371Present() and ShimBrd_isLis3mdlPresent() in
Boards/shimmer_boards.c - but uses that only in the factory test: the
configuration still accepts the channel, and a missing part streams zeros.
The release matrix therefore asks only for what is fitted.

Keep these in step with Boards/shimmer_boards.c.
"""

from __future__ import annotations

SHIMMER3_IMU = 31
EXP_BRD_GSR_UNIFIED = 48
EXP_BRD_EXG_UNIFIED = 47
EXP_BRD_BR_AMP_UNIFIED = 49
EXP_BRD_PROTO3_DELUXE = 38

_ALT_BOARDS = {(EXP_BRD_GSR_UNIFIED, 6, 0), (EXP_BRD_GSR_UNIFIED, 7, 0), (EXP_BRD_GSR_UNIFIED, 7, 1),
               (EXP_BRD_GSR_UNIFIED, 7, 2), (EXP_BRD_GSR_UNIFIED, 8, 0),
               (EXP_BRD_EXG_UNIFIED, 7, 0), (EXP_BRD_EXG_UNIFIED, 7, 1), (EXP_BRD_EXG_UNIFIED, 8, 0),
               (EXP_BRD_BR_AMP_UNIFIED, 4, 0), (EXP_BRD_PROTO3_DELUXE, 4, 0)}

OPTIONAL = {"alt_accel", "alt_mag"}


def fitted_optional(card_id: bytes) -> set[str]:
    """card_id: the expansion-board ID page's first three bytes (board, revision, special)."""
    if len(card_id) < 3 or card_id[0] == 0xFF:
        return set()                       # ShimBrd_isDaughterCardIdSet() false: neither
    board = (card_id[0], card_id[1], card_id[2])
    out = set()
    if card_id[0] == SHIMMER3_IMU or board in _ALT_BOARDS:
        out.add("alt_accel")
    if (SHIMMER3_IMU, 11, 0) == board or board in _ALT_BOARDS:
        out.add("alt_mag")
    return out
