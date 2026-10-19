"""Bluetooth LE link to a Shimmer3R, through its IF820's CYSPP serial service.

The IF820 bridges CYSPP to the same UART as its SPP link, so the protocol over
it is the one SHIMMER3_BT_COMMUNICATION_PROTOCOL.md describes for Bluetooth.
BleSerial is the part of pyserial's Serial that BtLink and rig/throughput.py
use - read with a timeout, in_waiting, write, reset_input_buffer, close - so a
measurement written for SPP runs over BLE unchanged: BtLink(ser=BleSerial(mac)).

Data arrives as notifications on CYSPP's Unacknowledged Data characteristic,
the way the web SDK receives it, and commands go out on it as writes without
response. The module's RX flow characteristic is subscribed to as well, as the
web SDK does: it tells the module the host honours its flow control.

The IF820 advertises under its classic MAC, so a unit is found by rig.toml's
`mac`. Needs bleak. Its asyncio loop runs in a background thread, so the
callers stay synchronous.
"""

from __future__ import annotations

import asyncio
import importlib.util
import sys
import threading
import time
from concurrent.futures import TimeoutError as FutureTimeout

SERVICE = "65333333-a115-11e2-9e9a-0800200ca100"
UNACKED_DATA = "65333333-a115-11e2-9e9a-0800200ca102"   # write without response; notify
RX_FLOW = "65333333-a115-11e2-9e9a-0800200ca103"        # indicate
ATT_HEADER = 3                                          # opcode and handle, out of each ATT MTU

SCAN_S = 15.0
CONNECT_S = 30.0
WRITE_S = 10.0
CANCEL_S = 5.0


# Host programs that hold a Bluetooth link of their own and so cut what the PC
# can carry to a unit over BLE. Phone Link keeps a link to the paired phone
# open: with it running, a Shimmer3R's BLE data-rate test read about 75 KB/s
# where the same unit, firmware and module read about 120 with it closed and
# Bluetooth switched off and on (19 Oct 2026, TEST_PROCEDURE §5.5).
BLE_INTERFERING = {"phoneexperiencehost.exe": "Phone Link (holds a Bluetooth link to a phone)"}


class BleError(Exception):
    """The BLE link could not be made, or dropped."""


def available() -> bool:
    """bleak is installed, so this host can make the link."""
    return importlib.util.find_spec("bleak") is not None


def running(names: dict[str, str], listing: str | None = None) -> list[str]:
    """The descriptions of the programs in `names` (executable -> description) that
    are running. Windows only; `listing` is tasklist's CSV output, for tests."""
    if listing is None:
        if sys.platform != "win32":
            return []
        import subprocess
        listing = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True,
                                 text=True, encoding="utf-8", errors="replace").stdout
    listing = listing.lower()
    return [desc for exe, desc in names.items() if f'"{exe}"' in listing]


def _address(mac: str) -> str:
    m = mac.replace(":", "").replace("-", "").upper()
    return ":".join(m[i:i + 2] for i in range(0, 12, 2))


class BleSerial:
    """A connected CYSPP link that reads and writes like a serial port.

    read(n) waits up to `timeout` seconds for n bytes and returns what has
    arrived by then, as pyserial does. Once the link drops, reading what is
    left still works; past that, read() and write() raise BleError.
    """

    def __init__(self, mac: str, timeout: float | None = 1.0, scan_s: float = SCAN_S):
        self.timeout = timeout
        self.address = _address(mac)
        self.mtu = 0
        self.write_size = 20            # the minimum ATT payload, until the link says more
        self.flow_indications = 0
        self._buf = bytearray()
        self._cond = threading.Condition()
        self._lost = False
        self._client = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True, name=f"ble {self.address}")
        self._thread.start()
        try:
            self._call(self._connect(scan_s), scan_s + CONNECT_S)
        except BaseException:
            self.close()
            raise

    # --- the asyncio side -------------------------------------------------

    def _call(self, coro, timeout_s: float):
        """Run a coroutine on the link's loop. Every failure comes back as BleError,
        bleak's own exceptions included, so callers handle one type."""
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            return fut.result(timeout_s)
        except (FutureTimeout, TimeoutError) as e:   # one class from Python 3.11, two on 3.10
            # A timed-out scan, connect or write would otherwise go on running
            # on the loop. Cancelling the future cancels its task there.
            fut.cancel()
            raise BleError(f"{self.address}: no answer within {timeout_s:.0f} s") from e
        except BleError:
            raise
        except Exception as e:
            raise BleError(f"{self.address}: {type(e).__name__}: {e}") from e

    @staticmethod
    async def _cancel_pending() -> None:
        tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def _connect(self, scan_s: float) -> None:
        from bleak import BleakClient, BleakScanner

        device = await BleakScanner.find_device_by_address(self.address, timeout=scan_s)
        if device is None:
            raise BleError(f"{self.address} is not advertising: is it on, in range, and not connected "
                           "to another host?")
        client = BleakClient(device, disconnected_callback=self._on_disconnect)
        # Kept before connecting: if the connect times out and is cancelled,
        # close() still has the client to disconnect.
        self._client = client
        await client.connect()
        await client.start_notify(UNACKED_DATA, self._on_data)
        try:
            await client.start_notify(RX_FLOW, self._on_flow)
        except Exception:   # best effort, as in the web SDK
            pass
        self.mtu = client.mtu_size
        # What one write without response may carry: the characteristic's own
        # figure where the platform gives it, else the ATT payload of the MTU.
        char = client.services.get_characteristic(UNACKED_DATA)
        size = getattr(char, "max_write_without_response_size", 0) if char else 0
        self.write_size = max(20, size or self.mtu - ATT_HEADER)

    def _on_data(self, _char, data: bytearray) -> None:
        with self._cond:
            self._buf += data
            self._cond.notify_all()

    def _on_flow(self, _char, _data) -> None:
        self.flow_indications += 1

    def _on_disconnect(self, _client) -> None:
        with self._cond:
            self._lost = True
            self._cond.notify_all()

    # --- the serial side --------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._client is not None and not self._lost and self.mtu > 0

    @property
    def in_waiting(self) -> int:
        with self._cond:
            return len(self._buf)

    def read(self, size: int = 1) -> bytes:
        end = None if self.timeout is None else time.monotonic() + self.timeout
        with self._cond:
            while len(self._buf) < size and not self._lost:
                left = None if end is None else end - time.monotonic()
                if left is not None and left <= 0:
                    break
                self._cond.wait(left)
            if not self._buf and self._lost:
                raise BleError(f"{self.address}: BLE link dropped")
            out = bytes(self._buf[:size])
            del self._buf[:size]
            return out

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise BleError(f"{self.address}: BLE link dropped")
        step = self.write_size
        for i in range(0, len(data), step):
            self._call(self._client.write_gatt_char(UNACKED_DATA, bytes(data[i:i + step]), response=False),
                       WRITE_S)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._cond:
            self._buf.clear()

    def close(self) -> None:
        try:
            if self._client is not None and self._client.is_connected:
                self._call(self._client.disconnect(), WRITE_S)
        except Exception:
            pass
        finally:
            # Under the condition, and with everyone told, as _on_disconnect does:
            # a reader blocked in read() then sees the link is gone at once.
            with self._cond:
                self._lost = True
                self._cond.notify_all()
            if self._loop.is_running():
                # Anything still in flight - a timed-out operation, a notification
                # handler - is cancelled and waited for before the loop stops.
                try:
                    asyncio.run_coroutine_threadsafe(self._cancel_pending(), self._loop).result(CANCEL_S)
                except Exception:
                    pass
                self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join(5.0)
            if not self._thread.is_alive() and not self._loop.is_closed():
                self._loop.close()

    def describe(self) -> str:
        return f"BLE {self.address}, ATT MTU {self.mtu}, notifications"
