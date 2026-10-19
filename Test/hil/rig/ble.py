"""Bluetooth LE link to a Shimmer3R or a Shimmer3 with an RN4678, through the
module's own BLE serial service.

The module bridges that service to the same UART as its SPP link, so the
protocol over it is the one SHIMMER3_BT_COMMUNICATION_PROTOCOL.md describes
for Bluetooth. Each module family has its own service (Profile): the IF820's
CYSPP, and the RN4678's Microchip transparent UART.
BleSerial is the part of pyserial's Serial that BtLink and rig/throughput.py
use - read with a timeout, in_waiting, write, reset_input_buffer, close - so a
measurement written for SPP runs over BLE unchanged: BtLink(ser=BleSerial(mac)).

Data arrives as notifications and commands go out as writes without response.
On CYSPP both use the Unacknowledged Data characteristic, the way the web SDK
does, and the RX flow characteristic is subscribed to as well: it tells the
module the host honours its flow control. On the RN4678 data comes on the
transparent UART's TX characteristic and goes out on its RX one.

Both modules advertise under their classic MAC (the RN4678 as "S3BLE-xxxx"),
so a unit is found by rig.toml's `mac`. Needs bleak. Its asyncio loop runs in a background thread, so the
callers stay synchronous.
"""

from __future__ import annotations

import asyncio
import importlib.util
import threading
import time
from concurrent.futures import TimeoutError as FutureTimeout

from dataclasses import dataclass

SERVICE = "65333333-a115-11e2-9e9a-0800200ca100"
UNACKED_DATA = "65333333-a115-11e2-9e9a-0800200ca102"   # write without response; notify
RX_FLOW = "65333333-a115-11e2-9e9a-0800200ca103"        # indicate
ATT_HEADER = 3                                          # opcode and handle, out of each ATT MTU


@dataclass(frozen=True)
class Profile:
    """A module family's BLE serial service: where data comes in and goes out."""
    name: str
    service: str
    notify: str              # module -> host, as notifications
    write: str               # host -> module, written without response
    flow: str | None = None  # flow control to subscribe to, where the module has it


CYSPP = Profile("IF820 CYSPP", SERVICE, UNACKED_DATA, UNACKED_DATA, RX_FLOW)
# Microchip's transparent UART on the RN4678 (bench, 19 Oct 2026: ATT MTU 160,
# 157 bytes per write without response, advertising as "S3BLE-xxxx").
MCHP_TRANSPARENT = Profile("RN4678 transparent UART", "49535343-fe7d-4ae5-8fa9-9fafd205e455",
                           notify="49535343-1e4d-4bd9-ba61-23c647249616",
                           write="49535343-8841-43f4-a8d4-ecbe34729bb3")


def profile_for(radio: str | None) -> Profile | None:
    """The BLE serial service of the module rig.toml's `radio` names, or None
    for a module that has no BLE side (an RN41 or RN42)."""
    r = (radio or "").upper()
    if r.startswith("IF820"):
        return CYSPP
    if r.startswith("RN4678"):
        return MCHP_TRANSPARENT
    return None

SCAN_S = 15.0
CONNECT_S = 30.0
WRITE_S = 10.0
CANCEL_S = 5.0


class BleError(Exception):
    """The BLE link could not be made, or dropped."""


def available() -> bool:
    """bleak is installed, so this host can make the link."""
    return importlib.util.find_spec("bleak") is not None


def _address(mac: str) -> str:
    m = mac.replace(":", "").replace("-", "").upper()
    return ":".join(m[i:i + 2] for i in range(0, 12, 2))


class BleSerial:
    """A connected CYSPP link that reads and writes like a serial port.

    read(n) waits up to `timeout` seconds for n bytes and returns what has
    arrived by then, as pyserial does. Once the link drops, reading what is
    left still works; past that, read() and write() raise BleError.
    """

    def __init__(self, mac: str, timeout: float | None = 1.0, scan_s: float = SCAN_S, profile: Profile = CYSPP):
        self.profile = profile
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
        await client.start_notify(self.profile.notify, self._on_data)
        if self.profile.flow:
            try:
                await client.start_notify(self.profile.flow, self._on_flow)
            except Exception:   # best effort, as in the web SDK
                pass
        self.mtu = client.mtu_size
        # What one write without response may carry: the characteristic's own
        # figure where the platform gives it, else the ATT payload of the MTU.
        char = client.services.get_characteristic(self.profile.write)
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
            self._call(self._client.write_gatt_char(self.profile.write, bytes(data[i:i + step]), response=False),
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
        return f"BLE {self.address} ({self.profile.name}), ATT MTU {self.mtu}, notifications"
