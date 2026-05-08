"""
OBD2 BLE Tester — Flask app for testing the WROOM-OBD firmware.

Architecture: Flask (sync, threaded=True) + BleManager (asyncio in daemon thread).
Commands are dispatched to the BLE loop via asyncio.run_coroutine_threadsafe().
Device logs and OBD responses are delivered to the browser via SSE (/api/stream).
"""

import asyncio
import itertools
import json
import queue
import threading
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner
from flask import Flask, Response, jsonify, render_template, request

# ── BLE constants ────────────────────────────────────────────────────────────

DEVICE_NAME = "WROOM-OBD"
NUS_RX_UUID = "6e400002-b5b3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5b3-f393-e0a9-e50e24dcca9e"

# ── OBD2 Mode 01 decoders ────────────────────────────────────────────────────
# Each lambda receives the raw OBD2 response bytes: [0x41, pid, A, B?, ...]

PID_DECODERS = {
    # 0x04: lambda d: f"Engine Load: {d[2] * 100 / 255:.1f}%",
    # 0x05: lambda d: f"Coolant Temp: {d[2] - 40}°C",
    # 0x06: lambda d: f"STFT Bank 1: {(d[2] - 128) * 100 / 128:.1f}%",
    # 0x07: lambda d: f"LTFT Bank 1: {(d[2] - 128) * 100 / 128:.1f}%",
    # 0x0B: lambda d: f"Intake MAP: {d[2]} kPa",
    # 0x0C: lambda d: f"Engine RPM: {((d[2] * 256) + d[3]) / 4:.0f} rpm",
    # 0x0D: lambda d: f"Vehicle Speed: {d[2]} km/h",
    # 0x0E: lambda d: f"Timing Advance: {d[2] / 2 - 64:.1f}°",
    # 0x0F: lambda d: f"Intake Air Temp: {d[2] - 40}°C",
    # 0x10: lambda d: f"MAF Rate: {((d[2] * 256) + d[3]) / 100:.2f} g/s",
    # 0x11: lambda d: f"Throttle Position: {d[2] * 100 / 255:.1f}%",
    # 0x1F: lambda d: f"Run Time: {d[2] * 256 + d[3]} s",
    # 0x2F: lambda d: f"Fuel Tank Level: {d[2] * 100 / 255:.1f}%",
    # 0x33: lambda d: f"Barometric Pressure: {d[2]} kPa",
    # 0x42: lambda d: f"Module Voltage: {(d[2] * 256 + d[3]) / 1000:.3f} V",
    # 0x46: lambda d: f"Ambient Air Temp: {d[2] - 40}°C",
    # 0x51: lambda d: f"Fuel Type: {d[2]}",
}


def decode_dtcs(data_bytes: list[int]) -> list[str]:
    """Decode Mode 03 response bytes into DTC code strings (e.g. P0300)."""
    codes = []
    # Format: [0x43, count, high1, low1, high2, low2, ...]
    i = 2
    while i + 1 < len(data_bytes):
        high = data_bytes[i]
        low = data_bytes[i + 1]
        if high == 0 and low == 0:
            i += 2
            continue
        prefix = ["P", "C", "B", "U"][(high >> 6) & 0x03]
        d1 = (high >> 4) & 0x03
        d2 = high & 0x0F
        d3 = (low >> 4) & 0x0F
        d4 = low & 0x0F
        codes.append(f"{prefix}{d1}{d2:X}{d3:X}{d4:X}")
        i += 2
    return codes


def format_response(mode: str, pid: int | None, raw: dict) -> str:
    """Convert a raw device JSON response into a human-readable string."""
    status = raw.get("status")
    data = raw.get("data")
    debug = raw.get("debug")

    if status == "ERROR":
        return f"[Mode {mode}] ERROR: {debug}"

    if mode == "09":
        if not data:
            return "VIN: (no data)"
        raw_bytes = bytes(data)
        # OBD2 Mode 09 VIN response: [0x49, 0x02, count, <VIN bytes>]
        if len(raw_bytes) >= 3 and raw_bytes[0] == 0x49 and raw_bytes[1] == 0x02:
            vin_bytes = raw_bytes[3:]
        else:
            vin_bytes = raw_bytes
        vin = vin_bytes.decode("ascii", errors="replace").strip("\x00 ")
        return f"VIN: {vin}"

    if mode == "03":
        if not data:
            return "Stored DTCs: none"
        all_codes: list[str] = []
        for ecu in data:
            all_codes.extend(decode_dtcs(ecu["data"]))
        label = ", ".join(all_codes) if all_codes else "none"
        return f"Stored DTCs: {label}"

    if mode == "04":
        return "DTCs cleared successfully"

    if mode == "01" and pid is not None:
        if not data:
            return f"PID 0x{pid:02X}: no data"
        for ecu in data:
            raw_bytes = ecu["data"]
            if len(raw_bytes) >= 2:
                decoder = PID_DECODERS.get(pid)
                if decoder:
                    try:
                        return decoder(raw_bytes)
                    except (IndexError, ZeroDivisionError):
                        pass
                hex_str = " ".join(f"{b:02X}" for b in raw_bytes)
                return f"PID 0x{pid:02X}: [{hex_str}]"
        return f"PID 0x{pid:02X}: no ECU response"

    return f"[Mode {mode}] {json.dumps(data)}"


# ── BLE Manager ──────────────────────────────────────────────────────────────

class BleManager:
    """
    Manages the BLE connection in a dedicated asyncio thread.
    All public methods are synchronous and safe to call from any thread.
    """

    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._client: BleakClient | None = None
        self._connected = False
        self._device_name = ""
        self._req_id = itertools.count(1)
        self._pending: dict[int, asyncio.Future] = {}
        self._rx_buf = b""

        self.log_queue: queue.Queue = queue.Queue()
        self.response_queue: queue.Queue = queue.Queue()

        t = threading.Thread(target=self._run_loop, daemon=True)
        t.start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_queue.put({"time": ts, "msg": msg})

    # ── Asyncio callbacks (called from BLE loop thread) ──────────────────────

    def _on_notification(self, _sender, data: bytearray) -> None:
        self._rx_buf += bytes(data)
        # Try to parse complete JSON objects; buffer handles fragmented MTU chunks
        while self._rx_buf:
            try:
                obj = json.loads(self._rx_buf.decode("utf-8"))
                self._rx_buf = b""
                self._dispatch(obj)
            except json.JSONDecodeError:
                if len(self._rx_buf) > 4096:
                    self._log(f"RX overflow, dropping {len(self._rx_buf)} bytes")
                    self._rx_buf = b""
                break

    def _dispatch(self, obj: dict) -> None:
        req_id = obj.get("id")
        if req_id is not None and req_id in self._pending:
            fut = self._pending.pop(req_id)
            if not fut.done():
                self._loop.call_soon_threadsafe(fut.set_result, obj)
        else:
            self._log(f"Unsolicited: {json.dumps(obj)}")

    def _on_disconnect(self, _client: BleakClient) -> None:
        self._connected = False
        self._client = None
        for fut in self._pending.values():
            if not fut.done():
                fut.cancel()
        self._pending.clear()
        self._log("Disconnected")

    # ── Async internals ──────────────────────────────────────────────────────

    async def _connect(self, address_or_name: str | None) -> bool:
        self._log("Scanning…")
        try:
            if address_or_name:
                address = address_or_name
            else:
                device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
                if device is None:
                    self._log("Device not found")
                    return False
                address = device.address
                self._log(f"Found {device.name} at {device.address}")

            client = BleakClient(address, disconnected_callback=self._on_disconnect)
            await client.connect(timeout=10.0)
            self._client = client
            self._connected = True
            self._device_name = address_or_name or DEVICE_NAME

            await client.start_notify(NUS_TX_UUID, self._on_notification)
            self._log(f"Connected to {self._device_name}")

            try:
                mtu = await client.request_mtu(247)
                self._log(f"MTU negotiated: {mtu}")
            except Exception:
                pass

            return True

        except Exception as e:
            self._log(f"Connection failed: {e}")
            return False

    async def _disconnect(self) -> None:
        if self._client and self._connected:
            await self._client.disconnect()
        self._connected = False
        self._client = None

    async def _send(self, cmd) -> dict:
        if not self._client or not self._connected:
            raise RuntimeError("Not connected")

        req_id = next(self._req_id)
        payload = json.dumps({"id": req_id, "cmd": cmd}).encode("utf-8")

        fut: asyncio.Future = self._loop.create_future()
        self._pending[req_id] = fut

        await self._client.write_gatt_char(NUS_RX_UUID, payload, response=False)
        try:
            return await asyncio.wait_for(fut, timeout=5.0)
        except asyncio.TimeoutError:
            self._pending.pop(req_id, None)
            raise

    # ── Public thread-safe API ───────────────────────────────────────────────

    def connect(self, address_or_name: str | None = None) -> bool:
        future = asyncio.run_coroutine_threadsafe(self._connect(address_or_name), self._loop)
        return future.result(timeout=15.0)

    def disconnect(self) -> None:
        future = asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        future.result(timeout=5.0)

    def send(self, cmd) -> dict:
        future = asyncio.run_coroutine_threadsafe(self._send(cmd), self._loop)
        return future.result(timeout=6.0)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def device_name(self) -> str:
        return self._device_name


# ── Flask app ────────────────────────────────────────────────────────────────

app = Flask(__name__)
ble = BleManager()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    return jsonify({"connected": ble.connected, "device": ble.device_name})


@app.route("/api/connect", methods=["POST"])
def api_connect():
    body = request.get_json(silent=True) or {}
    address = body.get("address") or None
    try:
        ok = ble.connect(address)
        return jsonify({"ok": ok})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    try:
        ble.disconnect()
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/request", methods=["POST"])
def api_request():
    body = request.get_json()
    mode = body.get("mode")
    pid = body.get("pid")

    if mode == "01":
        if pid is None:
            return jsonify({"error": "pid required for mode 01"}), 400
        cmd = {"get_live_data": {"pid": pid}}
    elif mode == "03":
        cmd = "get_stored_dtcs"
    elif mode == "04":
        cmd = "clear_dtcs"
    elif mode == "09":
        cmd = "get_vin"
    else:
        return jsonify({"error": f"unknown mode {mode}"}), 400

    try:
        raw = ble.send(cmd)
    except RuntimeError as e:
        return jsonify({"error": str(e)}), 400
    except asyncio.TimeoutError:
        return jsonify({"error": "OBD2 request timed out"}), 504
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    formatted = format_response(mode, pid, raw)
    ts = datetime.now().strftime("%H:%M:%S")
    ble.response_queue.put({"time": ts, "msg": formatted})
    return jsonify({"ok": True, "result": formatted, "raw": raw})


@app.route("/api/stream")
def api_stream():
    """SSE endpoint. Emits 'log' and 'response' events."""
    def generate():
        while True:
            sent = False

            try:
                while True:
                    item = ble.log_queue.get_nowait()
                    yield f"event: log\ndata: {json.dumps(item)}\n\n"
                    sent = True
            except queue.Empty:
                pass

            try:
                while True:
                    item = ble.response_queue.get_nowait()
                    yield f"event: response\ndata: {json.dumps(item)}\n\n"
                    sent = True
            except queue.Empty:
                pass

            if not sent:
                yield ": ping\n\n"

            time.sleep(0.4)

    return Response(
        generate(),
        content_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, threaded=True, debug=False)
