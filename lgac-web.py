#!/usr/bin/env python3
"""
lgac-web.py  --  Yalgacr v1.0  --  web interface
Yet Another LG AC Remote -- web UI for LG remote AKB74955603 (LG S12EQ)

Features:
  - Web GUI to control the AC (power, mode, temp, fan, vertical swing, jet,
    light, purify/moisture/quiet/energy)
  - LCD-style status display reflecting the last commanded state, with optional
    BME280 ambient readings and a per-user "LED backlight" colour
  - Sends IR commands via USB IR Toy v2 (local serial or remote socat TCP)
  - In-memory state, persisted to lgac-state.json on every change
  - Authentication (always required), admin/user roles, hashed passwords;
    plain-text audit log (lgac.log), shared with the CLI
  - Daily on/off scheduler (up to 3 cycles per weekday)
  - HTTP or HTTPS (self-signed / Let's Encrypt / manual cert), switchable in
    the Settings page with an in-process restart
  - All settings (server/IR/sensors/schedule) and users in lgac-config.json,
    managed from the Settings page

The IR encoding logic is intentionally duplicated here (not shared as a
module) so this file is self-contained and can be deployed without the CLI.

Run:
    python3 lgac-web.py                       # foreground (manual)
    python3 lgac-web.py --config /path/cfg    # custom config (state sits beside it)

Ports/protocol are set in the Settings page, not on the command line.
Default login on first run:  admin / admin  (change it in Settings).
"""

import os
import re
import sys
import copy
import json
import html
import time
import ssl
import struct
import signal
import socket
import logging
import secrets
import argparse
import threading
import subprocess
from functools import wraps
from datetime import timedelta
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed:  sudo apt install python3-serial")

try:
    from flask import (Flask, request, jsonify, Response, session,
                       redirect, url_for, abort)
    from werkzeug.serving import make_server
except ImportError:
    sys.exit("Flask is not installed:  sudo apt install python3-flask")

try:
    from werkzeug.security import generate_password_hash, check_password_hash
except ImportError:
    sys.exit("Werkzeug is not installed:  sudo apt install python3-werkzeug")

# Silence werkzeug's per-request access log (the "GET /api/changes 200 -" lines).
# Errors are still shown; our own startup/status prints are unaffected.
logging.getLogger("werkzeug").setLevel(logging.ERROR)


# ===========================================================================
# Configuration
# ===========================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(SCRIPT_DIR, "lgac-state.json")
CONFIG_FILE = os.path.join(SCRIPT_DIR, "lgac-config.json")

# How long a login session stays valid. Configurable here in code only
# (intentionally not exposed in the GUI).
SESSION_LIFETIME_DAYS = 7

# Default bind address. The server protocol, ports and IR connection all live
# in lgac-config.json and are managed from the Settings page.
WEB_HOST = "0.0.0.0"

IR_REPEAT = 1               # how many times to send each frame


# ===========================================================================
# IR timings (us) -- matched to captured AKB74955603 remote
# ===========================================================================
HDR_MARK   = 3150
HDR_SPACE  = 9900
BIT_MARK   = 450
ONE_SPACE  = 1600
ZERO_SPACE = 575
IRTOY_TICK_US = 21.3333


# ===========================================================================
# Protocol tables (verified against captures)
# ===========================================================================
PREFIX_N6, PREFIX_N5 = 0x8, 0x8

MODE_N3 = {"cool": 0x8, "dry": 0x9, "fan": 0xA, "auto": 0xB, "heat": 0xC}

# Fan speed -> N1 nibble. NON-LINEAR, must be a lookup table.
FAN_N1 = {"1": 0x0, "2": 0x9, "3": 0x2, "4": 0xA, "5": 0x4, "auto": 0x5}

TEMP_LIMITS = {
    "cool": (18, 30),
    "heat": (16, 30),
    "auto": (18, 30),
    "dry":  (18, 30),
    "fan":  (18, 30),
}

# Swing command space: N4 N3 = 1 3
VSWING = {
    "1": (0x0, 0x4), "2": (0x0, 0x5), "3": (0x0, 0x6),
    "4": (0x0, 0x7), "5": (0x0, 0x8), "6": (0x0, 0x9),
    "full": (0x1, 0x4), "off": (0x1, 0x5),
}

# Special fixed frames (N4=C command group)
FRAME_OFF   = 0x88C0051
FRAME_JET   = 0x8810089
FRAME_LIGHT = 0x88C00A6
FRAME_PURIFY_ON  = 0x88C000C
FRAME_PURIFY_OFF = 0x88C0084
FRAME_MOISTURE_ON  = 0x88C00B7
FRAME_MOISTURE_OFF = 0x88C00C8
FRAME_QUIET_ON  = 0x88C0A6C
FRAME_QUIET_OFF = 0x88C0A7D
FRAME_ENERGY = {
    "80":  0x88C07D0,
    "60":  0x88C07E1,
    "40":  0x88C0804,
    "off": 0x88C07F2,
}

DEFAULT_STATE = {
    "power": "off",
    "mode": "cool",
    "temp": 22,
    "fan": "auto",
    "vswing": None,
    "jet": "off",
    "light": "on",
    "purify": "off",
    "moisture": "off",
    "quiet": "off",
    "energy": "off",
    "updated": None,
    "last_action": "",       # short action string of the most recent command (footer/toast)
    "last_action_time": "",  # HH:MM of the most recent command
}


# ===========================================================================
# Encoder
# ===========================================================================
def _checksum(n6, n5, n4, n3, n2, n1):
    return (n6 + n5 + n4 + n3 + n2 + n1) & 0xF


def make_frame(n4, n3, n2, n1):
    n6, n5 = PREFIX_N6, PREFIX_N5
    k = _checksum(n6, n5, n4, n3, n2, n1)
    val = 0
    for nib in (n6, n5, n4, n3, n2, n1, k):
        val = (val << 4) | (nib & 0xF)
    return val


def encode_climate(mode, temp, fan):
    if mode not in MODE_N3:
        raise ValueError(f"Unknown mode: {mode}")
    lo, hi = TEMP_LIMITS[mode]
    if not (lo <= temp <= hi):
        raise ValueError(f"Temperature {temp}C out of range for {mode} ({lo}-{hi}C)")
    fan = str(fan)
    if fan not in FAN_N1:
        raise ValueError(f"Unknown fan speed: {fan}")
    n3 = MODE_N3[mode]
    n2 = (temp - 15) & 0xF
    n1 = FAN_N1[fan]
    return make_frame(0x0, n3, n2, n1)


def encode_power_on(mode, temp, fan):
    lo, hi = TEMP_LIMITS.get(mode, (18, 30))
    temp = max(lo, min(temp, hi))
    fan = str(fan)
    n2 = (temp - 15) & 0xF
    n1 = FAN_N1.get(fan, 0x5)
    return make_frame(0x0, 0x0, n2, n1)


def encode_vswing(pos):
    if pos not in VSWING:
        raise ValueError(f"Unknown vertical swing: {pos}")
    n2, n1 = VSWING[pos]
    return make_frame(0x1, 0x3, n2, n1)


def frame_to_timings(state):
    t = [HDR_MARK, HDR_SPACE]
    for i in range(27, -1, -1):
        t.append(BIT_MARK)
        t.append(ONE_SPACE if (state >> i) & 1 else ZERO_SPACE)
    t.append(BIT_MARK)
    return t


def timings_to_irtoy(timings):
    out = bytearray()
    for us in timings:
        ticks = int(round(us / IRTOY_TICK_US))
        ticks = max(1, min(ticks, 0xFFFE))
        out += struct.pack(">H", ticks)
    out += b"\xFF\xFF"
    return bytes(out)


# ===========================================================================
# Transports (local serial / remote socket) -- same design as the CLI
# ===========================================================================
class _SerialTransport:
    def __init__(self, device, timeout):
        self.s = serial.serial_for_url(device, baudrate=115200, timeout=timeout)

    def write(self, data):
        self.s.write(data)
        try:
            self.s.flush()
        except Exception:
            pass

    def read(self, n):
        return self.s.read(n)

    def drain_input(self):
        try:
            self.s.reset_input_buffer()
        except Exception:
            old = self.s.timeout
            self.s.timeout = 0.05
            try:
                while self.s.read(64):
                    pass
            finally:
                self.s.timeout = old

    def close(self):
        self.s.close()


class _SocketTransport:
    def __init__(self, host, port, timeout):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout

    def write(self, data):
        self.sock.sendall(data)

    def read(self, n):
        self.sock.settimeout(self.timeout)
        chunks = []
        remaining = n
        try:
            while remaining > 0:
                b = self.sock.recv(remaining)
                if not b:
                    break
                chunks.append(b)
                remaining -= len(b)
                if chunks and remaining > 0:
                    self.sock.settimeout(0.15)
        except socket.timeout:
            pass
        except OSError:
            pass
        return b"".join(chunks)

    def drain_input(self):
        self.sock.settimeout(0.05)
        try:
            while True:
                b = self.sock.recv(256)
                if not b:
                    break
        except (socket.timeout, OSError):
            pass
        finally:
            self.sock.settimeout(self.timeout)

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class IrToy:
    def __init__(self, transport, verbose=False):
        self.t = transport
        self.verbose = verbose
        time.sleep(0.1)
        self._init_device()

    def _init_device(self):
        self.t.write(b"\x00" * 5)
        time.sleep(0.1)
        self.t.drain_input()
        self.t.write(b"S")
        time.sleep(0.05)
        proto = self.t.read(3)
        if not proto.startswith(b"S"):
            raise IOError(
                f"IR Toy did not enter sample mode (returned: {proto!r}). "
                "Check: lircd stopped? socat at 115200? Correct port/host?"
            )
        self.t.write(bytes([0x26, 0x25, 0x24]))
        time.sleep(0.05)
        self.t.drain_input()

    def transmit_once(self, payload):
        self.t.drain_input()
        self.t.write(bytes([0x03]))
        hs = self.t.read(1)
        if not hs:
            raise IOError("No handshake response after 0x03.")
        free = hs[0]
        sent, n = 0, len(payload)
        while sent < n:
            chunk = payload[sent : sent + free]
            self.t.write(chunk)
            sent += len(chunk)
            if sent < n:
                hs = self.t.read(1)
                if not hs:
                    raise IOError("Handshake lost during transmission.")
                free = hs[0]
        time.sleep(0.05)
        # The IR Toy sends a short completion report ('t'<count> then 'C'),
        # NOT a fixed 8 bytes. A blocking read of a fixed size would wait for
        # the full serial timeout (several seconds) for bytes that never come.
        # The frame has been queued and the ~60 ms burst emits on its own; give
        # it a brief moment, then just clear whatever the Toy reported back.
        time.sleep(0.08)
        self.t.drain_input()

    def send_frame(self, frame, repeat=1, gap_ms=40):
        payload = timings_to_irtoy(frame_to_timings(frame))
        for i in range(repeat):
            if i:
                time.sleep(gap_ms / 1000.0)
            self.transmit_once(payload)

    def close(self):
        try:
            self.t.write(b"\x00" * 5)
            time.sleep(0.05)
        except Exception:
            pass
        self.t.close()


def open_irtoy(host=None, port=2000, device="/dev/ttyACM0", timeout=5):
    if host:
        transport = _SocketTransport(host, port, timeout)
    else:
        transport = _SerialTransport(device, timeout)
    return IrToy(transport)


# ===========================================================================
# State management (in-memory, persisted to file on every change)
# ===========================================================================
_state_lock = threading.Lock()
_state = dict(DEFAULT_STATE)


def load_state():
    global _state
    try:
        with open(STATE_FILE) as f:
            data = json.load(f)
        merged = dict(DEFAULT_STATE)
        merged.update(data)
        _state = merged
    except (FileNotFoundError, json.JSONDecodeError):
        _state = dict(DEFAULT_STATE)


def save_state():
    snapshot = dict(_state)
    snapshot["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    _state["updated"] = snapshot["updated"]
    try:
        with open(STATE_FILE, "w") as f:
            json.dump(snapshot, f, indent=2)
    except OSError as e:
        print(f"warning: cannot save state file: {e}", file=sys.stderr)


# ===========================================================================
# Config management (lgac-config.json) -- app settings + users
# ===========================================================================
# The config holds everything that isn't AC state: the session secret, the
# user accounts, and (in later stages) server/IR/sensor/schedule settings.
_config_lock = threading.Lock()
_config = {}


def _default_config():
    """First-run config: a random session secret and a default admin/admin."""
    return {
        "secret_key": secrets.token_hex(32),
        "users": {
            "admin": {
                "password_hash": generate_password_hash("admin"),
                "role": "admin",
                "display_color": "blue",
            }
        },
        "server": _default_server_config(),
        "ir": _default_ir_config(),
        "sensors": _default_sensors_config(),
        "schedule": _default_schedule_config(),
    }


def _default_server_config():
    return {
        "protocol": "http",        # "http" or "https" (only one active)
        "http_port": 8080,
        "https_port": 8443,
        # HTTPS certificate source: "selfsigned" (auto-generated, easiest),
        # "letsencrypt" (derive path from domain), or "manual" (explicit paths).
        "cert_source": "selfsigned",
        "domain": "",              # Let's Encrypt domain (cert path is derived)
        "cert_path": "",           # manual cert (used when cert_source == manual)
        "key_path": "",            # manual key  (used when cert_source == manual)
    }


def _default_ir_config():
    return {
        "mode": "local",           # "local" (USB serial) or "remote" (socat TCP)
        "device": "/dev/ttyACM0",
        "host": "",
        "port": 2000,
    }


def _default_sensors_config():
    return {
        "poll_period": 60,         # seconds between reads (one period for all)
        "list": [],                # each: {id, name, mode, address, host}
    }


SCHEDULE_DAYS = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]


def _default_schedule_config():
    # Per weekday, a list of up to 3 cycles. Each cycle: an on time, an off
    # time (both "HH:MM"), and an enabled flag for pausing it individually.
    return {
        "enabled": False,                       # global on/off for the scheduler
        "days": {d: [] for d in SCHEDULE_DAYS},
    }


def load_config():
    """Load config from disk, creating it with defaults on first run.
    Also repairs a missing secret_key or an empty user list so the app can
    never end up unusable (e.g. no way to log in)."""
    global _config
    try:
        with open(CONFIG_FILE) as f:
            cfg = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        cfg = _default_config()
        _config = cfg
        save_config()
        print(f"Created {CONFIG_FILE} with default admin/admin login.")
        return

    changed = False
    if not cfg.get("secret_key"):
        cfg["secret_key"] = secrets.token_hex(32)
        changed = True
    if not cfg.get("users"):
        cfg["users"] = _default_config()["users"]
        changed = True
        print("No users in config -- restored default admin/admin login.")
    else:
        # Older configs predate per-user display colour; default to blue.
        for u in cfg["users"].values():
            if "display_color" not in u:
                u["display_color"] = "blue"; changed = True
    # Merge in newer sections (so a config from an earlier stage gains them),
    # filling only missing keys without clobbering user-set values.
    if "server" not in cfg:
        cfg["server"] = _default_server_config(); changed = True
    else:
        for k, v in _default_server_config().items():
            if k not in cfg["server"]:
                cfg["server"][k] = v; changed = True
    if "ir" not in cfg:
        cfg["ir"] = _default_ir_config(); changed = True
    else:
        for k, v in _default_ir_config().items():
            if k not in cfg["ir"]:
                cfg["ir"][k] = v; changed = True
    if "sensors" not in cfg:
        cfg["sensors"] = _default_sensors_config(); changed = True
    else:
        for k, v in _default_sensors_config().items():
            if k not in cfg["sensors"]:
                cfg["sensors"][k] = v; changed = True
    if "schedule" not in cfg:
        cfg["schedule"] = _default_schedule_config(); changed = True
    else:
        if "enabled" not in cfg["schedule"]:
            cfg["schedule"]["enabled"] = False; changed = True
        if "days" not in cfg["schedule"]:
            cfg["schedule"]["days"] = {d: [] for d in SCHEDULE_DAYS}; changed = True
        else:
            for d in SCHEDULE_DAYS:
                if d not in cfg["schedule"]["days"]:
                    cfg["schedule"]["days"][d] = []; changed = True

    _config = cfg
    if changed:
        save_config()


def save_config():
    with _config_lock:
        snapshot = copy.deepcopy(_config)
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(snapshot, f, indent=2)
        # Config holds password hashes and the session secret -- keep it
        # readable only by the owner.
        try:
            os.chmod(CONFIG_FILE, 0o600)
        except OSError:
            pass
    except OSError as e:
        print(f"warning: cannot save config file: {e}", file=sys.stderr)


def count_admins():
    return sum(1 for u in _config.get("users", {}).values()
               if u.get("role") == "admin")


# ===========================================================================
# Audit log
# ===========================================================================
# A plain-text log next to the script. One line per event:
#   "DD.MM.YYYY HH:MM:SS | user | ip | action"
# Records logins/logouts (including failed attempts, logged as "system") and
# every AC command actually sent. Scheduled (automated) actions are logged as
# user "scheduler" with ip "local". The CLI writes to the same file as user
# "cli" (ip "local"). No rotation by design (KISS) -- it grows one short line
# per action. The lock guards the app's own threads (request handlers,
# scheduler, poller); cross-process appends with the CLI stay intact because
# the file is opened in O_APPEND mode and each line is a single small write.
LOG_FILE = os.path.join(SCRIPT_DIR, "lgac.log")
_log_lock = threading.Lock()


def log_event(user, ip, action):
    """Append one audit line. Never raises -- logging must not break a command.
    Format: 'DD.MM.YYYY HH:MM:SS | user | ip | action'. For server-originated
    actions (scheduler) ip is 'local'."""
    line = f"{time.strftime('%d.%m.%Y %H:%M:%S')} | {user} | {ip} | {action}\n"
    try:
        with _log_lock:
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line)
    except OSError as e:
        print(f"warning: cannot write log: {e}", file=sys.stderr)


def read_last_log_lines(n):
    """Return the last n non-empty lines of the log (oldest first), or [].
    Uses a bounded deque so only the last n lines are held in memory, not the
    whole (unrotated, ever-growing) file."""
    try:
        with open(LOG_FILE, encoding="utf-8") as f:
            tail = deque((ln.rstrip("\n") for ln in f if ln.strip()), maxlen=n)
        return list(tail)
    except FileNotFoundError:
        return []
    except OSError:
        return []


# ===========================================================================
# IR send (serialized so two requests can't hit the port at once)
# ===========================================================================
_ir_lock = threading.Lock()

# Minimum gap between commands sent from the web UI. Rapid-fire commands seem
# to be able to wedge the IR Toy (the port stops enumerating until a power
# cycle), so the web app refuses to send another command until this many
# seconds have passed since the last successful send. The CLI is unaffected.
# This is a code setting on purpose (not exposed in the UI). Scheduled
# (automated) actions bypass the cooldown -- they fire at most once a minute.
IR_COOLDOWN_SEC = 3.0
_cooldown_lock = threading.Lock()
_last_send_ts = 0.0  # time.monotonic() of the last successful UI send


def send_ir(frame):
    """Open IR Toy, send one frame, close. Serialized via _ir_lock.
    Connection settings are read fresh from config each call, so changing
    them in the UI takes effect immediately (no restart). Raises on failure
    so the caller can report it to the UI."""
    ircfg = _config.get("ir", {})
    if ircfg.get("mode") == "remote":
        host = ircfg.get("host") or None
        port = int(ircfg.get("port", 2000))
        device = None
    else:
        host = None
        port = 2000
        device = ircfg.get("device", "/dev/ttyACM0")
    with _ir_lock:
        toy = open_irtoy(host=host, port=port, device=device, timeout=5)
        try:
            toy.send_frame(frame, repeat=IR_REPEAT)
        finally:
            toy.close()


# ===========================================================================
# BME280 sensors (ambient temperature + humidity, display only for now)
# ===========================================================================
# Latest reading per sensor id, updated by the background poller. Other parts
# of the app (and any future automation) can read this without touching I2C.
_sensor_lock = threading.Lock()
_sensor_readings = {}   # id -> {"ok": bool, "temp": float, "humidity": float, "time": str, "error": str}


def _parse_i2c_address(value):
    """Accept '0x76', '76', or an int; I2C addresses are conventionally hex."""
    if isinstance(value, int):
        return value
    s = str(value).strip()
    return int(s, 16)   # '0x76' and '76' both parse as hex


def _read_bme280_local(address):
    """Read a locally-attached BME280 over I2C using RPi.bme280 + smbus2.
    Returns (temperature_c, humidity_pct). Raises on any failure."""
    import smbus2
    import bme280
    addr = _parse_i2c_address(address)
    bus = smbus2.SMBus(1)
    try:
        calib = bme280.load_calibration_params(bus, addr)
        sample = bme280.sample(bus, addr, calib)
        return float(sample.temperature), float(sample.humidity)
    finally:
        try:
            bus.close()
        except Exception:
            pass


def _read_bme280_remote(host, address):
    """Read a BME280 attached to a remote Raspberry Pi running pigpiod, via
    pigpio's network I2C. Implements the Bosch compensation (float variant).
    Returns (temperature_c, humidity_pct). Raises on any failure.

    NOTE: this path cannot be tested without real remote hardware; verify on
    an actual sensor. Based on the Bosch BME280 datasheet + pigpio I2C API."""
    import pigpio
    addr = _parse_i2c_address(address)
    pi = pigpio.pi(host)            # connect to remote pigpiod (port 8888)
    if not pi.connected:
        raise IOError(f"cannot connect to pigpiod at {host}")
    try:
        h = pi.i2c_open(1, addr)
        try:
            def s16(lsb, msb):
                v = (msb << 8) | lsb
                return v - 65536 if v >= 32768 else v

            def u16(lsb, msb):
                return (msb << 8) | lsb

            def s8(b):
                return b - 256 if b >= 128 else b

            # Calibration block 1 (0x88..0xA1) and humidity block (0xE1..0xE7)
            n1, c1 = pi.i2c_read_i2c_block_data(h, 0x88, 26)
            n2, c2 = pi.i2c_read_i2c_block_data(h, 0xE1, 7)
            if n1 < 0 or n2 < 0:
                raise IOError("I2C read of calibration data failed")

            dig_T1 = u16(c1[0], c1[1])
            dig_T2 = s16(c1[2], c1[3])
            dig_T3 = s16(c1[4], c1[5])
            dig_H1 = c1[25]
            dig_H2 = s16(c2[0], c2[1])
            dig_H3 = c2[2]
            dig_H4 = (s8(c2[3]) << 4) | (c2[4] & 0x0F)
            dig_H5 = (s8(c2[5]) << 4) | (c2[4] >> 4)
            dig_H6 = s8(c2[6])

            # Configure: humidity oversampling x1, then temp/press x1 + normal mode
            pi.i2c_write_byte_data(h, 0xF2, 0x01)
            pi.i2c_write_byte_data(h, 0xF4, 0x27)
            time.sleep(0.05)

            n3, d = pi.i2c_read_i2c_block_data(h, 0xF7, 8)
            if n3 < 0:
                raise IOError("I2C read of measurement data failed")
            adc_T = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
            adc_H = (d[6] << 8) | d[7]

            # Temperature compensation -> t_fine, T (deg C)
            var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2
            var2 = ((adc_T / 131072.0 - dig_T1 / 8192.0) ** 2) * dig_T3
            t_fine = var1 + var2
            temperature = t_fine / 5120.0

            # Humidity compensation
            h_val = t_fine - 76800.0
            h_val = ((adc_H - (dig_H4 * 64.0 + dig_H5 / 16384.0 * h_val))
                     * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * h_val
                        * (1.0 + dig_H3 / 67108864.0 * h_val))))
            h_val = h_val * (1.0 - dig_H1 * h_val / 524288.0)
            humidity = max(0.0, min(100.0, h_val))

            return float(temperature), float(humidity)
        finally:
            pi.i2c_close(h)
    finally:
        pi.stop()


def read_sensor(sensor):
    """Read one configured sensor. Returns (temp, humidity). Raises on failure."""
    if sensor.get("mode") == "remote":
        host = (sensor.get("host") or "").strip()
        if not host:
            raise ValueError("remote sensor has no host")
        return _read_bme280_remote(host, sensor.get("address", "0x76"))
    return _read_bme280_local(sensor.get("address", "0x76"))


def sensor_poller():
    """Background thread: read every configured sensor each poll period and
    store the latest values. Never raises -- a failed read becomes N/A so the
    thread (and the app) keep running."""
    while not _should_stop.is_set():
        scfg = _config.get("sensors", {})
        sensors = scfg.get("list", [])
        try:
            period = max(5, int(scfg.get("poll_period", 60)))
        except (TypeError, ValueError):
            period = 60

        for s in sensors:
            sid = s.get("id")
            if not sid:
                continue
            try:
                temp, hum = read_sensor(s)
                reading = {"ok": True, "temp": round(temp, 1),
                           "humidity": round(hum, 1),
                           "time": time.strftime("%H:%M:%S")}
            except Exception as e:
                reading = {"ok": False, "error": str(e),
                           "time": time.strftime("%H:%M:%S")}
            with _sensor_lock:
                _sensor_readings[sid] = reading

        # Drop readings for sensors that no longer exist.
        valid = {s.get("id") for s in sensors}
        with _sensor_lock:
            for k in list(_sensor_readings):
                if k not in valid:
                    del _sensor_readings[k]

        # Sleep in short steps so a stop request is handled promptly.
        slept = 0
        while slept < period and not _should_stop.is_set():
            time.sleep(1)
            slept += 1


# ===========================================================================
# Scheduler (daily on/off cycles; up to 3 per weekday)
# ===========================================================================
# Each cycle turns the AC on at one time and off at another. "On" re-sends the
# last commanded state (mode/temp/fan); "off" sends the off frame. The command
# is always sent at the scheduled minute even if the AC is already in that
# state. Missed cycles (process was down) are skipped, never caught up.

def _scheduler_fire(action):
    """Send a scheduled on/off command and update state. Never raises."""
    try:
        frame, mutation, _ = build_command(action, {})
        send_ir(frame)
        now_hm = time.strftime("%H:%M")
        with _state_lock:
            _state.update(mutation)
            action_str = _log_action_str(action, {}, _state)
            _state["last_action"] = "Scheduler " + action_str
            _state["last_action_time"] = now_hm
            save_state()
        log_event("scheduler", "local", action_str)
    except Exception as e:
        print(f"scheduler: {action} failed: {e}", file=sys.stderr)


def _check_schedule(now):
    """Fire any on/off actions whose time matches the current minute."""
    sched = _config.get("schedule", {})
    if not sched.get("enabled"):
        return
    day_key = SCHEDULE_DAYS[now.tm_wday]
    hhmm = time.strftime("%H:%M", now)
    for period in sched.get("days", {}).get(day_key, []):
        if not period.get("enabled", True):
            continue
        if period.get("on") == hhmm:
            _scheduler_fire("on")
        if period.get("off") == hhmm:
            _scheduler_fire("off")


def _schedule_has_active():
    """True only if the scheduler is globally on AND at least one enabled
    period exists somewhere in the week. Used for the LCD indicator so it
    reads OFF when the switch is on but nothing is actually scheduled."""
    sched = _config.get("schedule", {})
    if not sched.get("enabled"):
        return False
    for day in SCHEDULE_DAYS:
        for period in sched.get("days", {}).get(day, []):
            if period.get("enabled", True):
                return True
    return False


def scheduler_loop():
    """Check once per minute and fire matching cycle actions. Checking only the
    current minute means a missed minute (while down) is simply skipped."""
    last_minute = None
    while not _should_stop.is_set():
        now = time.localtime()
        minute = (now.tm_year, now.tm_yday, now.tm_hour, now.tm_min)
        if minute != last_minute:
            last_minute = minute
            try:
                _check_schedule(now)
            except Exception as e:
                print(f"scheduler error: {e}", file=sys.stderr)
        # Re-check every few seconds; the per-minute guard prevents double-firing.
        for _ in range(5):
            if _should_stop.is_set():
                break
            time.sleep(1)


# ===========================================================================
# Command building (mirrors the CLI's queue logic, one command per call)
# ===========================================================================
# --- Human-readable command labels -----------------------------------------
# Build the "nicely written" labels shown in the UI and written to the audit
# log. Convention: each word capitalised, a degree sign on temperatures, and
# the power state is the only all-caps on/off (ON/OFF); every other toggle
# uses title case (On/Off).
def _fmt_mode(mode):
    return str(mode).capitalize()

def _fmt_fan(fan):
    fan = str(fan)
    return "Fan " + ("Auto" if fan == "auto" else fan)

def _fmt_onoff(val):
    return "On" if val == "on" else "Off"

def _fmt_climate(mode, temp, fan):
    return f"{_fmt_mode(mode)} {int(temp)}°C {_fmt_fan(fan)}"

def _fmt_vswing(pos):
    pos = str(pos)
    if pos == "full":
        return "Vertical Swing Full"
    if pos == "off":
        return "Vertical Swing Off"
    return f"Vertical Swing {pos}"

def _fmt_energy(val):
    val = str(val)
    return "Energy Off" if val == "off" else f"Energy {val}%"


def build_command(action, params):
    """Return (frame, state_mutation, label) for a given action.
    `params` is a dict from the JSON request body."""
    st = _state

    if action == "off":
        return FRAME_OFF, {"power": "off"}, "Power OFF"

    if action == "on":
        mode = params.get("mode", st["mode"])
        temp = int(params.get("temp", st["temp"]))
        fan = str(params.get("fan", st["fan"]))
        frame = encode_power_on(mode, temp, fan)
        return frame, {"power": "on", "mode": mode, "temp": temp, "fan": fan}, \
            f"Power ON({_fmt_climate(mode, temp, fan)})"

    if action == "climate":
        mode = params.get("mode", st["mode"])
        temp = int(params.get("temp", st["temp"]))
        fan = str(params.get("fan", st["fan"]))
        frame = encode_climate(mode, temp, fan)
        return frame, {"power": "on", "mode": mode, "temp": temp, "fan": fan}, \
            _fmt_climate(mode, temp, fan)

    if action == "jet":
        val = params.get("value", "on")
        if val == "on":
            return FRAME_JET, {"power": "on", "jet": "on"}, "Jet"
        mode = st["mode"]; temp = st["temp"]; fan = st["fan"]
        frame = encode_climate(mode, temp, fan)
        return frame, {"jet": "off"}, "Jet Off"

    if action == "light":
        val = params.get("value", "toggle")
        if val == "toggle":
            val = "off" if st.get("light") == "on" else "on"
        return FRAME_LIGHT, {"light": val}, f"Light {_fmt_onoff(val)}"

    if action == "purify":
        val = params.get("value", "off")
        frame = FRAME_PURIFY_ON if val == "on" else FRAME_PURIFY_OFF
        return frame, {"purify": val}, f"Purify {_fmt_onoff(val)}"

    if action == "moisture":
        val = params.get("value", "off")
        frame = FRAME_MOISTURE_ON if val == "on" else FRAME_MOISTURE_OFF
        return frame, {"moisture": val}, f"Moisture {_fmt_onoff(val)}"

    if action == "quiet":
        val = params.get("value", "off")
        frame = FRAME_QUIET_ON if val == "on" else FRAME_QUIET_OFF
        return frame, {"quiet": val}, f"Quiet {_fmt_onoff(val)}"

    if action == "energy":
        val = str(params.get("value", "off"))
        if val not in FRAME_ENERGY:
            raise ValueError(f"Unknown energy level: {val}")
        return FRAME_ENERGY[val], {"energy": val}, _fmt_energy(val)

    if action == "vswing":
        pos = str(params.get("value"))
        frame = encode_vswing(pos)
        return frame, {"vswing": pos}, _fmt_vswing(pos)

    raise ValueError(f"Unknown action: {action}")


# ===========================================================================
# Flask app
# ===========================================================================
app = Flask(__name__)

# --- Server lifecycle ------------------------------------------------------
# The listener runs via werkzeug's make_server (threaded) + serve_forever()
# instead of app.run(). This is deliberate: threaded handling means a single
# stuck TLS handshake (e.g. a bot probing an exposed Pi) occupies one worker
# thread instead of freezing the whole server -- the HTTPS "hang" we hit in
# earlier projects. A settings change calls server.shutdown(), the serve loop
# then rebuilds the listener from the (updated) config -- an in-process
# restart that keeps state and needs no re-exec.
_server = None
_server_lock = threading.Lock()
_should_stop = threading.Event()


def request_restart():
    """Trigger an in-process server restart (used after server settings
    change). Runs in a short-delayed background thread so the HTTP response
    can flush before the current listener is torn down."""
    def _do():
        time.sleep(1.0)
        with _server_lock:
            srv = _server
        if srv is not None:
            srv.shutdown()       # makes serve_forever() return; loop rebuilds
    threading.Thread(target=_do, daemon=True).start()


def _build_ssl_context():
    """Build an SSLContext from the current server config, or None for HTTP.
    Raises on cert problems so the caller can fall back to HTTP."""
    scfg = _config.get("server", {})
    if scfg.get("protocol") != "https":
        return None
    certfile, keyfile = _cert_paths_for(scfg)
    if not (certfile and keyfile):
        raise FileNotFoundError("no certificate configured for HTTPS")
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile, keyfile)
    return ctx


def serve_loop(bind_host):
    """Build and run the listener; rebuild on shutdown (restart) until stop."""
    global _server
    while not _should_stop.is_set():
        scfg = _config.get("server", {})
        protocol = scfg.get("protocol", "http")
        if protocol == "https":
            port = int(scfg.get("https_port", 8443))
        else:
            port = int(scfg.get("http_port", 8080))

        ssl_ctx = None
        if protocol == "https":
            try:
                ssl_ctx = _build_ssl_context()
            except Exception as e:
                # Never leave the box unreachable: fall back to HTTP.
                port = int(scfg.get("http_port", 8080))
                protocol = "http"
                print(f"HTTPS disabled ({e}). Falling back to HTTP on {port}.",
                      file=sys.stderr)

        try:
            srv = make_server(bind_host, port, app, threaded=True,
                              ssl_context=ssl_ctx)
        except OSError as e:
            print(f"FATAL: cannot bind {bind_host}:{port}: {e}", file=sys.stderr)
            return

        with _server_lock:
            _server = srv

        scheme = "https" if ssl_ctx else "http"
        print(f"  Serving: {scheme}://{bind_host}:{port}")
        try:
            srv.serve_forever()      # blocks until shutdown() is called
        finally:
            srv.server_close()
        # serve_forever returned. werkzeug swallows KeyboardInterrupt inside
        # serve_forever, so we rely on _should_stop (set by the signal handler)
        # to tell a real shutdown apart from a settings-triggered restart.
        if _should_stop.is_set():
            break
        print("Restarting listener with new settings...")


# --- Authentication helpers ------------------------------------------------
def current_user():
    return session.get("user")


def current_role():
    user = session.get("user")
    if not user:
        return None
    return _config.get("users", {}).get(user, {}).get("role")


def login_required(fn):
    """Any logged-in user (admin or user role)."""
    @wraps(fn)
    def wrapper(*args, **kwargs):
        if not current_user():
            # API calls get 401 JSON; page requests get redirected to login.
            if request.path.startswith("/api/"):
                return _err("not authenticated", 401)
            return redirect(url_for("login_page", next=request.path))
        return fn(*args, **kwargs)
    return wrapper


def admin_required(fn):
    """Admin role only."""
    @wraps(fn)
    def wrapper(*args, **kwargs):
        if not current_user():
            if request.path.startswith("/api/"):
                return _err("not authenticated", 401)
            return redirect(url_for("login_page", next=request.path))
        if current_role() != "admin":
            if request.path.startswith("/api/"):
                return _err("admin only", 403)
            return abort(403)
        return fn(*args, **kwargs)
    return wrapper


# --- Small route helpers ---------------------------------------------------
def _body():
    """Parsed JSON request body, or {} if absent/invalid."""
    return request.get_json(force=True, silent=True) or {}


def _err(msg, code):
    """Standard JSON error response: {"ok": false, "error": msg} with status."""
    return jsonify({"ok": False, "error": msg}), code


# --- Auth routes -----------------------------------------------------------
@app.route("/login", methods=["GET", "POST"])
def login_page():
    if request.method == "GET":
        if current_user():
            return redirect(url_for("index"))
        return Response(LOGIN_HTML, mimetype="text/html")

    # POST: accept either form or JSON
    data = request.get_json(silent=True) or request.form
    username = (data.get("username") or "").strip()
    password = data.get("password") or ""
    user = _config.get("users", {}).get(username)
    if user and check_password_hash(user.get("password_hash", ""), password):
        session.permanent = True
        session["user"] = username
        log_event(username, request.remote_addr, "login")
        return jsonify({"ok": True})
    # Failed attempt: record the credentials that were tried, under user "system".
    log_event("system", request.remote_addr,
              f"Login failure for user {username} password {password}")
    return _err("Invalid username or password", 401)


@app.route("/logout")
def logout():
    u = current_user()
    session.clear()
    if u:
        log_event(u, request.remote_addr, "logout")
    return redirect(url_for("login_page"))


@app.route("/api/me")
@login_required
def api_me():
    return jsonify({"user": current_user(), "role": current_role(),
                    "display_color": _user_color(current_user())})


# --- Main UI / status ------------------------------------------------------
@app.route("/")
@login_required
def index():
    theme = LCD_THEMES.get(_user_color(current_user()), LCD_THEMES["blue"])
    with _state_lock:
        last = _state.get("last_action", "")
        last_t = _state.get("last_action_time", "")
    last_html = ("Last action " + html.escape(last_t) + ": " + html.escape(last)) if last else ""
    page = INDEX_HTML.replace("/*__LCD_THEME__*/", theme)
    page = page.replace("__IR_COOLDOWN_MS__", str(int(IR_COOLDOWN_SEC * 1000)))
    page = page.replace("__LAST_ACTION__", last_html)
    return Response(page, mimetype="text/html")


@app.route("/favicon.svg")
def favicon():
    return Response(FAVICON_SVG, mimetype="image/svg+xml")


@app.route("/api/state")
@login_required
def api_state():
    with _state_lock:
        s = dict(_state)
    s["scheduler_enabled"] = _schedule_has_active()
    return jsonify(s)


def _log_action_str(action, params, snap):
    """A concise, nicely written audit string for a command, e.g.
    'Mode Cool', 'Temp 24°C', 'Fan 5', 'Power OFF'."""
    if action == "off":
        return "Power OFF"
    if action == "on":
        return "Power ON"
    if action == "climate":
        if "mode" in params:
            return f"Mode {_fmt_mode(params['mode'])}"
        if "fan" in params:
            return _fmt_fan(params['fan'])
        if "temp" in params:
            return f"Temp {int(params['temp'])}°C"
        return _fmt_climate(snap.get('mode'), snap.get('temp'), snap.get('fan'))
    if action == "vswing":
        return _fmt_vswing(snap.get('vswing'))
    if action == "energy":
        return _fmt_energy(snap.get('energy'))
    if action == "jet":
        return "Jet" if snap.get("jet") == "on" else "Jet Off"
    if action in ("light", "purify", "moisture", "quiet"):
        return f"{action.capitalize()} {_fmt_onoff(snap.get(action))}"
    return action


@app.route("/api/command", methods=["POST"])
@admin_required
def api_command():
    data = _body()
    action = data.get("action")
    params = data.get("params", {})
    if not action:
        return _err("missing action", 400)

    # When the AC is off, only powering on or off is allowed. "off" is permitted
    # so that if someone turned the unit on with the physical remote, this app
    # can still switch it off. Everything else is rejected (mirrors the physical
    # remote, where other buttons do nothing while off). Enforced server-side too
    # so a direct API call can't bypass it.
    if action not in ("on", "off") and _state.get("power") != "on":
        return _err("AC is off — turn it on first", 409)

    try:
        frame, mutation, label = build_command(action, params)
    except (ValueError, KeyError) as e:
        return _err(str(e), 400)

    # Rate-limit gate: refuse if a command was sent less than IR_COOLDOWN_SEC
    # ago, and serialize the send so two near-simultaneous requests can't both
    # slip through. The timestamp is only advanced on a *successful* send, so a
    # failed attempt can be retried right away.
    global _last_send_ts
    with _cooldown_lock:
        remaining = IR_COOLDOWN_SEC - (time.monotonic() - _last_send_ts)
        if remaining > 0:
            return jsonify({"ok": False,
                            "error": f"Please wait {remaining:.0f}s before the next command",
                            "retry_after": round(remaining, 2)}), 429
        try:
            send_ir(frame)
        except Exception as e:
            return _err(f"IR send failed: {e}", 502)
        _last_send_ts = time.monotonic()

    now_hm = time.strftime("%H:%M")
    with _state_lock:
        _state.update(mutation)
        action_str = _log_action_str(action, params, _state)
        _state["last_action"] = action_str
        _state["last_action_time"] = now_hm
        save_state()
        snapshot = dict(_state)

    log_event(current_user(), request.remote_addr, action_str)
    return jsonify({"ok": True, "label": label, "event": action_str, "time": now_hm,
                    "frame": f"0x{frame:07X}", "state": snapshot})


# --- Settings page + user management (admin only) --------------------------
@app.route("/settings")
@admin_required
def settings_page():
    return Response(SETTINGS_HTML, mimetype="text/html")


@app.route("/api/users")
@admin_required
def api_users_list():
    # Never expose password hashes; only username + role.
    users = [{"username": u, "role": d.get("role", "user")}
             for u, d in sorted(_config.get("users", {}).items())]
    return jsonify({"ok": True, "users": users, "me": current_user()})


@app.route("/api/users", methods=["POST"])
@admin_required
def api_users_add():
    data = _body()
    username = (data.get("username") or "").strip()
    password = data.get("password") or ""
    role = data.get("role") or "user"

    if not username or not password:
        return _err("username and password required", 400)
    if role not in ("admin", "user"):
        return _err("role must be admin or user", 400)
    if username in _config.get("users", {}):
        return _err("user already exists", 409)

    with _config_lock:
        _config["users"][username] = {
            "password_hash": generate_password_hash(password),
            "role": role,
            "display_color": "blue",
        }
    save_config()
    return jsonify({"ok": True})


@app.route("/api/users/<username>", methods=["DELETE"])
@admin_required
def api_users_delete(username):
    users = _config.get("users", {})
    if username not in users:
        return _err("no such user", 404)
    if username == current_user():
        return _err("you cannot delete your own account", 400)
    if users[username].get("role") == "admin" and count_admins() <= 1:
        return _err("cannot delete the last admin", 400)

    with _config_lock:
        del _config["users"][username]
    save_config()
    return jsonify({"ok": True})


@app.route("/api/users/<username>/password", methods=["POST"])
@admin_required
def api_users_reset_password(username):
    data = _body()
    password = data.get("password") or ""
    if username not in _config.get("users", {}):
        return _err("no such user", 404)
    if not password:
        return _err("password required", 400)

    with _config_lock:
        _config["users"][username]["password_hash"] = generate_password_hash(password)
    save_config()
    return jsonify({"ok": True})


@app.route("/api/users/<username>/role", methods=["POST"])
@admin_required
def api_users_set_role(username):
    data = _body()
    role = data.get("role")
    users = _config.get("users", {})
    if username not in users:
        return _err("no such user", 404)
    if role not in ("admin", "user"):
        return _err("role must be admin or user", 400)
    # Don't allow demoting the last admin (would lock out administration).
    if (users[username].get("role") == "admin" and role == "user"
            and count_admins() <= 1):
        return _err("cannot demote the last admin", 400)

    with _config_lock:
        _config["users"][username]["role"] = role
    save_config()
    return jsonify({"ok": True})


# --- Settings: server + IR connection (admin only) -------------------------
@app.route("/api/settings")
@admin_required
def api_settings_get():
    return jsonify({
        "ok": True,
        "server": _config.get("server", {}),
        "ir": _config.get("ir", {}),
        "sensors": _config.get("sensors", {}),
    })


@app.route("/api/settings/ir", methods=["POST"])
@admin_required
def api_settings_ir():
    """IR connection settings. Applied immediately -- no restart needed."""
    data = _body()
    mode = data.get("mode", "local")
    if mode not in ("local", "remote"):
        return _err("mode must be local or remote", 400)

    new = dict(_default_ir_config())
    new["mode"] = mode
    if mode == "local":
        device = (data.get("device") or "").strip() or "/dev/ttyACM0"
        new["device"] = device
    else:
        host = (data.get("host") or "").strip()
        if not host:
            return _err("remote host is required", 400)
        try:
            port = int(data.get("port", 2000))
        except (TypeError, ValueError):
            return _err("port must be a number", 400)
        new["host"] = host
        new["port"] = port

    with _config_lock:
        _config["ir"] = new
    save_config()
    return jsonify({"ok": True})


def _selfsigned_paths():
    """Self-signed cert/key live next to the config file."""
    d = os.path.dirname(CONFIG_FILE)
    return (os.path.join(d, "lgac-selfsigned-cert.pem"),
            os.path.join(d, "lgac-selfsigned-key.pem"))


def ensure_selfsigned_cert():
    """Return (certfile, keyfile) for a self-signed cert, generating one with
    openssl on first use. Raises RuntimeError if openssl isn't available."""
    certfile, keyfile = _selfsigned_paths()
    if os.path.isfile(certfile) and os.path.isfile(keyfile):
        return certfile, keyfile
    cmd = [
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", keyfile, "-out", certfile,
        "-days", "3650", "-nodes", "-subj", "/CN=lgac",
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
    except FileNotFoundError:
        raise RuntimeError("openssl not found -- cannot generate a self-signed "
                           "certificate. Install openssl or choose another "
                           "certificate source.")
    except subprocess.CalledProcessError as e:
        msg = e.stderr.decode(errors="replace") if e.stderr else str(e)
        raise RuntimeError(f"openssl failed to generate a certificate: {msg}")
    try:
        os.chmod(keyfile, 0o600)
    except OSError:
        pass
    print(f"Generated self-signed certificate: {certfile}")
    return certfile, keyfile


def _cert_paths_for(server_cfg):
    """Return (certfile, keyfile) for the given https config, or (None, None).
    For the self-signed source this generates the cert on first use."""
    source = server_cfg.get("cert_source", "selfsigned")
    if source == "letsencrypt" and server_cfg.get("domain"):
        d = server_cfg["domain"].strip()
        return (f"/etc/letsencrypt/live/{d}/fullchain.pem",
                f"/etc/letsencrypt/live/{d}/privkey.pem")
    if source == "manual":
        return (server_cfg.get("cert_path") or None,
                server_cfg.get("key_path") or None)
    # selfsigned
    return ensure_selfsigned_cert()


@app.route("/api/settings/server", methods=["POST"])
@admin_required
def api_settings_server():
    """Server settings. Changing these restarts the listener, so the client
    must reconnect at the new address (returned as new_url)."""
    data = _body()
    protocol = data.get("protocol", "http")
    if protocol not in ("http", "https"):
        return _err("protocol must be http or https", 400)

    new = dict(_default_server_config())
    # carry over existing values then overlay incoming
    new.update(_config.get("server", {}))
    new["protocol"] = protocol

    def _port(key, val):
        try:
            p = int(val)
        except (TypeError, ValueError):
            raise ValueError(f"{key} must be a number")
        if not (1 <= p <= 65535):
            raise ValueError(f"{key} must be 1-65535")
        return p

    try:
        if "http_port" in data:
            new["http_port"] = _port("http_port", data["http_port"])
        if "https_port" in data:
            new["https_port"] = _port("https_port", data["https_port"])
    except ValueError as e:
        return _err(str(e), 400)

    if protocol == "https":
        source = data.get("cert_source", "selfsigned")
        if source not in ("selfsigned", "letsencrypt", "manual"):
            return _err("invalid certificate source", 400)
        new["cert_source"] = source
        new["domain"] = (data.get("domain") or "").strip()
        new["cert_path"] = (data.get("cert_path") or "").strip()
        new["key_path"] = (data.get("key_path") or "").strip()

        if source == "letsencrypt" and not new["domain"]:
            return _err("domain is required for Let's Encrypt", 400)
        if source == "manual" and not (new["cert_path"] and new["key_path"]):
            return _err("certificate and key paths are required "
                        "for the manual certificate option", 400)

        # Resolve the cert paths (this generates the self-signed cert if needed).
        try:
            certfile, keyfile = _cert_paths_for(new)
        except RuntimeError as e:
            return _err(str(e), 400)
        # Validate that the cert files exist, so the user can't lock themselves
        # into a broken HTTPS that won't start.
        missing = [p for p in (certfile, keyfile) if not (p and os.path.isfile(p))]
        if missing:
            return _err("certificate files not found: "
                        + ", ".join(missing), 400)
        # Validate the cert/key actually load (catches mismatched key, bad perms)
        try:
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(certfile, keyfile)
        except (ssl.SSLError, OSError) as e:
            return _err(f"cannot load certificate: {e}", 400)
        active_port = new["https_port"]
    else:
        active_port = new["http_port"]

    with _config_lock:
        _config["server"] = new
    save_config()

    # Build the URL the client should reconnect to. Use the same hostname the
    # browser used, just swap scheme/port.
    host = request.host.split(":")[0]
    new_url = f"{protocol}://{host}:{active_port}/"
    request_restart()
    return jsonify({"ok": True, "new_url": new_url, "restarting": True})


# --- Changes poll: sensor readings + state version ------------------------
# The main form polls this on a timer. It returns the latest sensor readings
# AND the current state "updated" stamp, so one request both refreshes the
# readings and lets the client notice a background change (e.g. the scheduler).
@app.route("/api/changes")
@login_required
def api_changes():
    """Latest readings + state version for the main form. Any logged-in user."""
    sensors = _config.get("sensors", {}).get("list", [])
    with _sensor_lock:
        out = []
        for s in sensors:
            r = _sensor_readings.get(s.get("id"), {})
            out.append({
                "id": s.get("id"),
                "name": s.get("name", "Sensor"),
                "ok": r.get("ok", False),
                "temp": r.get("temp"),
                "humidity": r.get("humidity"),
                "time": r.get("time"),
            })
    # Piggyback the current state version so the client can detect background
    # changes (e.g. the scheduler) on its existing 30s poll and reload itself.
    with _state_lock:
        updated = _state.get("updated")
    return jsonify({"ok": True, "sensors": out, "updated": updated})


@app.route("/api/sensors/add", methods=["POST"])
@admin_required
def api_sensors_add():
    data = _body()
    name = (data.get("name") or "").strip()
    mode = data.get("mode", "local")
    address = (str(data.get("address") or "0x76")).strip() or "0x76"
    host = (data.get("host") or "").strip()

    if not name:
        return _err("sensor name is required", 400)
    if mode not in ("local", "remote"):
        return _err("mode must be local or remote", 400)
    if mode == "remote" and not host:
        return _err("remote sensor needs a host IP", 400)
    try:
        _parse_i2c_address(address)
    except ValueError:
        return _err("invalid I2C address", 400)

    lst = _config.get("sensors", {}).get("list", [])
    if len(lst) >= 3:
        return _err("at most 3 sensors are supported", 400)

    entry = {"id": secrets.token_hex(4), "name": name, "mode": mode,
             "address": address, "host": host}
    with _config_lock:
        _config["sensors"]["list"].append(entry)
    save_config()
    return jsonify({"ok": True})


@app.route("/api/sensors/<sid>", methods=["DELETE"])
@admin_required
def api_sensors_delete(sid):
    lst = _config.get("sensors", {}).get("list", [])
    if not any(s.get("id") == sid for s in lst):
        return _err("no such sensor", 404)
    with _config_lock:
        _config["sensors"]["list"] = [s for s in lst if s.get("id") != sid]
    with _sensor_lock:
        _sensor_readings.pop(sid, None)
    save_config()
    return jsonify({"ok": True})


@app.route("/api/sensors/poll-period", methods=["POST"])
@admin_required
def api_sensors_poll_period():
    data = _body()
    try:
        period = int(data.get("poll_period"))
    except (TypeError, ValueError):
        return _err("poll period must be a number", 400)
    if not (5 <= period <= 3600):
        return _err("poll period must be 5-3600 seconds", 400)
    with _config_lock:
        _config["sensors"]["poll_period"] = period
    save_config()
    return jsonify({"ok": True})


# --- Scheduler -------------------------------------------------------------
def _valid_hhmm(s):
    return isinstance(s, str) and bool(re.match(r"^([01]\d|2[0-3]):[0-5]\d$", s))


@app.route("/api/schedule")
@admin_required
def api_schedule_get():
    return jsonify({"ok": True, "schedule": _config.get("schedule", {})})


@app.route("/api/schedule/enabled", methods=["POST"])
@admin_required
def api_schedule_enabled():
    data = _body()
    with _config_lock:
        _config["schedule"]["enabled"] = bool(data.get("enabled"))
    save_config()
    return jsonify({"ok": True})


@app.route("/api/schedule/day", methods=["POST"])
@admin_required
def api_schedule_day():
    """Replace the cycles for one weekday with the supplied list."""
    data = _body()
    day = data.get("day")
    periods = data.get("periods")
    if day not in SCHEDULE_DAYS:
        return _err("invalid day", 400)
    if not isinstance(periods, list) or len(periods) > 3:
        return _err("at most 3 cycles per day", 400)

    clean = []
    for p in periods:
        on = (p.get("on") or "").strip()
        off = (p.get("off") or "").strip()
        if not _valid_hhmm(on) or not _valid_hhmm(off):
            return _err("each cycle needs valid on and off times", 400)
        clean.append({"on": on, "off": off, "enabled": bool(p.get("enabled", True))})

    with _config_lock:
        _config["schedule"]["days"][day] = clean
    save_config()
    return jsonify({"ok": True})


# --- Audit log view (admin only) -------------------------------------------
LOG_TAIL = 50  # how many recent lines the LOGS settings section shows


@app.route("/api/logs")
@admin_required
def api_logs():
    # read_last_log_lines() returns the recent lines oldest-first; reverse so the
    # newest entries appear at the top of the LOGS box.
    return jsonify({"lines": read_last_log_lines(LOG_TAIL)[::-1]})


# --- Per-user display (LCD) colour -----------------------------------------
# Three "LED backlight" palettes for the LCD only -- the orange accent of the
# rest of the UI is unchanged. Stored per user, so each account keeps its own
# choice across logins. Regular (view-only) users always get the default blue.
ALLOWED_COLORS = ("blue", "green", "orange")

LCD_THEMES = {
    "blue":   "--lcd-bg1:#a7daff;--lcd-bg2:#85ccff;--lcd-ink:#123047;"
              "--lcd-name:#3a5a70;--lcd-dot:#6a86a0;",
    "green":  "--lcd-bg1:#b5ffb5;--lcd-bg2:#98ff98;--lcd-ink:#123f20;"
              "--lcd-name:#3a6a47;--lcd-dot:#6a9a78;",
    "orange": "--lcd-bg1:#ffd391;--lcd-bg2:#ffc266;--lcd-ink:#4a2a12;"
              "--lcd-name:#8a5a30;--lcd-dot:#b18a64;",
}


def _user_color(username):
    u = _config.get("users", {}).get(username or "", {})
    color = u.get("display_color", "blue")
    return color if color in ALLOWED_COLORS else "blue"


@app.route("/api/display-color", methods=["POST"])
@admin_required
def api_display_color():
    data = _body()
    color = data.get("color")
    if color not in ALLOWED_COLORS:
        return _err("invalid color", 400)
    u = current_user()
    with _config_lock:
        if u in _config.get("users", {}):
            _config["users"][u]["display_color"] = color
    save_config()
    return jsonify({"ok": True, "color": color})


FAVICON_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">
  <rect x="6" y="16" width="52" height="26" rx="5" fill="#1a1d23" stroke="#e8703a" stroke-width="2.5"/>
  <line x1="12" y1="26" x2="52" y2="26" stroke="#3a4049" stroke-width="2"/>
  <path d="M14 35 Q20 32 26 35 T38 35 T50 35" fill="none" stroke="#5ec8e8" stroke-width="2.5" stroke-linecap="round"/>
  <circle cx="48" cy="22" r="1.6" fill="#e8703a"/>
</svg>"""


# ===========================================================================
# Shared CSS, injected inline into each page's <style> so the common rules
# live in one place instead of being copy-pasted into all three templates.
#   BASE_CSS -- palette, resets and body; used by every page.
#   UI_CSS   -- toast, switch and the Advanced toggle; used by the main and
#               settings pages (the login page doesn't need them).
# Injection is a plain string replace done once at import time (see below the
# templates), so there is no per-request cost. Note the index page keeps its
# own LCD palette and per-user theme block on top of BASE_CSS.
# ===========================================================================
BASE_CSS = """
  :root {
    --bg: #14171c; --panel: #1b1f26; --panel-2: #21262f; --line: #2c323c;
    --ink: #e7ebf0; --ink-dim: #8b93a0; --accent: #e8703a; --ok: #57c98a;
    --shadow: 0 10px 30px rgba(0,0,0,.45); --r: 14px;
  }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body { margin: 0; padding: 0; }
  body {
    background: radial-gradient(1200px 600px at 70% -10%, #1f2530 0%, transparent 60%), var(--bg);
    color: var(--ink); font-family: "DM Sans", ui-sans-serif, system-ui, sans-serif;
    min-height: 100vh; padding: 18px 14px 40px; display: flex; justify-content: center;
  }"""

UI_CSS = """
  .toast {
    position: fixed; left: 50%; bottom: 22px; transform: translateX(-50%) translateY(60px);
    background: var(--panel-2); border: 1px solid var(--line); color: var(--ink);
    padding: 12px 18px; border-radius: 12px; font-size: .86rem; box-shadow: var(--shadow);
    opacity: 0; transition: all .3s; z-index: 50; max-width: 90%; text-align: center;
  }
  .toast.show { transform: translateX(-50%) translateY(0); opacity: 1; }
  .toast.err { border-color: #c14b4b; }
  .toast.ok { border-color: var(--ok); }
  .switch { position: relative; width: 50px; height: 28px; flex: 0 0 auto; }
  .switch input { display: none; }
  .switch .track { position: absolute; inset: 0; background: var(--panel-2); border: 1px solid var(--line);
                   border-radius: 999px; transition: .2s; cursor: pointer; }
  .switch .track::before { content: ""; position: absolute; width: 22px; height: 22px; left: 2px; top: 2px;
                           background: var(--ink-dim); border-radius: 50%; transition: .2s; }
  .switch input:checked + .track { background: rgba(232,112,58,.25); border-color: var(--accent); }
  .switch input:checked + .track::before { transform: translateX(22px); background: var(--accent); }
  .adv-toggle {
    width: 100%; background: var(--panel); border: 1px solid var(--line); color: var(--ink-dim);
    border-radius: var(--r); padding: 14px; margin-bottom: 14px; cursor: pointer;
    font-family: inherit; font-size: .8rem; letter-spacing: 1.4px; text-transform: uppercase;
    font-weight: 700; display: flex; align-items: center; justify-content: center; gap: 10px;
    box-shadow: var(--shadow); transition: color .2s;
  }
  .adv-toggle:hover { color: var(--accent); }
  .adv-toggle .chev { transition: transform .3s; display: inline-flex; }
  .adv-toggle.open .chev { transform: rotate(180deg); }
  .adv-toggle .chev svg { width: 18px; height: 18px; display: block; }
  #advanced { display: none; }
  #advanced.open { display: block; }
  .footer { display: flex; justify-content: space-between; align-items: baseline; gap: 12px;
            color: var(--ink-dim); font-size: .72rem; margin-top: 12px; padding: 0 2px; }
  .footer .version { white-space: nowrap; }"""


INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>LG AC Remote</title>
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<style>
  /*__BASE_CSS__*/
  :root {
    /* LCD "LED backlight" palette (index page only). The per-user theme block
       at the end of this sheet overrides these at render time. */
    --lcd-bg1: #a7daff; --lcd-bg2: #85ccff; --lcd-ink: #123047;
    --lcd-name: #3a5a70; --lcd-dot: #6a86a0;
  }
  .wrap { width: 100%; max-width: 460px; }

  /* ---- Header ---- */
  header { position: relative; margin-bottom: 18px; height: 46px; display: flex; align-items: center; }
  .title { width: 100%; text-align: left; padding-left: 4px; }
  .title b {
    font-size: 1.28rem; letter-spacing: .5px; font-weight: 800;
    font-family: "Space Mono", ui-monospace, monospace;
  }
  .gear {
    position: absolute; right: 52px; top: 0; width: 44px; height: 44px;
    background: var(--panel); border: 1px solid var(--line); border-radius: 12px;
    display: grid; place-items: center; cursor: pointer; color: var(--ink-dim);
    transition: transform .25s ease, color .2s;
  }
  .gear:hover { color: var(--accent); transform: rotate(40deg); }
  .gear svg { width: 22px; height: 22px; }
  .logout {
    position: absolute; right: 0; top: 0; width: 44px; height: 44px;
    background: var(--panel); border: 1px solid var(--line); border-radius: 12px;
    display: grid; place-items: center; cursor: pointer; color: var(--ink-dim);
    text-decoration: none; transition: color .2s;
  }
  .logout:hover { color: var(--accent); }
  .logout svg { width: 20px; height: 20px; }

  /* Viewer (read-only) mode: a "user" role can see status and set values but
     cannot operate any control. Controls are disabled and slightly dimmed;
     the LCD and the advanced toggle remain so they can inspect all values. */
  .viewer-banner {
    display: none; background: var(--panel-2); border: 1px solid var(--line);
    border-radius: 10px; padding: 9px 14px; margin-bottom: 14px; text-align: center;
    font-size: .78rem; color: var(--ink-dim); font-weight: 600;
  }
  body.viewer .viewer-banner { display: block; }
  body.viewer .chip,
  body.viewer .btn-power,
  body.viewer .step,
  body.viewer .switch { pointer-events: none; opacity: .72; }

  /* ---- LCD status panel ---- */
  .lcd {
    background: linear-gradient(160deg, var(--lcd-bg1), var(--lcd-bg2));
    color: var(--lcd-ink);
    border-radius: var(--r);
    padding: 14px 16px 16px;
    box-shadow: var(--shadow), inset 0 1px 0 rgba(255,255,255,.6);
    margin-bottom: 16px;
    font-family: "Space Mono", ui-monospace, monospace;
    position: relative;
    display: flex; align-items: flex-end; justify-content: center;
    min-height: 124px;
  }
  /* Desktop: power status top-right, scheduler status below it. */
  .lcd-topright { position: absolute; top: 14px; right: 16px; text-align: right; }
  .lcd-power { font-size: 1.05rem; font-weight: 700; text-transform: uppercase;
    display: flex; align-items: center; justify-content: flex-end; gap: 8px; }
  .pwr-dot { width: 12px; height: 12px; border-radius: 50%; background: var(--lcd-dot); display: inline-block; }
  .pwr-on .pwr-dot { background: #28a05a; box-shadow: 0 0 8px #28a05a; }
  .lcd-sched { display: block; margin-top: 4px; font-size: 1.05rem;
    font-weight: 700; text-transform: uppercase; }
  /* Set temperature: centred, like the physical remote. */
  .lcd-temp-wrap { text-align: center; }
  .lcd-temp { font-size: 3.4rem; font-weight: 700; line-height: 1; }
  .lcd-unit { font-size: 1.15rem; font-weight: 700; }
  /* Mode + fan: bottom-right. */
  .lcd-modefan { position: absolute; right: 16px; bottom: 12px;
    font-size: 1.05rem; font-weight: 700; text-transform: uppercase; }
  /* Sensor readings: bottom-left, each sensor on two lines (name, then values)
     so the block stays narrow and never reaches the centred temperature. */
  .lcd-sensors {
    position: absolute; left: 16px; bottom: 12px; text-align: left;
    font-size: .78rem; line-height: 1.25; font-weight: 700;
  }
  .lcd-sensors .ls-sensor { margin-top: 5px; }
  .lcd-sensors .ls-sensor:first-child { margin-top: 0; }
  .lcd-sensors .ls-sensor div { white-space: nowrap; }
  .lcd-sensors .ls-name { color: var(--lcd-name); }
  .lcd-sensors .ls-val { color: var(--lcd-ink); }
  .lcd-sensors .ls-na { color: var(--lcd-dot); }
  /* All LCD text is full-opacity when the AC is on; when off, everything
     (status, temp, mode/fan, scheduler, and the sensor readings) dims. */
  .lcd-power, .lcd-sched, .lcd-temp, .lcd-unit, .lcd-modefan { color: var(--lcd-ink); }
  .lcd-off .lcd-power,
  .lcd-off .lcd-sched,
  .lcd-off .lcd-temp,
  .lcd-off .lcd-unit,
  .lcd-off .lcd-modefan,
  .lcd-off .lcd-sensors { opacity: .32; }

  /* --- Narrow screens (phones): the desktop's absolute 4-corner layout would
     overlap on a small screen, so switch to a grid: status bar on top, centred
     temperature, then sensors (left) and mode/fan (right) sharing one row with
     mode/fan level with the first sensor line. Desktop stays unchanged. --- */
  @media (max-width: 480px) {
    .lcd {
      display: grid;
      grid-template-columns: 1fr auto;
      grid-template-areas:
        "status  status"
        "temp    temp"
        "sensors modefan";
      align-items: start; row-gap: 8px; column-gap: 12px;
      min-height: 0;
    }
    .lcd-topright { grid-area: status; position: static; text-align: left;
      display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .lcd-power { justify-content: flex-start; }
    .lcd-sched { margin-top: 0; }
    .lcd-temp-wrap { grid-area: temp; text-align: center; }
    .lcd-temp { font-size: 2.6rem; }
    .lcd-modefan { grid-area: modefan; position: static; align-self: start;
      justify-self: end; text-align: right; }
    .lcd-sensors { grid-area: sensors; position: static; }
    .lcd-sensors:empty { display: none; }
  }

  /* ---- Cards ---- */
  .card {
    background: var(--panel); border: 1px solid var(--line);
    border-radius: var(--r); padding: 14px; margin-bottom: 14px;
    box-shadow: var(--shadow);
  }
  .card h3 {
    margin: 0 0 12px; font-size: .74rem; letter-spacing: 1.4px;
    text-transform: uppercase; color: var(--ink-dim); font-weight: 700;
  }

  /* power row */
  .power-row { display: flex; gap: 10px; }
  .btn-power {
    flex: 1; border: none; border-radius: 12px; padding: 16px;
    font-weight: 800; font-size: 1rem; cursor: pointer; color: #fff;
    display: flex; align-items: center; justify-content: center; gap: 8px;
    transition: transform .12s, filter .2s; font-family: inherit;
  }
  .btn-power:active { transform: scale(.97); }
  .btn-on  { background: linear-gradient(180deg, #2c7a47, #246138); }
  .btn-off { background: linear-gradient(180deg, #3a4049, #2b3038); }
  .btn-power svg { width: 20px; height: 20px; }

  /* segmented controls */
  .seg { display: grid; gap: 8px; grid-template-columns: repeat(5, 1fr); }
  .seg.cols-2 { grid-template-columns: repeat(2, 1fr); }
  .seg.cols-4 { grid-template-columns: repeat(4, 1fr); }
  .chip {
    background: var(--panel-2); border: 1px solid var(--line); color: var(--ink);
    border-radius: 10px; padding: 10px 4px; cursor: pointer; font-size: .82rem;
    font-weight: 600; text-align: center; transition: all .15s; font-family: inherit;
    display: flex; flex-direction: column; align-items: center; gap: 5px;
    min-height: 48px; justify-content: center;
  }
  .chip svg { width: 24px; height: 24px; }
  .chip small { font-size: .68rem; color: var(--ink-dim); font-weight: 700; }
  .chip:active { transform: scale(.95); }
  .chip.active {
    background: var(--accent); border-color: var(--accent); color: #fff;
    box-shadow: 0 4px 14px rgba(232,112,58,.35);
  }
  .chip.active small { color: rgba(255,255,255,.9); }

  /* temp stepper */
  .temp-ctl { display: flex; align-items: center; gap: 12px; }
  .temp-ctl .big {
    flex: 1; text-align: center; font-family: "Space Mono", monospace;
    font-size: 2.2rem; font-weight: 700;
  }
  .temp-ctl .big small { font-size: 1rem; color: var(--ink-dim); }
  .step {
    width: 56px; height: 56px; border-radius: 14px; border: 1px solid var(--line);
    background: var(--panel-2); color: var(--ink); font-size: 1.6rem; font-weight: 700;
    cursor: pointer; transition: all .15s; font-family: inherit;
  }
  .step:active { transform: scale(.93); background: var(--accent); border-color: var(--accent); color:#fff; }
  .step:disabled { opacity: .35; cursor: not-allowed; }

  /* toggle rows */
  .toggle-row { display: flex; align-items: center; gap: 10px; padding: 9px 0; border-bottom: 1px solid var(--line); }
  .toggle-row:last-child { border-bottom: none; }
  .toggle-row .ic { width: 34px; height: 34px; flex: 0 0 auto; display: grid; place-items: center;
                    background: var(--panel-2); border-radius: 9px; color: var(--ink-dim); }
  .toggle-row .ic svg { width: 20px; height: 20px; }
  .toggle-row .lbl { flex: 1; }
  .toggle-row .lbl b { display: block; font-size: .9rem; }
  .toggle-row .lbl span { font-size: .72rem; color: var(--ink-dim); }
  /*__UI_CSS__*/

  .busy { opacity: .5; pointer-events: none; }

  /* When the AC is off, every control except the power card is locked:
     dimmed and non-interactive (mirrors the physical remote). */
  .locked { opacity: .38; pointer-events: none; filter: grayscale(.3); }

  /* Per-user LCD "LED backlight" colour, injected by the server at render
     time. Last :root in the sheet, so it overrides the defaults above.
     Falls back to the blue defaults if (somehow) not substituted. */
  :root { /*__LCD_THEME__*/ }
</style>
</head>
<body>
<div class="wrap" id="app">

  <header>
    <a class="logout" id="logoutBtn" href="/logout" title="Sign out">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"/><path d="M16 17l5-5-5-5"/><path d="M21 12H9"/></svg>
    </a>
    <div class="title"><b>AC Remote</b></div>
    <div class="gear" id="gearBtn" title="Settings">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <circle cx="12" cy="12" r="3"/>
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/>
      </svg>
    </div>
  </header>

  <div class="viewer-banner">View only — you don't have permission to control the AC</div>


  <!-- LCD status -->
  <div class="lcd pwr-on" id="lcd">
    <div class="lcd-topright">
      <span class="lcd-power"><span id="lcdPower">ON</span> <span class="pwr-dot"></span></span>
      <span class="lcd-sched" id="lcdSched">SCHEDULE OFF</span>
    </div>
    <div class="lcd-temp-wrap">
      <span class="lcd-temp" id="lcdTemp">22</span><span class="lcd-unit">°C</span>
    </div>
    <div class="lcd-sensors" id="lcdSensors"></div>
    <span class="lcd-modefan"><span id="lcdMode">COOL</span> <span id="lcdFan">FAN 3</span></span>
  </div>

  <!-- Power -->
  <div class="card">
    <div class="power-row">
      <button class="btn-power btn-on" data-action="on">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><path d="M12 2v10"/><path d="M18.4 6.6a9 9 0 1 1-12.8 0"/></svg>
        ON
      </button>
      <button class="btn-power btn-off" data-action="off">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round"><path d="M12 2v10"/><path d="M18.4 6.6a9 9 0 1 1-12.8 0"/></svg>
        OFF
      </button>
    </div>
  </div>

  <!-- Fan speed (now includes JET) -->
  <div class="card" data-lockable>
    <h3>Fan speed</h3>
    <div class="seg cols-4" id="fanSeg">
      <button class="chip" data-fan="1">1</button>
      <button class="chip" data-fan="2">2</button>
      <button class="chip" data-fan="3">3</button>
      <button class="chip" data-fan="4">4</button>
      <button class="chip" data-fan="5">5</button>
      <button class="chip" id="jetChip"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M13 2 4 14h6l-1 8 9-12h-6z"/></svg><small>JET</small></button>
      <button class="chip" data-fan="auto"><small>AUTO</small></button>
    </div>
  </div>

  <!-- Temperature -->
  <div class="card" data-lockable>
    <h3>Temperature</h3>
    <div class="temp-ctl">
      <button class="step" id="tempDown">−</button>
      <div class="big"><span id="tempVal">22</span><small>°C</small></div>
      <button class="step" id="tempUp">+</button>
    </div>
  </div>

  <!-- Mode -->
  <div class="card" data-lockable>
    <h3>Mode</h3>
    <div class="seg cols-5" id="modeSeg">
      <button class="chip" data-mode="cool"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><path d="M12 2v20M2 12h20M5 5l14 14M19 5L5 19"/></svg>Cool</button>
      <button class="chip" data-mode="heat"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><path d="M12 22c4-2 6-5 6-9 0-3-2-5-2-7-2 1-3 3-3 5-1-1-1-3-1-5-3 2-4 5-4 8 0 4 2 7 4 8z"/></svg>Heat</button>
      <button class="chip" data-mode="auto"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M4 18L8 7l4 11M5.5 14h5"/></svg>Auto</button>
      <button class="chip" data-mode="dry"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3s6 7 6 11a6 6 0 0 1-12 0c0-4 6-11 6-11z"/></svg>Dry</button>
      <button class="chip" data-mode="fan"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="2"/><path d="M12 10c0-4 1-7 3-7s2 4-1 6M14 12c4 0 7 1 7 3s-4 2-6-1M12 14c0 4-1 7-3 7s-2-4 1-6M10 12c-4 0-7-1-7-3s4-2 6 1"/></svg>Fan</button>
    </div>
  </div>

  <!-- ADVANCED toggle -->
  <button class="adv-toggle" id="advToggle" data-lockable>
    <span>Advanced</span>
    <span class="chev"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M6 9l6 6 6-6"/></svg></span>
  </button>

  <div id="advanced" data-lockable>
    <!-- Vertical swing -->
    <div class="card">
      <h3>Vertical swing</h3>
      <div class="seg cols-4" id="vswingSeg">
      <button class="chip" data-vswing="1"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L10.0 16.9"/><path d="M10.0 16.9 L10.1 13.7"/><path d="M10.0 16.9 L12.9 15.6"/></svg></button>
      <button class="chip" data-vswing="2"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L8.2 15.3"/><path d="M8.2 15.3 L8.9 12.2"/><path d="M8.2 15.3 L11.3 14.6"/></svg></button>
      <button class="chip" data-vswing="3"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L6.6 13.5"/><path d="M6.6 13.5 L7.9 10.6"/><path d="M6.6 13.5 L9.8 13.4"/></svg></button>
      <button class="chip" data-vswing="4"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L5.5 11.4"/><path d="M5.5 11.4 L7.3 8.8"/><path d="M5.5 11.4 L8.7 11.9"/></svg></button>
      <button class="chip" data-vswing="5"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L4.8 9.1"/><path d="M4.8 9.1 L7.1 6.9"/><path d="M4.8 9.1 L7.8 10.2"/></svg></button>
      <button class="chip" data-vswing="6"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M17.0 6.5 L4.5 6.9"/><path d="M4.5 6.9 L7.2 5.1"/><path d="M4.5 6.9 L7.3 8.5"/></svg></button>
      <button class="chip" data-vswing="full"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M18.0 5.0 L7.0 5.0"/><path d="M7.0 5.0 L9.2 3.6"/><path d="M7.0 5.0 L9.2 6.4"/><path d="M18.0 5.0 L9.0 11.0"/><path d="M9.0 11.0 L10.1 8.6"/><path d="M9.0 11.0 L11.6 10.9"/><path d="M18.0 5.0 L11.0 17.0"/><path d="M11.0 17.0 L10.9 14.4"/><path d="M11.0 17.0 L13.3 15.8"/></svg></button>
      <button class="chip" data-vswing="off"><small>OFF</small></button>
      </div>
    </div>

    <!-- Energy control -->
    <div class="card">
      <h3>Energy saving</h3>
      <div class="seg cols-4" id="energySeg">
        <button class="chip" data-energy="80">80%</button>
        <button class="chip" data-energy="60">60%</button>
        <button class="chip" data-energy="40">40%</button>
        <button class="chip" data-energy="off"><small>OFF</small></button>
      </div>
    </div>

    <!-- Special toggles -->
    <div class="card">
      <h3>Functions</h3>
      <div class="toggle-row">
        <div class="ic"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M5 5l2 2M17 17l2 2M19 5l-2 2M7 17l-2 2"/></svg></div>
        <div class="lbl"><b>Air purify</b><span>Removes airborne particles</span></div>
        <label class="switch"><input type="checkbox" id="tgPurify"><span class="track"></span></label>
      </div>
      <div class="toggle-row">
        <div class="ic"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3s6 7 6 11a6 6 0 0 1-12 0c0-4 6-11 6-11z"/><path d="M9 17a3 3 0 0 0 3 2"/></svg></div>
        <div class="lbl"><b>Moisture removal</b><span>Dries the indoor unit</span></div>
        <label class="switch"><input type="checkbox" id="tgMoisture"><span class="track"></span></label>
      </div>
      <div class="toggle-row">
        <div class="ic"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M11 5 6 9H2v6h4l5 4z"/><path d="M16 9a5 5 0 0 1 0 6"/></svg></div>
        <div class="lbl"><b>Quiet mode</b><span>Reduces outdoor unit noise</span></div>
        <label class="switch"><input type="checkbox" id="tgQuiet"><span class="track"></span></label>
      </div>
      <div class="toggle-row">
        <div class="ic"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/></svg></div>
        <div class="lbl"><b>Display light</b><span>Indoor unit LED on/off</span></div>
        <label class="switch"><input type="checkbox" id="tgLight"><span class="track"></span></label>
      </div>
    </div>
  </div>

  <div class="footer">
    <span id="lastFrame">__LAST_ACTION__</span>
    <span class="version">Yalgacr v1.0</span>
  </div>

</div>

<div class="toast" id="toast"></div>

<script>
const MODE_MIN = { cool:18, heat:16, auto:18, dry:18, fan:18 };
const MODE_MAX = { cool:30, heat:30, auto:30, dry:30, fan:30 };
const IR_COOLDOWN_MS = __IR_COOLDOWN_MS__;  // injected from IR_COOLDOWN_SEC
let state = null;
let busy = false;
let cooldownUntil = 0;
// Track the state's "updated" stamp so the 30s sensor poll can detect a
// background change (e.g. the scheduler) and reload the form. Changes the
// client makes itself keep this in sync so they don't trigger a reload.
let lastUpdated = null;
let baselineSet = false;

const $ = id => document.getElementById(id);

function toast(msg, kind) {
  const t = $('toast');
  t.textContent = msg;
  t.className = 'toast show' + (kind ? ' ' + kind : '');
  clearTimeout(t._t);
  t._t = setTimeout(() => { t.className = 'toast'; }, 2200);
}

function renderState() {
  if (!state) return;
  const on = state.power === 'on';
  $('lcd').className = 'lcd ' + (on ? 'pwr-on' : 'pwr-off') + (on ? '' : ' lcd-off');
  $('lcdPower').textContent = on ? 'ON' : 'OFF';
  $('lcdTemp').textContent = state.temp;
  $('lcdMode').textContent = state.mode.toUpperCase();
  $('lcdFan').textContent = 'FAN ' + String(state.fan).toUpperCase();
  // Global scheduler status (present on /api/state; command responses omit it).
  if (state.scheduler_enabled !== undefined)
    $('lcdSched').textContent = 'SCHEDULE ' + (state.scheduler_enabled ? 'ON' : 'OFF');

  document.querySelectorAll('[data-mode]').forEach(b => b.classList.toggle('active', b.dataset.mode === state.mode));
  document.querySelectorAll('[data-fan]').forEach(b => b.classList.toggle('active', b.dataset.fan === String(state.fan)));
  document.querySelectorAll('[data-vswing]').forEach(b => b.classList.toggle('active', b.dataset.vswing === String(state.vswing)));
  document.querySelectorAll('[data-energy]').forEach(b => b.classList.toggle('active', b.dataset.energy === String(state.energy)));

  // JET chip is active when jet is on
  $('jetChip').classList.toggle('active', state.jet === 'on');

  $('tempVal').textContent = state.temp;
  $('tgPurify').checked = state.purify === 'on';
  $('tgMoisture').checked = state.moisture === 'on';
  $('tgQuiet').checked = state.quiet === 'on';
  $('tgLight').checked = state.light === 'on';

  const lo = MODE_MIN[state.mode], hi = MODE_MAX[state.mode];
  $('tempDown').disabled = state.temp <= lo;
  $('tempUp').disabled = state.temp >= hi;

  // Lock everything except power when the AC is off (mirrors physical remote)
  const locked = !on;
  document.querySelectorAll('[data-lockable]').forEach(el =>
    el.classList.toggle('locked', locked));
}

async function send(action, params) {
  // A rejected or too-early command must not leave a toggle switch flipped,
  // so every early-out re-syncs the UI back to the last known state.
  if (busy) { renderState(); return; }
  // Silent 3s gate: no dimming or countdown -- just refuse and show a toast
  // if a button is pressed too soon. The server enforces the same window.
  if (Date.now() < cooldownUntil) { toast('Wait 3 seconds', ''); renderState(); return; }
  busy = true;
  $('app').classList.add('busy');
  try {
    const r = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action, params: params || {} })
    });
    const j = await r.json();
    if (j.ok) {
      state = j.state;
      renderState();
      lastUpdated = state.updated;
      baselineSet = true;
      $('lastFrame').textContent = 'Last action ' + j.time + ': ' + j.event;
      toast(j.event, 'ok');
      cooldownUntil = Date.now() + IR_COOLDOWN_MS;
    } else if (r.status === 429) {
      // Server cooldown (e.g. another tab sent a command); align our window.
      if (j.retry_after) cooldownUntil = Date.now() + Math.ceil(j.retry_after * 1000);
      toast('Wait 3 seconds', '');
      renderState();
    } else {
      toast(j.error || 'Error', 'err');
      renderState();
    }
  } catch (e) {
    toast('Network error: ' + e.message, 'err');
    renderState();
  } finally {
    busy = false;
    $('app').classList.remove('busy');
  }
}

async function loadState() {
  try {
    const r = await fetch('/api/state');
    state = await r.json();
    lastUpdated = state.updated;
    baselineSet = true;
    renderState();
  } catch (e) {
    toast('Cannot load state', 'err');
  }
}

document.querySelectorAll('[data-action]').forEach(b =>
  b.addEventListener('click', () => send(b.dataset.action)));
document.querySelectorAll('[data-mode]').forEach(b =>
  b.addEventListener('click', () => send('climate', { mode: b.dataset.mode })));
document.querySelectorAll('[data-fan]').forEach(b =>
  b.addEventListener('click', () => send('climate', { fan: b.dataset.fan })));
document.querySelectorAll('[data-vswing]').forEach(b =>
  b.addEventListener('click', () => send('vswing', { value: b.dataset.vswing })));
document.querySelectorAll('[data-energy]').forEach(b =>
  b.addEventListener('click', () => send('energy', { value: b.dataset.energy })));

// JET chip: toggle based on current state
$('jetChip').addEventListener('click', () => {
  const next = (state && state.jet === 'on') ? 'off' : 'on';
  send('jet', { value: next });
});

$('tempUp').addEventListener('click', () => {
  if (!state) return;
  if (state.temp < MODE_MAX[state.mode]) send('climate', { temp: state.temp + 1 });
});
$('tempDown').addEventListener('click', () => {
  if (!state) return;
  if (state.temp > MODE_MIN[state.mode]) send('climate', { temp: state.temp - 1 });
});

$('tgPurify').addEventListener('change', e => send('purify', { value: e.target.checked ? 'on' : 'off' }));
$('tgMoisture').addEventListener('change', e => send('moisture', { value: e.target.checked ? 'on' : 'off' }));
$('tgQuiet').addEventListener('change', e => send('quiet', { value: e.target.checked ? 'on' : 'off' }));
$('tgLight').addEventListener('change', e => send('light', { value: e.target.checked ? 'on' : 'off' }));

// Advanced rollout: closed on load, stays as user leaves it during the session
$('advToggle').addEventListener('click', () => {
  $('advToggle').classList.toggle('open');
  $('advanced').classList.toggle('open');
});

$('gearBtn').addEventListener('click', () => { location.href = '/settings'; });

// Determine the logged-in role and adapt the UI for "user" (view-only).
async function loadMe() {
  try {
    const r = await fetch('/api/me');
    if (r.status === 401) { location.href = '/login'; return; }
    const j = await r.json();
    if (j.role === 'user') {
      document.body.classList.add('viewer');
      $('gearBtn').style.display = 'none';   // users can't open settings
    }
  } catch (e) { /* ignore; controls are still server-protected */ }
}

async function loadSensors() {
  try {
    const r = await fetch('/api/changes');
    if (!r.ok) return;
    const j = await r.json();
    // Background change detection: if the state's "updated" stamp moved without
    // this client causing it (i.e. the scheduler fired), reload the whole form.
    if (j.updated !== undefined) {
      if (!baselineSet) { lastUpdated = j.updated; baselineSet = true; }
      else if (j.updated !== lastUpdated) { location.reload(); return; }
    }
    const list = j.sensors || [];
    const box = $('lcdSensors');
    if (!box) return;
    box.innerHTML = list.map(s => {
      const val = s.ok
        ? '<span class="ls-val">' + s.temp + '°C ' + s.humidity + '%</span>'
        : '<span class="ls-na">N/A</span>';
      return '<div class="ls-sensor"><div class="ls-name">' + s.name + '</div>' +
             '<div>' + val + '</div></div>';
    }).join('');
  } catch (e) { /* ignore */ }
}

loadMe();
loadState();
loadSensors();
setInterval(loadSensors, 30000);   // refresh ambient readings
</script>
</body>
</html>
"""


# ===========================================================================
# Main
# ===========================================================================

LOGIN_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Sign in — LG AC Remote</title>
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<style>
  /*__BASE_CSS__*/
  /* Login centres its card vertically; BASE_CSS otherwise top-aligns. */
  body { align-items: center; padding: 20px; }
  .login {
    width: 100%; max-width: 360px; background: var(--panel);
    border: 1px solid var(--line); border-radius: var(--r); padding: 28px 24px;
    box-shadow: var(--shadow);
  }
  .brand { text-align: center; margin-bottom: 24px; }
  .brand svg { width: 44px; height: 44px; }
  .brand b { display: block; font-family: "Space Mono", ui-monospace, monospace;
             font-size: 1.15rem; font-weight: 800; margin-top: 10px; letter-spacing: .5px; }
  label { display: block; font-size: .74rem; letter-spacing: 1px; text-transform: uppercase;
          color: var(--ink-dim); font-weight: 700; margin: 14px 0 6px; }
  input {
    width: 100%; padding: 13px 14px; background: var(--panel-2); border: 1px solid var(--line);
    border-radius: 10px; color: var(--ink); font-size: 1rem; font-family: inherit;
  }
  input:focus { outline: none; border-color: var(--accent); }
  button {
    width: 100%; margin-top: 22px; padding: 14px; border: none; border-radius: 10px;
    background: var(--accent); color: #fff; font-weight: 800; font-size: 1rem;
    cursor: pointer; font-family: inherit; transition: filter .2s, transform .12s;
  }
  button:hover { filter: brightness(1.08); }
  button:active { transform: scale(.98); }
  .err {
    margin-top: 16px; padding: 11px 14px; border-radius: 10px; font-size: .85rem;
    background: rgba(193,75,75,.15); border: 1px solid #c14b4b; color: #f0a8a8;
    display: none;
  }
  .err.show { display: block; }
</style>
</head>
<body>
  <form class="login" id="loginForm">
    <div class="brand">
      <svg viewBox="0 0 64 64" xmlns="http://www.w3.org/2000/svg">
        <rect x="6" y="16" width="52" height="26" rx="5" fill="none" stroke="#e8703a" stroke-width="3"/>
        <line x1="12" y1="26" x2="52" y2="26" stroke="#5ec8e8" stroke-width="2.2"/>
        <path d="M14 35 Q20 32 26 35 T38 35 T50 35" fill="none" stroke="#5ec8e8" stroke-width="2.6" stroke-linecap="round"/>
      </svg>
      <b>LG AC Remote</b>
    </div>
    <label for="username">Username</label>
    <input type="text" id="username" autocomplete="username" autofocus>
    <label for="password">Password</label>
    <input type="password" id="password" autocomplete="current-password">
    <button type="submit">Sign in</button>
    <div class="err" id="err"></div>
  </form>
<script>
  const params = new URLSearchParams(location.search);
  const nextUrl = params.get('next') || '/';
  const form = document.getElementById('loginForm');
  const err = document.getElementById('err');
  form.addEventListener('submit', async (e) => {
    e.preventDefault();
    err.className = 'err';
    try {
      const r = await fetch('/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          username: document.getElementById('username').value,
          password: document.getElementById('password').value
        })
      });
      const j = await r.json();
      if (j.ok) {
        location.href = nextUrl;
      } else {
        err.textContent = j.error || 'Login failed';
        err.className = 'err show';
      }
    } catch (e) {
      err.textContent = 'Network error: ' + e.message;
      err.className = 'err show';
    }
  });
</script>
</body>
</html>
"""

SETTINGS_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Settings — LG AC Remote</title>
<link rel="icon" href="/favicon.svg" type="image/svg+xml">
<style>
  /*__BASE_CSS__*/
  .wrap { width: 100%; max-width: 560px; }

  header { position: relative; margin-bottom: 18px; height: 46px; display: flex; align-items: center; }
  .title { width: 100%; text-align: center; }
  .title b { font-size: 1.28rem; letter-spacing: .5px; font-weight: 800;
             font-family: "Space Mono", ui-monospace, monospace; }
  .back {
    position: absolute; left: 0; top: 0; height: 44px; padding: 0 14px 0 10px;
    background: var(--panel); border: 1px solid var(--line); border-radius: 12px;
    display: flex; align-items: center; gap: 6px; cursor: pointer; color: var(--ink-dim);
    text-decoration: none; font-size: .85rem; font-weight: 600; transition: color .2s;
  }
  .back:hover { color: var(--accent); }
  .back svg { width: 18px; height: 18px; }

  .card {
    background: var(--panel); border: 1px solid var(--line);
    border-radius: var(--r); padding: 16px; margin-bottom: 14px; box-shadow: var(--shadow);
  }
  .card h3 {
    margin: 0 0 4px; font-size: .74rem; letter-spacing: 1.4px;
    text-transform: uppercase; color: var(--ink-dim); font-weight: 700;
  }
  .card .hint { font-size: .76rem; color: var(--ink-dim); margin: 0 0 14px; }

  table { width: 100%; border-collapse: collapse; }
  th, td { text-align: left; padding: 10px 8px; border-bottom: 1px solid var(--line); font-size: .9rem; }
  /* The User management card has an <hr> right after the table; drop the
     last row's border so the two don't form a double line (matches Sensors). */
  #userRows tr:last-child td { border-bottom: none; }
  th { font-size: .7rem; letter-spacing: 1px; text-transform: uppercase; color: var(--ink-dim); }
  td.actions { text-align: right; white-space: nowrap; }
  .role-badge {
    font-size: .7rem; font-weight: 700; padding: 3px 9px; border-radius: 999px;
    text-transform: uppercase; letter-spacing: .5px;
  }
  .role-admin { background: rgba(232,112,58,.18); color: var(--accent); border: 1px solid rgba(232,112,58,.4); }
  .role-user  { background: var(--panel-2); color: var(--ink-dim); border: 1px solid var(--line); }
  .me-tag { font-size: .68rem; color: var(--ink-dim); margin-left: 6px; }

  .btn {
    border: 1px solid var(--line); background: var(--panel-2); color: var(--ink);
    border-radius: 8px; padding: 7px 11px; font-size: .8rem; font-weight: 600;
    cursor: pointer; font-family: inherit; margin-left: 6px; transition: all .15s;
  }
  .btn:hover { border-color: var(--accent); color: var(--accent); }
  .btn-danger:hover { border-color: #c14b4b; color: #f0a8a8; }
  .btn:disabled { opacity: .3; cursor: not-allowed; }
  .btn:disabled:hover { border-color: var(--line); color: var(--ink); }

  /* Narrow screens: the Room sensors / User management tables are too wide for
     a phone (sensor IPs + action buttons overflow). Turn each row into a
     stacked, labelled block so nothing runs off the edge. Desktop unaffected. */
  @media (max-width: 480px) {
    table, tbody, tr, td { display: block; width: 100%; }
    thead { display: none; }
    tr { border-bottom: 1px solid var(--line); padding: 8px 0; }
    #userRows tr:last-child, #sensorRows tr:last-child { border-bottom: none; }
    td { border: none; padding: 3px 0; font-size: .86rem; }
    td[data-label]::before {
      content: attr(data-label) ": "; color: var(--ink-dim); font-size: .68rem;
      text-transform: uppercase; letter-spacing: .5px; font-weight: 700; margin-right: 6px;
    }
    td.actions { text-align: left; white-space: normal; padding-top: 8px; }
    td.actions .btn { display: block; width: 100%; margin: 7px 0 0; padding: 9px; }
    td.actions .btn:first-child { margin-top: 0; }
  }

  .add-grid { display: grid; grid-template-columns: 1fr 1fr auto auto; gap: 10px; align-items: end; }
  @media (max-width: 520px) { .add-grid { grid-template-columns: 1fr 1fr; } }
  label { display: block; font-size: .7rem; letter-spacing: 1px; text-transform: uppercase;
          color: var(--ink-dim); font-weight: 700; margin-bottom: 6px; }
  input, select {
    width: 100%; padding: 11px 12px; background: var(--panel-2); border: 1px solid var(--line);
    border-radius: 9px; color: var(--ink); font-size: .92rem; font-family: inherit;
  }
  input:focus, select:focus { outline: none; border-color: var(--accent); }
  .btn-primary {
    background: var(--accent); border-color: var(--accent); color: #fff; padding: 11px 16px;
    border-radius: 9px; border: none; font-weight: 700; cursor: pointer; font-family: inherit;
    font-size: .9rem; white-space: nowrap;
  }
  .btn-primary:hover { filter: brightness(1.08); }
  .btn-primary:disabled { opacity: .4; cursor: not-allowed; filter: none; }

  /* segmented radio (http/https, local/remote) */
  .seg-radio { display: flex; gap: 8px; margin-bottom: 14px; }
  .seg-radio label {
    flex: 1; margin: 0; text-align: center; padding: 11px; border-radius: 10px;
    border: 1px solid var(--line); background: var(--panel-2); cursor: pointer;
    font-size: .88rem; font-weight: 700; letter-spacing: .5px; color: var(--ink-dim);
    text-transform: none; transition: all .15s;
  }
  .seg-radio input { display: none; }
  .seg-radio input:checked + label,
  .seg-radio label.checked {
    background: var(--accent); border-color: var(--accent); color: #fff;
  }
  .field { margin-bottom: 14px; }
  .field:last-child { margin-bottom: 0; }
  .row2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
  .toggle-line { display: flex; align-items: center; justify-content: space-between;
                 padding: 10px 0; }
  .toggle-line .lbl b { display: block; font-size: .9rem; }
  .toggle-line .lbl span { font-size: .72rem; color: var(--ink-dim); }
  /*__UI_CSS__*/
  .hidden { display: none; }
  .save-row { margin-top: 16px; display: flex; align-items: center; gap: 12px; }
  .note { font-size: .72rem; color: var(--ink-dim); }
  .note-warn { color: var(--accent); }

  /* Scheduler */
  .day-tabs { display: flex; gap: 6px; margin: 4px 0 16px; }
  .day-tabs button {
    flex: 1; padding: 9px 0; border-radius: 8px; border: 1px solid var(--line);
    background: var(--panel-2); color: var(--ink-dim); font-size: .78rem;
    font-weight: 700; cursor: pointer; font-family: inherit; position: relative;
  }
  .day-tabs button.active { background: var(--accent); border-color: var(--accent); color: #fff; }
  .day-tabs button .dot {
    position: absolute; top: 5px; right: 6px; width: 5px; height: 5px;
    border-radius: 50%; background: var(--accent);
  }
  .day-tabs button.active .dot { background: #fff; }
  .period {
    background: var(--panel-2); border: 1px solid var(--line); border-radius: 10px;
    padding: 12px; margin-bottom: 10px;
  }
  .period.disabled { opacity: .5; }
  .period-head { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }
  .period-head .p-title { font-size: .8rem; font-weight: 700; color: var(--ink-dim);
    text-transform: uppercase; letter-spacing: .5px; }
  .period-head .p-actions { display: flex; align-items: center; gap: 10px; }
  .period-times { display: flex; gap: 12px; }
  .period-times .pt { flex: 1; }
  .period-times label { margin-bottom: 5px; }
  .period-times .time-row { display: flex; align-items: center; gap: 6px; }
  .period-times .time-sel {
    flex: 1; width: auto; padding: 9px 8px; background: var(--panel); border: 1px solid var(--line);
    border-radius: 8px; color: var(--ink); font-size: .92rem; font-family: inherit; text-align: center;
  }
  .period-times .time-sel:focus { outline: none; border-color: var(--accent); }
  .period-times .time-colon { color: var(--ink-dim); font-weight: 700; }
  .icon-btn {
    background: none; border: none; color: var(--ink-dim); cursor: pointer; padding: 2px;
    display: grid; place-items: center;
  }
  .icon-btn:hover { color: #f0a8a8; }
  .icon-btn svg { width: 18px; height: 18px; }

  /* Display colour swatches: each shows the actual LCD gradient. */
  .swatches { display: flex; gap: 12px; flex-wrap: wrap; }
  .swatch {
    flex: 1 1 0; min-width: 92px; cursor: pointer; border-radius: 12px;
    border: 2px solid var(--line); padding: 6px; background: var(--panel-2);
    transition: border-color .15s; text-align: center;
  }
  .swatch:hover { border-color: var(--ink-dim); }
  .swatch.active { border-color: var(--accent); }
  .swatch .preview {
    height: 44px; border-radius: 8px; box-shadow: inset 0 1px 0 rgba(255,255,255,.6);
  }
  .swatch .sw-label {
    display: block; margin-top: 7px; font-size: .78rem; font-weight: 600; color: var(--ink-dim);
  }
  .swatch.active .sw-label { color: var(--ink); }
  .pv-blue   { background: linear-gradient(160deg, #a7daff, #85ccff); }
  .pv-green  { background: linear-gradient(160deg, #b5ffb5, #98ff98); }
  .pv-orange { background: linear-gradient(160deg, #ffd391, #ffc266); }

  /* Log viewer */
  .logbox {
    background: #14171c; border: 1px solid var(--line); border-radius: 10px;
    padding: 12px 14px; font-family: "Space Mono", ui-monospace, monospace;
    font-size: .76rem; line-height: 1.6; color: var(--ink-dim);
    white-space: pre-wrap; word-break: break-word;
    /* Show ~10 lines (font-size x line-height x 10 + vertical padding); the rest
       of the loaded lines scroll inside this fixed-height box. */
    max-height: calc(.76rem * 1.6 * 10 + 24px); overflow-y: auto;
  }
  .logbox .empty { color: var(--ink-dim); font-style: italic; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <a class="back" href="/"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M15 18l-6-6 6-6"/></svg>Back</a>
    <div class="title"><b>Settings</b></div>
  </header>

  <!-- Scheduler -->
  <div class="card">
    <h3>Schedule</h3>
    <label class="switch" style="margin:2px 0 4px;"><input type="checkbox" id="schedEnabled"><span class="track"></span></label>
    <hr style="border:none; border-top:1px solid var(--line); margin:16px 0;">
    <div class="day-tabs" id="dayTabs"></div>
    <div id="periodList"></div>
    <div class="save-row">
      <button class="btn-primary" id="addPeriod">Add cycle</button>
      <button class="btn-primary" id="savePeriods" disabled>Save</button>
    </div>
  </div>

  <!-- IR Toy connection -->
  <div class="card">
    <h3>IR Toy connection</h3>
    <div class="seg-radio">
      <input type="radio" name="irmode" id="irLocal" value="local"><label for="irLocal">Local (USB)</label>
      <input type="radio" name="irmode" id="irRemote" value="remote"><label for="irRemote">Remote (socat)</label>
    </div>

    <div id="irLocalFields">
      <div class="field">
        <label for="irDevice">Serial device</label>
        <input type="text" id="irDevice" placeholder="/dev/ttyACM0" value="/dev/ttyACM0">
      </div>
    </div>

    <div id="irRemoteFields" class="hidden">
      <div class="row2">
        <div class="field">
          <label for="irHost">Host IP</label>
          <input type="text" id="irHost" placeholder="192.168.1.10">
        </div>
        <div class="field">
          <label for="irPort">Port</label>
          <input type="number" id="irPort" min="1" max="65535" value="2000">
        </div>
      </div>
    </div>

    <div class="save-row">
      <button class="btn-primary" id="saveIr">Save</button>
      <span class="note" id="irNote"></span>
    </div>
  </div>

  <!-- Room sensors (table + add form in one card) -->
  <div class="card">
    <h3>Room sensors (BME280)</h3>
    <table id="sensorTable">
      <thead><tr><th>Name</th><th>Type</th><th>I2C address</th><th class="actions">Actions</th></tr></thead>
      <tbody id="sensorRows"></tbody>
    </table>
    <div class="field" style="margin-top:14px; max-width:220px;">
      <label for="pollPeriod">Poll period (seconds)</label>
      <div style="display:flex; gap:10px;">
        <input type="number" id="pollPeriod" min="5" max="3600" value="60">
        <button class="btn-primary" id="savePoll" style="white-space:nowrap;">Save</button>
      </div>
    </div>
    <hr style="border:none; border-top:1px solid var(--line); margin:18px 0 16px;">
    <label style="margin-bottom:10px;">Add sensor</label>
    <div class="field">
      <label for="sName">Name</label>
      <input type="text" id="sName" placeholder="Room 1">
    </div>
    <div class="seg-radio">
      <input type="radio" name="smode" id="sLocal" value="local" checked><label for="sLocal">Local (I2C)</label>
      <input type="radio" name="smode" id="sRemote" value="remote"><label for="sRemote">Remote (pigpiod)</label>
    </div>
    <div class="row2">
      <div class="field">
        <label for="sAddr">I2C address</label>
        <input type="text" id="sAddr" value="0x76">
      </div>
      <div class="field hidden" id="sHostField">
        <label for="sHost">Host IP</label>
        <input type="text" id="sHost" placeholder="192.168.1.10">
      </div>
    </div>
    <div class="save-row">
      <button class="btn-primary" id="addSensor">Add sensor</button>
    </div>
  </div>

  <button class="adv-toggle" id="advToggle">
    <span>Advanced</span>
    <span class="chev"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"><path d="M6 9l6 6 6-6"/></svg></span>
  </button>

  <div id="advanced">
  <!-- User management (table + add form in one card) -->
  <div class="card">
    <h3>User management</h3>
    <table id="userTable">
      <thead><tr><th>Username</th><th>Role</th><th class="actions">Actions</th></tr></thead>
      <tbody id="userRows"></tbody>
    </table>
    <hr style="border:none; border-top:1px solid var(--line); margin:18px 0 16px;">
    <label style="margin-bottom:10px;">Add user</label>
    <div class="add-grid">
      <div>
        <label for="newUser">Username</label>
        <input type="text" id="newUser" autocomplete="off">
      </div>
      <div>
        <label for="newPass">Password</label>
        <input type="password" id="newPass" autocomplete="new-password">
      </div>
      <div>
        <label for="newRole">Role</label>
        <select id="newRole">
          <option value="user">User</option>
          <option value="admin">Admin</option>
        </select>
      </div>
      <button class="btn-primary" id="addBtn">Add user</button>
    </div>
  </div>

  <!-- Web server -->
  <div class="card">
    <h3>Web server</h3>
    <div class="seg-radio">
      <input type="radio" name="proto" id="protoHttp" value="http"><label for="protoHttp">HTTP</label>
      <input type="radio" name="proto" id="protoHttps" value="https"><label for="protoHttps">HTTPS</label>
    </div>

    <div id="httpFields">
      <div class="field">
        <label for="httpPort">HTTP port</label>
        <input type="number" id="httpPort" min="1" max="65535" value="8080">
      </div>
    </div>

    <div id="httpsFields" class="hidden">
      <div class="field">
        <label for="httpsPort">HTTPS port</label>
        <input type="number" id="httpsPort" min="1" max="65535" value="8443">
      </div>
      <label>Certificate</label>
      <div class="seg-radio" id="certSource">
        <input type="radio" name="certsrc" id="csSelf" value="selfsigned"><label for="csSelf">Self-signed</label>
        <input type="radio" name="certsrc" id="csLE" value="letsencrypt"><label for="csLE">Let's Encrypt</label>
        <input type="radio" name="certsrc" id="csManual" value="manual"><label for="csManual">Manual</label>
      </div>
      <div id="csSelfNote" class="field">
        <p class="note">A self-signed certificate is generated automatically. Your browser will warn the first time — choose "proceed" to continue; the connection is still encrypted.</p>
      </div>
      <div id="leFields" class="field hidden">
        <label for="leDomain">Domain</label>
        <input type="text" id="leDomain" placeholder="example.com">
        <p class="note">Certificate is read from /etc/letsencrypt/live/&lt;domain&gt;/</p>
      </div>
      <div id="manualCertFields" class="hidden">
        <div class="field">
          <label for="certPath">Certificate path (fullchain.pem)</label>
          <input type="text" id="certPath" placeholder="/path/to/fullchain.pem">
        </div>
        <div class="field">
          <label for="keyPath">Private key path (privkey.pem)</label>
          <input type="text" id="keyPath" placeholder="/path/to/privkey.pem">
        </div>
      </div>
    </div>

    <div class="save-row">
      <button class="btn-primary" id="saveServer">Save &amp; restart</button>
      <span class="note note-warn" id="serverNote"></span>
    </div>
  </div>

  <!-- Display colour (per-user LCD backlight) -->
  <div class="card">
    <h3>Display colour</h3>
    <div class="swatches" id="swatches">
      <div class="swatch" data-color="blue">
        <div class="preview pv-blue"></div><span class="sw-label">Blue</span>
      </div>
      <div class="swatch" data-color="green">
        <div class="preview pv-green"></div><span class="sw-label">Green</span>
      </div>
      <div class="swatch" data-color="orange">
        <div class="preview pv-orange"></div><span class="sw-label">Orange</span>
      </div>
    </div>
  </div>

  <!-- Audit log (last entries) -->
  <div class="card">
    <h3>Logs</h3>
    <div class="logbox" id="logbox"><span class="empty">Loading…</span></div>
  </div>
  </div>

  <div class="footer"><span></span><span class="version">Yalgacr v1.0</span></div>

</div>

<div class="toast" id="toast"></div>

<script>
  const $ = id => document.getElementById(id);
  let me = null;

  function toast(msg, kind) {
    const t = $('toast');
    t.textContent = msg;
    t.className = 'toast show' + (kind ? ' ' + kind : '');
    clearTimeout(t._t);
    t._t = setTimeout(() => { t.className = 'toast'; }, 2400);
  }

  async function loadUsers() {
    const r = await fetch('/api/users');
    if (r.status === 403 || r.status === 401) { location.href = '/login'; return; }
    const j = await r.json();
    me = j.me;
    const rows = $('userRows');
    rows.innerHTML = '';
    j.users.forEach(u => {
      const tr = document.createElement('tr');
      const isMe = u.username === me;
      tr.innerHTML =
        '<td data-label="Username">' + u.username + (isMe ? '<span class="me-tag">(you)</span>' : '') + '</td>' +
        '<td data-label="Role"><span class="role-badge role-' + u.role + '">' + u.role + '</span></td>' +
        '<td class="actions">' +
          '<button class="btn" data-act="pass" data-user="' + u.username + '">Reset password</button>' +
          '<button class="btn" data-act="role" data-user="' + u.username + '" data-role="' + u.role + '">' +
             (u.role === 'admin' ? 'Make user' : 'Make admin') + '</button>' +
          '<button class="btn btn-danger" data-act="del" data-user="' + u.username + '"' +
             (isMe ? ' disabled title="You cannot delete your own account"' : '') + '>Delete</button>' +
        '</td>';
      rows.appendChild(tr);
    });
  }

  document.addEventListener('click', async (e) => {
    const btn = e.target.closest('button[data-act]');
    if (!btn) return;
    const user = btn.dataset.user;
    const act = btn.dataset.act;

    if (act === 'del') {
      if (!confirm('Delete user "' + user + '"? This cannot be undone.')) return;
      const r = await fetch('/api/users/' + encodeURIComponent(user), { method: 'DELETE' });
      const j = await r.json();
      if (j.ok) { toast('User "' + user + '" deleted', 'ok'); loadUsers(); }
      else toast(j.error || 'Failed', 'err');
    }

    else if (act === 'pass') {
      const pw = prompt('New password for "' + user + '":');
      if (pw === null) return;
      if (!pw) { toast('Password cannot be empty', 'err'); return; }
      const r = await fetch('/api/users/' + encodeURIComponent(user) + '/password', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ password: pw })
      });
      const j = await r.json();
      if (j.ok) toast('Password reset for "' + user + '"', 'ok');
      else toast(j.error || 'Failed', 'err');
    }

    else if (act === 'role') {
      const newRole = btn.dataset.role === 'admin' ? 'user' : 'admin';
      const r = await fetch('/api/users/' + encodeURIComponent(user) + '/role', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ role: newRole })
      });
      const j = await r.json();
      if (j.ok) { toast(user + ' is now ' + newRole, 'ok'); loadUsers(); }
      else toast(j.error || 'Failed', 'err');
    }
  });

  $('addBtn').addEventListener('click', async () => {
    const username = $('newUser').value.trim();
    const password = $('newPass').value;
    const role = $('newRole').value;
    if (!username || !password) { toast('Username and password required', 'err'); return; }
    const r = await fetch('/api/users', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ username, password, role })
    });
    const j = await r.json();
    if (j.ok) {
      toast('User "' + username + '" added', 'ok');
      $('newUser').value = ''; $('newPass').value = ''; $('newRole').value = 'user';
      loadUsers();
    } else toast(j.error || 'Failed', 'err');
  });

  // ---- Server & IR settings ----
  function applyProtoVisibility() {
    const https = $('protoHttps').checked;
    $('httpFields').classList.toggle('hidden', https);
    $('httpsFields').classList.toggle('hidden', !https);
    applyCertVisibility();
  }
  function applyCertVisibility() {
    const src = (document.querySelector('input[name=certsrc]:checked') || {}).value || 'selfsigned';
    $('csSelfNote').classList.toggle('hidden', src !== 'selfsigned');
    $('leFields').classList.toggle('hidden', src !== 'letsencrypt');
    $('manualCertFields').classList.toggle('hidden', src !== 'manual');
  }
  function applyIrVisibility() {
    const remote = $('irRemote').checked;
    $('irLocalFields').classList.toggle('hidden', remote);
    $('irRemoteFields').classList.toggle('hidden', !remote);
  }
  document.querySelectorAll('input[name=proto]').forEach(el =>
    el.addEventListener('change', applyProtoVisibility));
  document.querySelectorAll('input[name=certsrc]').forEach(el =>
    el.addEventListener('change', applyCertVisibility));
  document.querySelectorAll('input[name=irmode]').forEach(el =>
    el.addEventListener('change', applyIrVisibility));

  async function loadSettings() {
    const r = await fetch('/api/settings');
    if (!r.ok) return;
    const j = await r.json();
    const s = j.server || {}, ir = j.ir || {};
    // server
    (s.protocol === 'https' ? $('protoHttps') : $('protoHttp')).checked = true;
    $('httpPort').value = s.http_port || 8080;
    $('httpsPort').value = s.https_port || 8443;
    const src = s.cert_source || 'selfsigned';
    ({selfsigned:$('csSelf'), letsencrypt:$('csLE'), manual:$('csManual')}[src] || $('csSelf')).checked = true;
    $('leDomain').value = s.domain || '';
    $('certPath').value = s.cert_path || '';
    $('keyPath').value = s.key_path || '';
    applyProtoVisibility();
    // ir
    (ir.mode === 'remote' ? $('irRemote') : $('irLocal')).checked = true;
    $('irDevice').value = ir.device || '/dev/ttyACM0';
    $('irHost').value = ir.host || '';
    $('irPort').value = ir.port || 2000;
    applyIrVisibility();
  }

  $('saveServer').addEventListener('click', async () => {
    const protocol = $('protoHttps').checked ? 'https' : 'http';
    const certSource = (document.querySelector('input[name=certsrc]:checked') || {}).value || 'selfsigned';
    const body = {
      protocol,
      http_port: parseInt($('httpPort').value, 10),
      https_port: parseInt($('httpsPort').value, 10),
      cert_source: certSource,
      domain: $('leDomain').value.trim(),
      cert_path: $('certPath').value.trim(),
      key_path: $('keyPath').value.trim()
    };
    $('serverNote').textContent = '';
    const r = await fetch('/api/settings/server', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    const j = await r.json();
    if (j.ok) {
      $('serverNote').textContent = 'Restarting... reconnecting to ' + j.new_url;
      toast('Settings saved — server is restarting', 'ok');
      // Give the server a moment to rebind, then move to the new address.
      setTimeout(() => { location.href = j.new_url + 'settings'; }, 3000);
    } else {
      toast(j.error || 'Failed', 'err');
    }
  });

  $('saveIr').addEventListener('click', async () => {
    const mode = $('irRemote').checked ? 'remote' : 'local';
    const body = { mode };
    if (mode === 'local') body.device = $('irDevice').value.trim();
    else { body.host = $('irHost').value.trim(); body.port = parseInt($('irPort').value, 10); }
    const r = await fetch('/api/settings/ir', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    const j = await r.json();
    if (j.ok) toast('IR connection saved', 'ok');
    else toast(j.error || 'Failed', 'err');
  });

  // ---- Sensors ----
  function applySensorModeVisibility() {
    $('sHostField').classList.toggle('hidden', !$('sRemote').checked);
  }
  document.querySelectorAll('input[name=smode]').forEach(el =>
    el.addEventListener('change', applySensorModeVisibility));

  async function loadSensors() {
    const r = await fetch('/api/settings');
    if (!r.ok) return;
    const j = await r.json();
    const sc = j.sensors || {};
    $('pollPeriod').value = sc.poll_period || 60;
    const rows = $('sensorRows');
    rows.innerHTML = '';
    (sc.list || []).forEach(s => {
      const tr = document.createElement('tr');
      const loc = s.mode === 'remote' ? ('Remote · ' + (s.host || '?')) : 'Local';
      tr.innerHTML =
        '<td data-label="Name">' + s.name + '</td>' +
        '<td data-label="Type">' + loc + '</td>' +
        '<td data-label="I2C address">' + s.address + '</td>' +
        '<td class="actions"><button class="btn btn-danger" data-sdel="' + s.id + '">Delete</button></td>';
      rows.appendChild(tr);
    });
    if (!(sc.list || []).length) {
      rows.innerHTML = '<tr><td colspan="4" style="color:var(--ink-dim);font-size:.84rem;">No sensors configured.</td></tr>';
    }
  }

  document.addEventListener('click', async (e) => {
    const del = e.target.closest('button[data-sdel]');
    if (!del) return;
    if (!confirm('Delete this sensor?')) return;
    const r = await fetch('/api/sensors/' + encodeURIComponent(del.dataset.sdel), { method: 'DELETE' });
    const j = await r.json();
    if (j.ok) { toast('Sensor deleted', 'ok'); loadSensors(); }
    else toast(j.error || 'Failed', 'err');
  });

  $('addSensor').addEventListener('click', async () => {
    const body = {
      name: $('sName').value.trim(),
      mode: $('sRemote').checked ? 'remote' : 'local',
      address: $('sAddr').value.trim() || '0x76',
      host: $('sHost').value.trim()
    };
    if (!body.name) { toast('Sensor name is required', 'err'); return; }
    const r = await fetch('/api/sensors/add', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    const j = await r.json();
    if (j.ok) {
      toast('Sensor added', 'ok');
      $('sName').value = ''; $('sHost').value = ''; $('sAddr').value = '0x76';
      $('sLocal').checked = true; applySensorModeVisibility();
      loadSensors();
    } else toast(j.error || 'Failed', 'err');
  });

  $('savePoll').addEventListener('click', async () => {
    const r = await fetch('/api/sensors/poll-period', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ poll_period: parseInt($('pollPeriod').value, 10) })
    });
    const j = await r.json();
    if (j.ok) toast('Poll period saved', 'ok');
    else toast(j.error || 'Failed', 'err');
  });

  // ---- Scheduler ----
  const DAYS = ['mon','tue','wed','thu','fri','sat','sun'];
  const DAY_LABELS = {mon:'Mon',tue:'Tue',wed:'Wed',thu:'Thu',fri:'Fri',sat:'Sat',sun:'Sun'};
  let schedule = { enabled: false, days: {} };
  let curDay = DAYS[(new Date().getDay() + 6) % 7];  // today (mon=0..sun=6)
  let dirty = false;                                  // unsaved period edits

  function renderDayTabs() {
    const tabs = $('dayTabs');
    tabs.innerHTML = DAYS.map(d => {
      // Dot only when the day has at least one ENABLED cycle.
      const has = (schedule.days[d] || []).some(p => p.enabled);
      return '<button data-day="' + d + '" class="' + (d === curDay ? 'active' : '') + '">' +
             DAY_LABELS[d] + (has ? '<span class="dot"></span>' : '') + '</button>';
    }).join('');
  }

  // Build two dropdowns (hour 00-23, minute in 5-min steps) for a HH:MM value.
  // This gives a 24-hour widget identically on every browser/OS, since the
  // native <input type=time> renders 12h/24h based on device locale.
  const pad2 = n => String(n).padStart(2, '0');

  function timeSelectHtml(which, i, value) {
    const parts = (value || '').split(':');
    const curH = (parts[0] || '00');
    const curM = (parts[1] || '00');
    let h = '';
    for (let n = 0; n < 24; n++) {
      const v = pad2(n);
      h += '<option value="' + v + '"' + (v === curH ? ' selected' : '') + '>' + v + '</option>';
    }
    const mins = [];
    for (let n = 0; n < 60; n += 5) mins.push(pad2(n));
    if (!mins.includes(curM)) { mins.push(curM); mins.sort(); }  // keep odd legacy values
    let m = '';
    for (const v of mins)
      m += '<option value="' + v + '"' + (v === curM ? ' selected' : '') + '>' + v + '</option>';
    return '<select class="time-sel" data-' + which + '-h="' + i + '">' + h + '</select>' +
           '<span class="time-colon">:</span>' +
           '<select class="time-sel" data-' + which + '-m="' + i + '">' + m + '</select>';
  }

  function renderPeriods() {
    const list = schedule.days[curDay] || [];
    const box = $('periodList');
    if (!list.length) {
      box.innerHTML = '<p class="note" style="margin:4px 0 0;">No cycles for ' +
        DAY_LABELS[curDay] + '. Add one below.</p>';
    } else {
      box.innerHTML = list.map((p, i) =>
        '<div class="period' + (p.enabled ? '' : ' disabled') + '">' +
          '<div class="period-head">' +
            '<span class="p-title">Cycle ' + (i + 1) + '</span>' +
            '<div class="p-actions">' +
              '<label class="switch"><input type="checkbox" data-pen="' + i + '"' +
                 (p.enabled ? ' checked' : '') + '><span class="track"></span></label>' +
              '<button class="icon-btn" data-pdel="' + i + '" title="Delete cycle">' +
                '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 6h18"/><path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/><path d="M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6"/></svg>' +
              '</button>' +
            '</div>' +
          '</div>' +
          '<div class="period-times">' +
            '<div class="pt"><label>Turn on</label><div class="time-row">' + timeSelectHtml('pon', i, p.on) + '</div></div>' +
            '<div class="pt"><label>Turn off</label><div class="time-row">' + timeSelectHtml('poff', i, p.off) + '</div></div>' +
          '</div>' +
        '</div>'
      ).join('');
    }
    $('addPeriod').style.display = list.length >= 3 ? 'none' : '';
  }

  // The Save button is enabled only when there are unsaved add/delete/time
  // edits. Per-period enable/disable toggles save themselves immediately.
  function setDirty(v) {
    dirty = v;
    $('savePeriods').disabled = !v;
  }

  async function saveDay() {
    const r = await fetch('/api/schedule/day', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ day: curDay, periods: schedule.days[curDay] || [] })
    });
    const j = await r.json();
    if (j.ok) { toast('Schedule saved', 'ok'); setDirty(false); renderDayTabs(); }
    else toast(j.error || 'Failed', 'err');
    return j.ok;
  }

  async function loadSchedule() {
    const r = await fetch('/api/schedule');
    if (!r.ok) return;
    const j = await r.json();
    schedule = j.schedule || { enabled: false, days: {} };
    DAYS.forEach(d => { if (!schedule.days[d]) schedule.days[d] = []; });
    $('schedEnabled').checked = !!schedule.enabled;
    setDirty(false);
    renderDayTabs();
    renderPeriods();
  }

  // The global scheduler switch still applies immediately.
  $('schedEnabled').addEventListener('change', async () => {
    const r = await fetch('/api/schedule/enabled', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: $('schedEnabled').checked })
    });
    const j = await r.json();
    if (j.ok) { schedule.enabled = $('schedEnabled').checked;
                toast('Scheduler ' + (schedule.enabled ? 'enabled' : 'disabled'), 'ok'); }
    else toast(j.error || 'Failed', 'err');
  });

  $('dayTabs').addEventListener('click', (e) => {
    const btn = e.target.closest('button[data-day]');
    if (!btn || btn.dataset.day === curDay) return;
    if (dirty && !confirm('You have unsaved changes for this day. Discard them?')) return;
    // Reload from server so unsaved edits to the previous day are dropped.
    curDay = btn.dataset.day;
    loadSchedule();
  });

  // Adding a cycle is an unsaved edit -> enables Save.
  $('addPeriod').addEventListener('click', () => {
    const list = schedule.days[curDay] || (schedule.days[curDay] = []);
    if (list.length >= 3) return;
    list.push({ on: '08:00', off: '10:00', enabled: true });
    renderPeriods();
    setDirty(true);
  });

  $('savePeriods').addEventListener('click', saveDay);

  // Read a period's HH:MM from its two dropdowns.
  function readTime(i, which) {
    const h = document.querySelector('select[data-' + which + '-h="' + i + '"]').value;
    const m = document.querySelector('select[data-' + which + '-m="' + i + '"]').value;
    return h + ':' + m;
  }

  $('periodList').addEventListener('change', async (e) => {
    const list = schedule.days[curDay] || [];
    const en = e.target.closest('input[data-pen]');
    if (en) {                                                          // toggle -> auto-save
      list[+en.dataset.pen].enabled = en.checked;
      renderPeriods();
      const ok = await saveDay();
      if (!ok) loadSchedule();   // resync if the save failed
      return;
    }
    const onSel = e.target.closest('select[data-pon-h], select[data-pon-m]');
    const offSel = e.target.closest('select[data-poff-h], select[data-poff-m]');
    if (onSel) {                                                       // on-time edit -> Save
      const i = +(onSel.dataset.ponH ?? onSel.dataset.ponM);
      list[i].on = readTime(i, 'pon');
      setDirty(true);
    } else if (offSel) {                                              // off-time edit -> Save
      const i = +(offSel.dataset.poffH ?? offSel.dataset.poffM);
      list[i].off = readTime(i, 'poff');
      setDirty(true);
    }
  });

  // Deleting a cycle is an unsaved edit -> enables Save.
  $('periodList').addEventListener('click', (e) => {
    const del = e.target.closest('button[data-pdel]');
    if (!del) return;
    const list = schedule.days[curDay] || [];
    list.splice(+del.dataset.pdel, 1);
    renderPeriods();
    setDirty(true);
  });

  // ---- Display colour (per-user) ----
  async function loadDisplayColor() {
    try {
      const r = await fetch('/api/me');
      if (!r.ok) return;
      const j = await r.json();
      const cur = j.display_color || 'blue';
      document.querySelectorAll('#swatches .swatch').forEach(s =>
        s.classList.toggle('active', s.dataset.color === cur));
    } catch (e) { /* ignore */ }
  }

  document.querySelectorAll('#swatches .swatch').forEach(s =>
    s.addEventListener('click', async () => {
      const color = s.dataset.color;
      const r = await fetch('/api/display-color', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ color })
      });
      const j = await r.json();
      if (j.ok) {
        document.querySelectorAll('#swatches .swatch').forEach(x =>
          x.classList.toggle('active', x.dataset.color === color));
        toast('Display colour saved', 'ok');
      } else {
        toast(j.error || 'Failed', 'err');
      }
    }));

  // ---- Logs ----
  async function loadLogs() {
    const box = $('logbox');
    try {
      const r = await fetch('/api/logs');
      if (!r.ok) { box.innerHTML = '<span class="empty">Cannot load logs</span>'; return; }
      const j = await r.json();
      const lines = j.lines || [];
      if (!lines.length) {
        box.innerHTML = '<span class="empty">No log entries yet</span>';
        return;
      }
      box.textContent = lines.join('\n');
    } catch (e) {
      box.innerHTML = '<span class="empty">Cannot load logs</span>';
    }
  }

  loadUsers();
  loadSettings();
  loadSensors();
  loadSchedule();
  // Advanced section: collapsed on load, toggles open/closed like the main page.
  $('advToggle').addEventListener('click', () => {
    $('advToggle').classList.toggle('open');
    $('advanced').classList.toggle('open');
  });

  loadDisplayColor();
  loadLogs();
</script>
</body>
</html>
"""


# Inject the shared CSS into each template once, now that all three are
# defined. INDEX_HTML keeps its own runtime placeholders (/*__LCD_THEME__*/ and
# __IR_COOLDOWN_MS__), which are substituted per request in index().
INDEX_HTML = INDEX_HTML.replace("/*__BASE_CSS__*/", BASE_CSS).replace("/*__UI_CSS__*/", UI_CSS)
LOGIN_HTML = LOGIN_HTML.replace("/*__BASE_CSS__*/", BASE_CSS)
SETTINGS_HTML = SETTINGS_HTML.replace("/*__BASE_CSS__*/", BASE_CSS).replace("/*__UI_CSS__*/", UI_CSS)


def main():
    global CONFIG_FILE, STATE_FILE

    ap = argparse.ArgumentParser(description="Yalgacr v1.0 web interface")
    ap.add_argument("--version", action="version", version="Yalgacr v1.0 web interface")
    ap.add_argument("--host", default=WEB_HOST,
                    help="Bind address (default 0.0.0.0; rarely changed)")
    ap.add_argument("--config", default=CONFIG_FILE,
                    help="Path to config file (server/IR settings + users)")
    args = ap.parse_args()

    CONFIG_FILE = os.path.abspath(args.config)
    # Keep the state file next to the config file so a custom --config stays
    # self-contained.
    STATE_FILE = os.path.join(os.path.dirname(CONFIG_FILE), "lgac-state.json")

    load_config()
    load_state()

    # Session cookie signing + lifetime
    app.secret_key = _config["secret_key"]
    app.permanent_session_lifetime = timedelta(days=SESSION_LIFETIME_DAYS)

    ircfg = _config.get("ir", {})
    ir_where = (f"remote {ircfg.get('host')}:{ircfg.get('port')}"
                if ircfg.get("mode") == "remote"
                else f"local {ircfg.get('device')}")
    print("Yalgacr v1.0 web interface")
    print(f"  IR Toy : {ir_where}")
    print(f"  State  : {STATE_FILE}")
    print(f"  Config : {CONFIG_FILE}")

    # Background BME280 poller (no-op until sensors are configured).
    threading.Thread(target=sensor_poller, name="bme280-poller",
                     daemon=True).start()
    # Background scheduler (no-op until enabled with cycles).
    threading.Thread(target=scheduler_loop, name="scheduler",
                     daemon=True).start()

    # Run the listener in its own thread so the main thread stays free to
    # receive signals. werkzeug's serve_forever() swallows KeyboardInterrupt,
    # so Ctrl+C alone wouldn't stop us -- instead we install SIGINT/SIGTERM
    # handlers that set the stop flag and shut the current listener down (from
    # a helper thread, since shutdown() must run off the serving thread).
    server_thread = threading.Thread(target=serve_loop, args=(args.host,),
                                     name="server")
    server_thread.start()

    def _handle_stop(signum, frame):
        if _should_stop.is_set():
            return
        print("\nStopping...")
        _should_stop.set()
        with _server_lock:
            srv = _server
        if srv is not None:
            threading.Thread(target=srv.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    while server_thread.is_alive():
        server_thread.join(timeout=0.5)
    print("Stopped.")


if __name__ == "__main__":
    main()
