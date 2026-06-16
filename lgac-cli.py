#!/usr/bin/env python3
"""
lgac-cli.py  --  Yalgacr v1.0  --  command-line interface
Yet Another LG AC Remote -- CLI for the LG remote AKB74955603 (LG S12EQ)

Controls an LG air conditioner via a USB Infrared Toy v2, either connected
locally (/dev/ttyACM0) or shared over the network (ser2net/socat TCP).

Protocol fully reverse-engineered and verified against captured frames from
remote AKB74955603. 28-bit frame, 38 kHz:

    88 [N4] [N3] [N2] [N1] [checksum]

    N4 = command group   0=climate, 1=jet, C=power/off/light
    N3 = mode            COOL=8 DRY=9 FAN=A AUTO=B HEAT=C
    N2 = temperature     temp - 15   (COOL 18-30, HEAT 16-30)
    N1 = fan             {1:0, 2:9, 3:2, 4:A, 5:4, auto:5}
    N0 = checksum        (8+8+N4+N3+N2+N1) & 0xF

Swing uses a separate command space (N4 N3 = 1 3) and is fire-and-forget;
swing state is recorded in the state file for status display only.

State file (lgac-state.json next to the script) remembers the last commanded
so that partial commands work, e.g. `lgac-cli.py --temp 19` keeps the current
mode and fan. State is best-effort: using the physical remote will desync it.

Examples:
    lgac-cli.py --on                          # power on (restores last climate state)
    lgac-cli.py --off                         # power off
    lgac-cli.py --mode cool --temp 22 --fan 3 # set everything at once
    lgac-cli.py --temp 19                      # change only temperature
    lgac-cli.py --fan auto                     # change only fan
    lgac-cli.py --jet on                       # jet mode on
    lgac-cli.py --vswing 3                     # vertical louver to position 3
    lgac-cli.py --light                        # toggle display light
    lgac-cli.py --status                       # show last known state (no send)
    lgac-cli.py --temp 22 --host 192.168.200.24  # send via remote IR Toy

    lgac-cli.py --raw 0x88C0051               # send an arbitrary frame (debug)
"""

import os
import sys
import json
import time
import socket
import struct
import argparse

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed:  sudo apt install python3-serial")


# ===========================================================================
# IR timings (μs) -- matched to captured AKB74955603 remote
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

# Temperature limits per mode
TEMP_LIMITS = {
    "cool": (18, 30),
    "heat": (16, 30),
    "auto": (18, 30),   # auto behaves like cool range on this unit
    "dry":  (18, 30),   # dry largely ignores temp but keep in range
    "fan":  (18, 30),   # fan ignores temp
}

# Swing command space: N4 N3 = 1 3
# Vertical: N2=0 + N1 in 4..9 for steps 1..6; N2=1 N1=4 full, N2=1 N1=5 off
VSWING = {
    "1": (0x0, 0x4), "2": (0x0, 0x5), "3": (0x0, 0x6),
    "4": (0x0, 0x7), "5": (0x0, 0x8), "6": (0x0, 0x9),
    "full": (0x1, 0x4), "off": (0x1, 0x5),
}

# Special fixed frames (all in the N4=C command group, like OFF and LIGHT)
FRAME_OFF   = 0x88C0051
FRAME_JET   = 0x8810089   # jet ON; jet OFF returns to prior climate frame
FRAME_LIGHT = 0x88C00A6

# Air purify (removes particles entering the indoor unit) -- distinct on/off
FRAME_PURIFY_ON  = 0x88C000C
FRAME_PURIFY_OFF = 0x88C0084
# Moisture removal (removes moisture inside the indoor unit) -- distinct on/off
FRAME_MOISTURE_ON  = 0x88C00B7
FRAME_MOISTURE_OFF = 0x88C00C8
# Quiet mode (reduces outdoor unit noise) -- distinct on/off
FRAME_QUIET_ON  = 0x88C0A6C
FRAME_QUIET_OFF = 0x88C0A7D
# Energy control (power saving) -- four explicit levels, not a toggle
FRAME_ENERGY = {
    "80":  0x88C07D0,
    "60":  0x88C07E1,
    "40":  0x88C0804,
    "off": 0x88C07F2,
}

# State file lives next to this script (based on the script's own location,
# not the current working directory), so it is found regardless of where the
# script is launched from.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STATE_FILE = os.path.join(_SCRIPT_DIR, "lgac-state.json")

# Shared audit log, the same file the web app writes to (both live in this
# directory in a normal install). CLI actions are recorded as user "cli".
# Format matches the web app: "DD.MM.YYYY HH:MM:SS | cli | action".
LOG_FILE = os.path.join(_SCRIPT_DIR, "lgac.log")


def log_event(action):
    """Append one audit line for a sent command. Never raises.
    Shares the web app's 4-field format; CLI runs locally so the IP is 'local'."""
    line = f"{time.strftime('%d.%m.%Y %H:%M:%S')} | cli | local | {action}\n"
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(line)
    except OSError as e:
        print(f"warning: cannot write log: {e}", file=sys.stderr)


DEFAULT_STATE = {
    "power": "off",
    "mode": "cool",
    "temp": 22,
    "fan": "auto",
    "vswing": None,    # last commanded vertical swing (display only)
    "jet": "off",
    "light": "on",     # LED display state (best-effort; toggle has no feedback)
    "purify": "off",   # air purification on/off
    "moisture": "off", # indoor unit moisture removal on/off
    "quiet": "off",    # outdoor unit quiet mode on/off
    "energy": "off",   # power-saving level: 80/60/40/off
    "updated": None,
}


# ===========================================================================
# Encoder
# ===========================================================================
def _checksum(n6, n5, n4, n3, n2, n1):
    return (n6 + n5 + n4 + n3 + n2 + n1) & 0xF


def make_frame(n4, n3, n2, n1):
    """Assemble a 28-bit frame from the four variable nibbles."""
    n6, n5 = PREFIX_N6, PREFIX_N5
    k = _checksum(n6, n5, n4, n3, n2, n1)
    val = 0
    for nib in (n6, n5, n4, n3, n2, n1, k):
        val = (val << 4) | (nib & 0xF)
    return val


def encode_climate(mode, temp, fan):
    """Build a normal climate frame (mode + temp + fan)."""
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
    """Power-on toggle from OFF: N4=0 N3=0, carrying temp + fan.

    This mirrors what the real remote sends when turning the unit on
    (e.g. captured 0x8800347 = on, 23C-equivalent, fan4). We always send
    an explicit temp+fan so the unit powers into a known state.
    """
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


# --- Human-readable command labels -----------------------------------------
# Build the "nicely written" labels printed to the console and written to the
# audit log (shared with the web app). Convention: each word capitalised, a
# degree sign on temperatures, and the power state is the only all-caps on/off
# (ON/OFF); every other toggle uses title case (On/Off).
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


# ===========================================================================
# Frame -> IR Toy byte stream
# ===========================================================================
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
# Transports
# ---------------------------------------------------------------------------
# IR Toy needs a byte pipe with read/write/timeout. Local USB is a real
# serial port (pyserial). Network sharing via ser2net/socat is a plain TCP
# stream -- and pyserial's socket:// wrapper does NOT cleanly support serial
# operations like reset_input_buffer()/flush() over a socket (they can drop
# the connection -> BrokenPipe). So for the network case we use a raw socket
# directly and expose the same tiny interface the driver needs.
# ===========================================================================
class _SerialTransport:
    """Local USB IR Toy via pyserial."""
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
            self._read_until_empty()

    def _read_until_empty(self):
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
    """Remote IR Toy shared over TCP (ser2net / socat). Plain socket, no
    serial-port emulation -- avoids the socket:// BrokenPipe problems."""
    def __init__(self, host, port, timeout):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        # TCP_NODELAY: send small control bytes immediately, no Nagle batching
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout

    def write(self, data):
        # sendall handles partial writes; no flush concept on a socket
        self.sock.sendall(data)

    def read(self, n):
        """Read up to n bytes, honouring the socket timeout. Returns b'' on
        timeout (matching pyserial's read semantics)."""
        self.sock.settimeout(self.timeout)
        chunks = []
        remaining = n
        try:
            while remaining > 0:
                b = self.sock.recv(remaining)
                if not b:            # peer closed
                    break
                chunks.append(b)
                remaining -= len(b)
                # For control reads we usually want exactly n, but the IR Toy
                # sends them in one go; if we got something, don't block forever
                # waiting for the rest -- one short read is enough to proceed.
                if chunks and remaining > 0:
                    self.sock.settimeout(0.15)
        except socket.timeout:
            pass
        except OSError:
            pass
        return b"".join(chunks)

    def drain_input(self):
        """Discard any buffered incoming bytes without closing the socket."""
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


# ===========================================================================
# USB IR Toy v2 driver -- transport-agnostic
# ===========================================================================
class IrToy:
    def __init__(self, transport, verbose=False):
        self.t = transport
        self.verbose = verbose
        time.sleep(0.1)
        self._init_device()

    def _log(self, *a):
        if self.verbose:
            print("[irtoy]", *a)

    def _init_device(self):
        self.t.write(b"\x00" * 5)        # reset to bulk from any mode
        time.sleep(0.1)
        self.t.drain_input()
        self.t.write(b"S")               # enter sample mode
        time.sleep(0.05)
        proto = self.t.read(3)
        if not proto.startswith(b"S"):
            raise IOError(
                f"IR Toy did not enter sample mode (returned: {proto!r}). "
                "Check: lircd stopped? ser2net at 115200? Correct port/host?"
            )
        self._log("sample mode banner:", proto)
        self.t.write(bytes([0x26, 0x25, 0x24]))   # byte-count, notify, handshake
        time.sleep(0.05)
        self.t.drain_input()

    def transmit_once(self, payload):
        self.t.drain_input()
        self.t.write(bytes([0x03]))      # start transmit
        hs = self.t.read(1)
        if not hs:
            raise IOError("No handshake response after 0x03.")
        free = hs[0]
        self._log("free buffer:", free)
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
        # NOT a fixed 8 bytes. A blocking read of a fixed size waits for the
        # full serial timeout for bytes that never arrive. Give the ~60 ms
        # burst a brief moment to emit, then clear the reported bytes.
        time.sleep(0.08)
        self.t.drain_input()
        self._log("completion: drained")

    def send_frame(self, state, repeat=1, gap_ms=40):
        payload = timings_to_irtoy(frame_to_timings(state))
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


def open_irtoy(host=None, port=2000, device="/dev/ttyACM0", timeout=5, verbose=False):
    """Factory: pick serial or socket transport and return an IrToy."""
    if host:
        transport = _SocketTransport(host, port, timeout)
    else:
        transport = _SerialTransport(device, timeout)
    return IrToy(transport, verbose=verbose)


# ===========================================================================
# State file
# ===========================================================================
def load_state(path):
    try:
        with open(path) as f:
            st = json.load(f)
        merged = dict(DEFAULT_STATE)
        merged.update(st)
        return merged
    except (FileNotFoundError, json.JSONDecodeError):
        return dict(DEFAULT_STATE)


def save_state(path, state):
    state = dict(state)
    state["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        with open(path, "w") as f:
            json.dump(state, f, indent=2)
    except OSError as e:
        print(f"  warning: cannot save state file: {e}", file=sys.stderr)


def print_status(state):
    print("Last known AC state:")
    print(f"  Power      : {state['power']}")
    print(f"  Mode       : {state['mode']}")
    print(f"  Temperature: {state['temp']}C")
    print(f"  Fan        : {state['fan']}")
    print(f"  Jet mode   : {state['jet']}")
    print(f"  LED display: {state.get('light', '?')}")
    print(f"  Air purify : {state.get('purify', '?')}")
    print(f"  Moisture   : {state.get('moisture', '?')}")
    print(f"  Quiet mode : {state.get('quiet', '?')}")
    print(f"  Energy ctrl: {state.get('energy', '?')}")
    print(f"  Vert. swing: {state['vswing'] if state['vswing'] else '-'}")
    print(f"  Updated    : {state['updated'] if state['updated'] else '-'}")
    print()
    print("  (state is best-effort; the physical remote does not update it)")


# ===========================================================================
# Main
# ===========================================================================
def build_parser():
    p = argparse.ArgumentParser(
        description="Yalgacr CLI v1.0 -- LG AC control (AKB74955603) via USB IR Toy v2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Examples:")[1] if "Examples:" in __doc__ else None,
    )
    p.add_argument("--version", action="version", version="Yalgacr CLI v1.0")
    # Power
    p.add_argument("--on", action="store_true", help="Turn AC on (last known state)")
    p.add_argument("--off", action="store_true", help="Turn AC off")
    # Climate parameters
    p.add_argument("--mode", choices=list(MODE_N3.keys()),
                   help="Operating mode: cool/dry/fan/auto/heat")
    p.add_argument("--temp", type=int, help="Temperature (COOL 18-30, HEAT 16-30)")
    p.add_argument("--fan", choices=list(FAN_N1.keys()),
                   help="Fan: 1/2/3/4/5/auto")
    # Jet
    p.add_argument("--jet", choices=["on", "off"], help="Jet mode on/off")
    # Swing
    p.add_argument("--vswing", choices=list(VSWING.keys()),
                   help="Vertical swing: 1-6/full/off")
    # Light
    p.add_argument("--light", nargs="?", const="toggle",
                   choices=["on", "off", "toggle"],
                   help="LED display: on/off/toggle (frame is the same, only the "
                        "recorded state changes). No argument = toggle.")
    # Special functions (each has distinct on/off codes)
    p.add_argument("--purify", choices=["on", "off"],
                   help="Air purification (removes particles): on/off")
    p.add_argument("--moisture", choices=["on", "off"],
                   help="Indoor unit moisture removal: on/off")
    p.add_argument("--quiet", choices=["on", "off"],
                   help="Outdoor unit quiet mode (noise reduction): on/off")
    p.add_argument("--energy", choices=["80", "60", "40", "off"],
                   help="Power-saving level: 80/60/40/off")
    # Status / debug
    p.add_argument("--status", action="store_true", help="Show state (without sending)")
    p.add_argument("--raw", help="Send an arbitrary 28-bit frame, e.g. 0x88C0051")
    # Connection
    p.add_argument("--host", help="IP of remote IR Toy (ser2net/socat)")
    p.add_argument("--port", type=int, default=2000, help="TCP port (default 2000)")
    p.add_argument("--device", default="/dev/ttyACM0", help="Local device")
    # Behaviour
    p.add_argument("--repeat", type=int, default=1,
                   help="Send frame N times (default 1)")
    p.add_argument("--state-file", default=DEFAULT_STATE_FILE,
                   help=f"State file path (default {DEFAULT_STATE_FILE})")
    p.add_argument("--dry", action="store_true",
                   help="Compute and print frame, do not send")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()

    print("Yalgacr CLI v1.0")

    state = load_state(args.state_file)

    # --- status only ---
    if args.status:
        print_status(state)
        return

    # Determine connection target
    if args.host:
        timeout = 5
        conn_desc = f"remote IR Toy ({args.host}:{args.port})"
    else:
        timeout = 2
        conn_desc = f"local IR Toy ({args.device})"

    # Build the queue of (description, frame, state_mutation) to send.
    # state_mutation is a dict applied to `state` after a successful send.
    queue = []

    # --- raw debug frame ---
    if args.raw:
        frame = int(args.raw, 16) & 0xFFFFFFF
        queue.append((f"Raw 0x{frame:07X}", frame, {}, f"Raw 0x{frame:07X}"))

    # --- power off ---
    elif args.off:
        queue.append(("Power OFF", FRAME_OFF, {"power": "off"}, "Power OFF"))

    # --- power on ---
    elif args.on:
        mode = args.mode or state["mode"]
        temp = args.temp or state["temp"]
        fan = args.fan or state["fan"]
        frame = encode_power_on(mode, temp, fan)
        queue.append((f"Power ON({_fmt_climate(mode, temp, fan)})", frame,
                      {"power": "on", "mode": mode, "temp": temp, "fan": fan},
                      "Power ON"))

    # --- jet ---
    elif args.jet == "on":
        queue.append(("Jet", FRAME_JET, {"power": "on", "jet": "on"},
                      "Jet"))
    elif args.jet == "off":
        # Jet off = resend current climate state
        mode = args.mode or state["mode"]
        temp = args.temp or state["temp"]
        fan = args.fan or state["fan"]
        frame = encode_climate(mode, temp, fan)
        queue.append(("Jet Off", frame,
                      {"jet": "off", "mode": mode, "temp": temp, "fan": fan},
                      "Jet Off"))

    # --- light on/off/toggle ---
    # The AC sends the SAME toggle frame regardless of direction; we only
    # differ in what we record. 'on'/'off' force the recorded state,
    # 'toggle' flips whatever is currently recorded.
    elif args.light is not None:
        if args.light == "on":
            new_light = "on"
        elif args.light == "off":
            new_light = "off"
        else:  # toggle
            new_light = "off" if state.get("light") == "on" else "on"
        queue.append((f"Light {_fmt_onoff(new_light)}", FRAME_LIGHT, {"light": new_light},
                      f"Light {_fmt_onoff(new_light)}"))

    # --- air purify (distinct on/off codes) ---
    elif args.purify is not None:
        frame = FRAME_PURIFY_ON if args.purify == "on" else FRAME_PURIFY_OFF
        queue.append((f"Purify {_fmt_onoff(args.purify)}", frame, {"purify": args.purify},
                      f"Purify {_fmt_onoff(args.purify)}"))

    # --- moisture removal (distinct on/off codes) ---
    elif args.moisture is not None:
        frame = FRAME_MOISTURE_ON if args.moisture == "on" else FRAME_MOISTURE_OFF
        queue.append((f"Moisture {_fmt_onoff(args.moisture)}", frame,
                      {"moisture": args.moisture}, f"Moisture {_fmt_onoff(args.moisture)}"))

    # --- quiet mode (distinct on/off codes) ---
    elif args.quiet is not None:
        frame = FRAME_QUIET_ON if args.quiet == "on" else FRAME_QUIET_OFF
        queue.append((f"Quiet {_fmt_onoff(args.quiet)}", frame, {"quiet": args.quiet},
                      f"Quiet {_fmt_onoff(args.quiet)}"))

    # --- energy control (four explicit levels) ---
    elif args.energy is not None:
        frame = FRAME_ENERGY[args.energy]
        queue.append((_fmt_energy(args.energy), frame, {"energy": args.energy},
                      _fmt_energy(args.energy)))

    # --- swing (independent, fire-and-forget) ---
    elif args.vswing is not None:
        frame = encode_vswing(args.vswing)
        queue.append((_fmt_vswing(args.vswing), frame, {"vswing": args.vswing},
                      _fmt_vswing(args.vswing)))
    # --- climate change (mode/temp/fan) ---
    elif args.mode or args.temp is not None or args.fan:
        mode = args.mode or state["mode"]
        temp = args.temp if args.temp is not None else state["temp"]
        fan = args.fan or state["fan"]
        try:
            frame = encode_climate(mode, temp, fan)
        except ValueError as e:
            sys.exit(f"Error: {e}")
        # Log exactly what the user asked to change (one or more of mode/temp/fan).
        changed = []
        if args.mode:
            changed.append(f"Mode {_fmt_mode(mode)}")
        if args.temp is not None:
            changed.append(f"Temp {int(temp)}°C")
        if args.fan:
            changed.append(_fmt_fan(fan))
        queue.append((_fmt_climate(mode, temp, fan), frame,
                      {"power": "on", "mode": mode, "temp": temp, "fan": fan},
                      ", ".join(changed)))

    else:
        parser.print_help()
        sys.exit("\nError: no command specified.")

    # When the AC is off, only power-on is allowed -- this mirrors the physical
    # remote, where no button works while the unit is off. --off is harmless
    # (already off) and --raw is an intentional debug bypass, so both are let
    # through; everything else is rejected.
    if state.get("power") != "on" and not (args.on or args.off or args.raw):
        sys.exit("Error: AC is off — turn it on first (lgac-cli.py --on)")

    # --- dry run ---
    print(f"Target: {conn_desc}")
    for desc, frame, _mut, _log in queue:
        print(f"  {desc:<32} frame=0x{frame:07X}")
    if args.dry:
        print("  (dry run -- nothing was sent)")
        return

    # --- transmit ---
    toy = open_irtoy(host=args.host, port=args.port, device=args.device,
                     timeout=timeout, verbose=args.verbose)
    try:
        for desc, frame, mutation, logstr in queue:
            toy.send_frame(frame, repeat=args.repeat)
            state.update(mutation)
            log_event(logstr)
            print(f"  -> sent: {desc}")
    finally:
        toy.close()

    save_state(args.state_file, state)


if __name__ == "__main__":
    main()
