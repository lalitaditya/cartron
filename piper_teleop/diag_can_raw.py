"""
Raw CAN adapter diagnostic — bypasses Piper SDK entirely.
Tests python-can send/receive at different serial baud rates.
"""
import sys
import time

import can
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"

print(f"=== Raw CAN Adapter Diagnostic on {PORT} ===\n")

# ── Step 1: Check what the serial port responds to ──
print("[1] Raw serial probe:")
BAUD_RATES = [115200, 921600, 1000000, 2000000, 460800, 500000, 256000]
for baud in BAUD_RATES:
    try:
        ser = serial.Serial(PORT, baud, timeout=0.5)
        time.sleep(0.1)
        # SLCAN: close channel, then request version
        ser.write(b"C\r")
        time.sleep(0.1)
        ser.flushInput()
        ser.write(b"V\r")
        time.sleep(0.3)
        resp = ser.read(200)
        ser.close()
        flag = "** HAS DATA **" if resp else "(empty)"
        print(f"  {baud:>8} baud: V response = {resp!r} {flag}")
        if resp:
            # Try full SLCAN init at this baud
            print(f"    → Testing SLCAN init at {baud}...")
            ser = serial.Serial(PORT, baud, timeout=0.5)
            ser.write(b"C\r"); time.sleep(0.1)
            ser.write(b"S8\r"); time.sleep(0.1)  # S8 = 1Mbps CAN
            ser.write(b"O\r"); time.sleep(0.1)   # Open channel
            # Try sending a CAN frame (ID=0x000, empty)
            ser.write(b"t0000\r"); time.sleep(0.3)
            resp2 = ser.read(200)
            print(f"    → After init+send: {resp2!r}")
            ser.write(b"C\r")
            ser.close()
    except Exception as e:
        print(f"  {baud:>8} baud: ERROR: {e}")

# ── Step 2: Try python-can SLCAN at different serial bauds ──
print("\n[2] python-can SLCAN bus test:")
for baud in [115200, 921600, 2000000, 1000000]:
    try:
        bus = can.interface.Bus(channel=PORT, interface="slcan",
                                bitrate=1000000, tty_baudrate=baud)
        state = bus.state
        # Try sending
        msg = can.Message(arbitration_id=0x151, data=[0]*8, is_extended_id=False)
        try:
            bus.send(msg, timeout=0.5)
            send_ok = "SEND OK"
        except Exception as e:
            send_ok = f"SEND FAIL: {e}"
        # Try receiving
        rx = bus.recv(timeout=0.5)
        recv_ok = f"RECV: {rx}" if rx else "RECV: None (no response)"
        bus.shutdown()
        print(f"  tty_baud={baud}: state={state}, {send_ok}, {recv_ok}")
    except Exception as e:
        print(f"  tty_baud={baud}: FAILED: {e}")

# ── Step 3: Try python-can 'serial' interface  ──
print("\n[3] python-can 'serial' (generic) bus test:")
try:
    bus = can.interface.Bus(channel=PORT, interface="serial", bitrate=1000000)
    state = bus.state
    msg = can.Message(arbitration_id=0x151, data=[0]*8, is_extended_id=False)
    try:
        bus.send(msg, timeout=0.5)
        send_ok = "SEND OK"
    except Exception as e:
        send_ok = f"SEND FAIL: {e}"
    rx = bus.recv(timeout=0.5)
    recv_ok = f"RECV: {rx}" if rx else "RECV: None"
    bus.shutdown()
    print(f"  state={state}, {send_ok}, {recv_ok}")
except Exception as e:
    print(f"  FAILED: {e}")

# ── Step 4: Try robotell (another serial CAN protocol) ──
print("\n[4] python-can 'robotell' bus test:")
for baud in [115200, 2000000]:
    try:
        bus = can.interface.Bus(channel=PORT, interface="robotell",
                                bitrate=1000000, tty_baudrate=baud)
        msg = can.Message(arbitration_id=0x151, data=[0]*8, is_extended_id=False)
        try:
            bus.send(msg, timeout=0.5)
            send_ok = "SEND OK"
        except Exception as e:
            send_ok = f"SEND FAIL: {e}"
        rx = bus.recv(timeout=0.5)
        recv_ok = f"RECV: {rx}" if rx else "RECV: None"
        bus.shutdown()
        print(f"  tty_baud={baud}: {send_ok}, {recv_ok}")
    except Exception as e:
        print(f"  tty_baud={baud}: FAILED: {e}")

print("\n=== Done ===")
