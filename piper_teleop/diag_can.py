"""Diagnose CAN adapter type on Windows."""
import sys

# 1. Check serial
print("=== Serial Module ===")
try:
    import serial
    print(f"  pyserial version: {serial.VERSION}")
    print(f"  serial module location: {serial.__file__}")
except ImportError as e:
    print(f"  FAILED: {e}")

# 2. Check python-can
print("\n=== python-can ===")
try:
    import can
    print(f"  python-can version: {can.__version__}")
except ImportError as e:
    print(f"  FAILED: {e}")

# 3. List available interfaces
print("\n=== Available CAN interfaces ===")
try:
    from can.interfaces import BACKENDS
    for name in sorted(BACKENDS.keys()):
        print(f"  - {name}")
except Exception:
    print("  Could not list backends")

# 4. Try SLCAN on COM5
print("\n=== Test: SLCAN on COM5 ===")
try:
    bus = can.interface.Bus(channel="COM5", interface="slcan", bitrate=1000000)
    print(f"  Bus created: {bus}")
    print(f"  Bus state: {bus.state}")
    bus.shutdown()
    print("  SLCAN works!")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")

# 5. Try gs_usb
print("\n=== Test: gs_usb ===")
try:
    import gs_usb
    print(f"  gs_usb module found: {gs_usb.__file__}")
except ImportError:
    print("  gs_usb module not installed (pip install gs_usb)")

try:
    bus = can.interface.Bus(channel="COM5", interface="gs_usb", bitrate=1000000)
    print(f"  Bus created: {bus}")
    bus.shutdown()
    print("  gs_usb works!")
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")

# 6. Try opening COM5 raw to see what's there
print("\n=== Raw serial test on COM5 ===")
try:
    import serial
    ser = serial.Serial("COM5", 2000000, timeout=1)
    print(f"  Opened COM5 at 2000000 baud")
    # Try SLCAN version command
    ser.write(b"V\r")
    import time; time.sleep(0.3)
    resp = ser.read(100)
    print(f"  SLCAN 'V' response: {resp!r}")
    # Try SLCAN setup
    ser.write(b"S8\r")  # 1Mbps
    time.sleep(0.1)
    resp2 = ser.read(100)
    print(f"  SLCAN 'S8' response: {resp2!r}")
    ser.close()
except Exception as e:
    print(f"  FAILED: {type(e).__name__}: {e}")

print("\nDone!")
