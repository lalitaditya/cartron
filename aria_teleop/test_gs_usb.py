import can

def test_gs_usb():
    print("Testing GS_USB interface on Windows...")
    try:
        # For gs_usb on Windows, channel is usually the serial number or index
        bus = can.interface.Bus(interface='gs_usb', index=0, bitrate=1000000)
        print("SUCCESS: Connected to GS_USB!")
        msg = bus.recv(timeout=1.0)
        if msg:
            print(f"Message received: {msg}")
        else:
            print("Connected but no messages yet (waiting for robot heartbeat).")
        bus.shutdown()
    except Exception as e:
        print(f"GS_USB Connection Failed: {e}")
        print("Note: You might need to use Zadig to install WinUSB driver for the candleLight.")

if __name__ == "__main__":
    test_gs_usb()
