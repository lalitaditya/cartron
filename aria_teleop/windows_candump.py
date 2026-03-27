import can
import time

def windows_candump(channel='COM4', bustype='slcan', bitrate=1000000):
    print(f"Monitoring {channel} at {bitrate}bps... Press Ctrl+C to stop.")
    try:
        bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        while True:
            msg = bus.recv(1.0)
            if msg:
                print(msg)
            else:
                print("No data received in 1s...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Done.")

if __name__ == "__main__":
    windows_candump()
