import can
import time

def check_state(port):
    print(f"Testing {port} with bustype='slcan'...")
    try:
        bus = can.interface.Bus(channel=port, bustype='slcan', bitrate=1000000)
        print(f"Bus opened. Current state: {bus.state}")
        
        # Try to send a dummy message (e.g. search version)
        msg = can.Message(arbitration_id=0x111, data=[0]*8, is_extended_id=False)
        try:
            bus.send(msg)
            print("Send successful!")
        except Exception as e:
            print(f"Send failed: {e}")
            
        print("Waiting for any message (2s)...")
        start = time.time()
        while time.time() - start < 2:
            m = bus.recv(0.1)
            if m:
                print(f"Received: {m}")
                break
        else:
            print("No messages received.")
            
        bus.shutdown()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM4'
    check_state(port)
