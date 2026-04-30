import time
from piper_sdk import C_PiperInterface_V2

def test_can_feedback():
    print("--- Piper CAN Sniffer (Windows/COM5) ---")
    # piper_sdk v2 uses the COM port as the identifier
    piper = C_PiperInterface_V2("COM5")
    
    print("Connecting to COM5... (Make sure CAN adapter is plugged in)")
    if not piper.ConnectPort():
        print("[ERROR] Could not open COM5! Is it already in use by another program?")
        return

    print("Success! Port COM5 opened.")
    print("Listening for feedback from robot... (Moving the joints manually might trigger data)")
    print("Press Ctrl+C to stop.")

    try:
        count = 0
        while True:
            # Get latest arm status
            status = piper.GetArmStatus()
            arm_low_status = status.arm_low_status
            
            # Print if we see any non-zero feedback
            if arm_low_status.can_connected:
                print(f"[RECV] Frame #{count:04d} | Joint State: {arm_low_status.joint_feedback} | Error: {arm_low_status.arm_error_code}")
                count += 1
            else:
                if count % 10 == 0:
                    print("... (Still waiting for CAN heartbeat. Check robot power!)")
                    
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        pass # V2 SDK closes it automatically if it goes out of scope, or just let it exit.

if __name__ == "__main__":
    test_can_feedback()
