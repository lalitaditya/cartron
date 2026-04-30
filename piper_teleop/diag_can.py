"""
Diagnostic: Test CAN communication with the Piper arm.
Checks if we can send AND receive CAN frames.
"""
import sys, os, time, platform

# Add parent dir for piper_sdk
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from piper_sdk import C_PiperInterface_V2

CAN_PORT = sys.argv[1] if len(sys.argv) > 1 else ("COM5" if platform.system() == "Windows" else "can0")
is_windows = platform.system() == "Windows"

print(f"=== Piper Arm CAN Diagnostic ===")
print(f"Port: {CAN_PORT}")
print(f"OS: {platform.system()}")
print()

# Step 1: Connect
print("[1/4] Connecting CAN bus...")
try:
    if is_windows:
        piper = C_PiperInterface_V2(CAN_PORT, can_auto_init=False)
        piper.CreateCanBus(CAN_PORT, bustype="slcan",
                           expected_bitrate=1000000, judge_flag=False)
    else:
        piper = C_PiperInterface_V2(CAN_PORT)
    piper.ConnectPort()
    print("  OK: CAN bus connected.")
except Exception as e:
    print(f"  FAIL: {e}")
    sys.exit(1)

# Step 2: Wait for arm status messages
print("\n[2/4] Waiting for arm status (3 seconds)...")
print("  (If the arm is powered on, we should see status data)")
time.sleep(3)

try:
    status = piper.GetArmStatus()
    print(f"  Arm Status: {status}")
except Exception as e:
    print(f"  GetArmStatus error: {e}")

# Check joint feedback
try:
    joints = piper.GetArmJointMsgNum()
    print(f"  Joint feedback: {joints}")
except Exception as e:
    print(f"  GetArmJointMsgNum error: {e}")

# Check end pose
try:
    endpose = piper.GetArmEndPoseMsgNum()
    print(f"  End pose: {endpose}")
except Exception as e:
    print(f"  GetArmEndPoseMsgNum error: {e}")

# Step 3: Try to enable
print("\n[3/4] Attempting to enable arm (5 seconds)...")
t0 = time.time()
enabled = False
attempt = 0
while time.time() - t0 < 5.0:
    attempt += 1
    result = piper.EnablePiper()
    if result:
        enabled = True
        print(f"  Enabled after {attempt} attempts ({time.time()-t0:.1f}s)!")
        break
    time.sleep(0.05)

if not enabled:
    print(f"  FAILED to enable after {attempt} attempts.")
    print("  Possible causes:")
    print("    - Arm is not powered on (check power supply)")
    print("    - Emergency stop is engaged")
    print("    - CAN adapter not communicating (wrong protocol?)")
    print("    - Wrong CAN port")

# Step 4: Check status again
print("\n[4/4] Post-enable status check...")
time.sleep(1)
try:
    status = piper.GetArmStatus()
    print(f"  Arm Status: {status}")
except Exception as e:
    print(f"  Error: {e}")

# Cleanup
try:
    piper.DisableArm(7)
except:
    pass

print("\n=== Done ===")
