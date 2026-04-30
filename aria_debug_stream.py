"""
Minimal debug script to check if ANY data arrives from Aria Gen 1 SDK.
"""
import sys
import os
import time
import subprocess
import numpy as np

# ── Force FastDDS unicast (multicast is broken in WSL2) ──
_FASTDDS_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "fastdds_aria_unicast.xml")
if os.path.isfile(_FASTDDS_XML) and "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ:
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _FASTDDS_XML
    print(f"[fastdds] Using unicast profile: {_FASTDDS_XML}")

import aria.sdk as aria


class DebugObserver:
    """Observer that prints EVERYTHING it receives."""

    def __init__(self):
        self.total = 0

    def on_image_received(self, image: np.ndarray, record) -> None:
        self.total += 1
        print(f"  [IMAGE #{self.total}] camera_id={record.camera_id} "
              f"shape={image.shape} ts={record.capture_timestamp_ns}")

    def on_imu_received(self, samples, imu_idx: int) -> None:
        self.total += 1
        print(f"  [IMU #{self.total}] imu_idx={imu_idx}")

    def on_magneto_received(self, sample) -> None:
        self.total += 1
        print(f"  [MAG #{self.total}]")

    def on_baro_received(self, sample) -> None:
        self.total += 1
        print(f"  [BARO #{self.total}]")

    def on_streaming_client_failure(self, reason, message: str) -> None:
        print(f"  [FAILURE] {reason}: {message}")


if __name__ == "__main__":
    aria.set_log_level(aria.Level.Info)

    print("=== Connecting to Aria glasses ===")
    device_client = aria.DeviceClient()
    client_config = aria.DeviceClientConfig()
    client_config.device_serial = "1WM103500C1272"
    device_client.set_client_config(client_config)
    device = device_client.connect()
    print("Connected!")

    streaming_manager = device.streaming_manager
    streaming_client = streaming_manager.streaming_client

    # Stop any stuck streaming session from a previous run
    try:
        streaming_manager.stop_streaming()
        print("Stopped previous streaming session.")
        time.sleep(2)
    except Exception:
        pass  # no streaming was active, that's fine

    streaming_config = aria.StreamingConfig()
    streaming_config.profile_name = "profile18"
    streaming_config.streaming_interface = aria.StreamingInterface.Usb
    streaming_config.security_options.use_ephemeral_certs = False
    streaming_manager.streaming_config = streaming_config

    print("Starting USB streaming (glasses will re-enumerate as UsbNcm)...")
    print("Make sure watch_aria_usb.ps1 is running in an Admin PowerShell!")
    print("  Waiting for start_streaming()... (this may take 30-60s)")
    streaming_manager.start_streaming()
    print("Streaming started!")

    # Give WSL time to detect the new USB NCM interface
    print("Waiting 5s for USB NCM network interface to settle...")
    time.sleep(5)

    # Check if 192.168.42.129 is reachable
    print("Checking connectivity to glasses (192.168.42.129)...")
    try:
        ret = subprocess.run(
            ["ping", "-c", "2", "-W", "2", "192.168.42.129"],
            capture_output=True, timeout=10
        )
        if ret.returncode == 0:
            print("  Aria glasses reachable at 192.168.42.129!")
        else:
            print("  WARNING: Cannot ping 192.168.42.129 - NCM interface may not be up")
            print("  Continuing anyway (DDS might still work)...")
    except Exception as e:
        print(f"  Ping check failed: {e}")


    config = streaming_client.subscription_config
    config.subscriber_data_type = (
        aria.StreamingDataType.Rgb |
        aria.StreamingDataType.Slam |
        aria.StreamingDataType.Imu
    )
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1
    config.message_queue_size[aria.StreamingDataType.Slam] = 1

    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = False
    config.security_options = options
    streaming_client.subscription_config = config

    observer = DebugObserver()
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()

    print("\n=== Listening for ANY data (15 seconds) ===")
    print("If nothing appears below, DDS transport is blocked.\n")

    try:
        for i in range(15):
            time.sleep(1)
            print(f"  ... {i+1}s elapsed, total callbacks: {observer.total}")
            if observer.total > 0:
                print("\n  DATA IS FLOWING!")
                break
        else:
            print("\n  NO DATA received in 15 seconds.")
            print("  DDS multicast is blocked in WSL.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        streaming_client.unsubscribe()
        streaming_manager.stop_streaming()
        device_client.disconnect(device)
        print("Disconnected.")
