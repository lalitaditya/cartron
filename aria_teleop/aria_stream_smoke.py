"""
Minimal Aria classic SDK streaming smoke test.

This intentionally has no GUI and mirrors aria_debug_stream.py closely. Use it
to verify that DDS callbacks arrive before debugging EyeTrack visualization.
"""

import os
import subprocess
import time
import argparse
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
FASTDDS_XML = os.path.join(CARTRON_ROOT, "fastdds_aria_unicast.xml")


def requested_interface() -> str:
    for index, arg in enumerate(os.sys.argv):
        if arg == "--interface" and index + 1 < len(os.sys.argv):
            return os.sys.argv[index + 1]
        if arg.startswith("--interface="):
            return arg.split("=", 1)[1]
    return "usb"


def configure_fastdds_profile() -> None:
    if requested_interface() == "wifi":
        for env_name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
            env_value = os.environ.get(env_name, "")
            if os.path.basename(env_value).startswith("fastdds_aria_unicast"):
                os.environ.pop(env_name, None)
        print("[smoke] Wi-Fi mode: not using the USB-NCM FastDDS profile.")
        return

    if os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        print(f"[smoke] Using FastDDS profile from env: {os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']}")
    elif os.path.isfile(FASTDDS_XML):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = FASTDDS_XML
        os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = FASTDDS_XML
        print(f"[smoke] Using FastDDS unicast profile: {FASTDDS_XML}")


configure_fastdds_profile()

import aria.sdk as aria


def ping_command(ip_address: str):
    if sys.platform.startswith("win"):
        return ["ping", "-n", "2", "-w", "2000", ip_address]
    return ["ping", "-c", "2", "-W", "2", ip_address]


class Observer:
    def __init__(self):
        self.total = 0
        self.images = 0
        self.imu = 0
        self.cameras = {}

    def on_image_received(self, image, record):
        self.total += 1
        self.images += 1
        self.cameras[record.camera_id] = self.cameras.get(record.camera_id, 0) + 1
        print(f"[smoke] IMAGE camera_id={record.camera_id} shape={image.shape}")

    def on_imu_received(self, samples, imu_idx: int):
        count = len(samples) if samples is not None else 1
        self.total += count
        self.imu += count
        print(f"[smoke] IMU imu_idx={imu_idx} samples={count}")

    def on_streaming_client_failure(self, reason, message: str):
        print(f"[smoke] FAILURE {reason}: {message}")


def parse_args():
    parser = argparse.ArgumentParser(description="Minimal Aria streaming callback smoke test")
    parser.add_argument(
        "--serial",
        default=os.environ.get("ARIA_SERIAL", ""),
        help="Optional Aria device serial. Defaults to first device found.",
    )
    parser.add_argument("--interface", choices=["usb", "wifi"], default="usb")
    parser.add_argument("--device-ip", default="", help="Aria IP address for Wi-Fi streaming.")
    parser.add_argument("--profile", default="profile18", help="Streaming profile.")
    parser.add_argument(
        "--ephemeral-certs",
        action="store_true",
        help="Use ephemeral streaming certs for both stream manager and subscriber.",
    )
    parser.add_argument(
        "--stop-previous",
        action="store_true",
        help="Try to stop a previous streaming session before starting.",
    )
    parser.add_argument(
        "--client-mode",
        choices=["standalone", "manager"],
        default="standalone",
        help="Use a standalone StreamingClient after USB-NCM is up, or the manager-provided client.",
    )
    parser.add_argument(
        "--data",
        choices=["rgb", "rgb-slam-imu", "rgb-eye", "eye"],
        default="rgb-slam-imu",
        help="Which stream types to subscribe to for the smoke test.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    aria.set_log_level(aria.Level.Info)

    device_client = aria.DeviceClient()
    client_config = aria.DeviceClientConfig()
    if args.serial:
        client_config.device_serial = args.serial
    if args.device_ip:
        client_config.ip_v4_address = args.device_ip
    device_client.set_client_config(client_config)

    target = args.device_ip or args.serial or "first available device"
    print(f"[smoke] Connecting to {target}...")
    device = device_client.connect()
    print("[smoke] Connected.")

    streaming_manager = device.streaming_manager

    if args.stop_previous:
        try:
            print("[smoke] Stopping any previous streaming session...")
            streaming_manager.stop_streaming()
            time.sleep(2)
        except Exception as exc:
            print(f"[smoke] Previous stream stop skipped: {exc}")
    else:
        print("[smoke] Skipping previous-stream stop; starting directly.")

    streaming_config = aria.StreamingConfig()
    streaming_config.profile_name = args.profile
    if args.interface == "usb":
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
    streaming_config.security_options.use_ephemeral_certs = args.ephemeral_certs
    streaming_manager.streaming_config = streaming_config

    print(f"[smoke] Starting streaming profile={args.profile} interface={args.interface}...")
    streaming_manager.start_streaming()
    print(f"[smoke] Streaming state: {streaming_manager.streaming_state}")

    print("[smoke] Waiting 5s for network/DDS...")
    time.sleep(5)
    ping_ip = args.device_ip or "192.168.42.129"
    ret = subprocess.run(ping_command(ping_ip), capture_output=True)
    print(f"[smoke] ping {ping_ip} returncode={ret.returncode}")
    if not sys.platform.startswith("win"):
        subprocess.run(["ip", "-brief", "addr"], check=False)
        subprocess.run(["ip", "route", "get", ping_ip], check=False)

    # Create/access the DDS streaming client only after USB-NCM exists.
    # In WSL, creating it before the 192.168.42.100 interface is up can leave
    # FastDDS bound to the wrong interfaces and result in zero callbacks.
    if args.client_mode == "standalone":
        print("[smoke] Creating standalone StreamingClient after USB-NCM is up.")
        streaming_client = aria.StreamingClient()
    else:
        print("[smoke] Using StreamingManager-provided StreamingClient.")
        streaming_client = streaming_manager.streaming_client

    config = streaming_client.subscription_config
    if args.data == "rgb":
        config.subscriber_data_type = aria.StreamingDataType.Rgb
    elif args.data == "rgb-eye":
        config.subscriber_data_type = aria.StreamingDataType.Rgb | aria.StreamingDataType.EyeTrack
    elif args.data == "eye":
        config.subscriber_data_type = aria.StreamingDataType.EyeTrack
    else:
        config.subscriber_data_type = (
            aria.StreamingDataType.Rgb
            | aria.StreamingDataType.Slam
            | aria.StreamingDataType.Imu
        )
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1
    config.message_queue_size[aria.StreamingDataType.Slam] = 1
    config.message_queue_size[aria.StreamingDataType.EyeTrack] = 1

    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = args.ephemeral_certs
    config.security_options = options
    streaming_client.subscription_config = config

    observer = Observer()
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()
    print("[smoke] Subscribed. Waiting 15s for callbacks...")

    try:
        for i in range(15):
            time.sleep(1)
            print(
                f"[smoke] {i + 1}s total={observer.total} images={observer.images} "
                f"imu={observer.imu} cameras={observer.cameras or 'none'}"
            )
    finally:
        try:
            streaming_client.unsubscribe()
        except Exception:
            pass
        try:
            streaming_manager.stop_streaming()
        except Exception:
            pass
        try:
            device_client.disconnect(device)
        except Exception:
            pass
        print("[smoke] Done.")


if __name__ == "__main__":
    main()
