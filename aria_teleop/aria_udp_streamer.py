"""
Aria Gen 2 Hand Data UDP Streamer
==================================
Runs on Linux/WSL where the Aria Client SDK is supported.
Streams hand tracking data from Aria Gen 2 glasses to the Windows-side
bridge (aria_piper_bridge.py) over UDP.

Uses the Gen 2 SDK API:
  - aria.sdk_gen2 for device connection
  - aria.stream_receiver for typed streaming callbacks
  - register_hand_pose_callback() for hand tracking data

Usage (on Linux/WSL):
  pip install projectaria-client-sdk
  python aria_udp_streamer.py --target-ip <WINDOWS_IP> --target-port 5010
"""
import argparse
import json
import socket
import sys
import time

import aria.sdk_gen2 as sdk_gen2
import aria.stream_receiver as receiver
import aria.sdk as sdk_gen1 # For log control

from projectaria_tools.core.mps import hand_tracking

# Aria Gen 2 hand landmark indices (different from MediaPipe!)
# See: projectaria_tools.core.mps.hand_tracking.HandLandmark
ARIA_THUMB_FINGERTIP = 0
ARIA_INDEX_FINGERTIP = 1
ARIA_MIDDLE_FINGERTIP = 2
ARIA_RING_FINGERTIP = 3
ARIA_PINKY_FINGERTIP = 4
ARIA_WRIST = 5
ARIA_NUM_LANDMARKS = 21


class HandTrackingUDPStreamer:
    """Receives hand tracking and streams it over UDP."""

    def __init__(self, target_ip: str, target_port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (target_ip, target_port)
        self.frame_count = 0
        self.last_print = 0.0

    def on_hand_tracking(self, ht_data: hand_tracking.HandTrackingResult):
        """Typed callback invoked by StreamReceiver for each hand tracking frame."""
        try:
            # Prefer right hand, fall back to left
            hand = ht_data.right_hand if ht_data.right_hand is not None else ht_data.left_hand
            if hand is None:
                return

            packet = {
                "type": "hand_tracking",
                "ts": ht_data.tracking_timestamp.total_seconds(),
                "confidence": float(hand.confidence),
                "handedness": "right" if ht_data.right_hand is not None else "left",
            }

            # Wrist and palm positions (3D, in device frame, meters)
            wrist = hand.get_wrist_position_device()
            palm = hand.get_palm_position_device()
            if wrist is not None:
                packet["wrist_pos"] = [float(wrist[0]), float(wrist[1]), float(wrist[2])]
            if palm is not None:
                packet["palm_pos"] = [float(palm[0]), float(palm[1]), float(palm[2])]

            # Wrist and palm normals
            if hand.wrist_and_palm_normal_device is not None:
                normals = hand.wrist_and_palm_normal_device
                wn = normals.wrist_normal_device
                pn = normals.palm_normal_device
                if wn is not None:
                    packet["wrist_normal"] = [float(wn[0]), float(wn[1]), float(wn[2])]
                if pn is not None:
                    packet["palm_normal"] = [float(pn[0]), float(pn[1]), float(pn[2])]

            # Hand landmarks (21 keypoints)
            if hasattr(hand, 'hand_landmarks') and hand.hand_landmarks is not None:
                landmarks = []
                for lm in hand.hand_landmarks:
                    landmarks.append([float(lm[0]), float(lm[1]), float(lm[2])])
                packet["landmarks"] = landmarks

            # Send UDP
            data = json.dumps(packet).encode('utf-8')
            self.sock.sendto(data, self.target)

            self.frame_count += 1
            now = time.time()
            if now - self.last_print > 2.0:
                self.last_print = now
                conf = packet.get("confidence", 0)
                side = packet.get("handedness", "?")
                has_lm = "landmarks" in packet
                print(f"[streamer] Frame {self.frame_count} | "
                      f"{side} hand | conf={conf:.2f} | "
                      f"landmarks={'yes' if has_lm else 'no'} | "
                      f"-> {self.target[0]}:{self.target[1]}")

        except Exception as e:
            print(f"[streamer] Error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Stream Aria Gen 2 hand tracking over UDP")
    parser.add_argument("--target-ip", required=True,
                        help="Windows machine IP address")
    parser.add_argument("--target-port", type=int, default=5010,
                        help="UDP port to send to (default: 5010)")
    parser.add_argument("--profile", default="profile9",
                        help="Aria streaming profile (default: profile9)")
    parser.add_argument("--aria-ip", default=None,
                        help="Aria glasses IP address (for Wi-Fi connection, get from companion app)")
    parser.add_argument("--interface", default="usb",
                        choices=["usb", "wifi"],
                        help="Streaming interface (default: usb)")
    parser.add_argument("--duration", type=int, default=0,
                        help="Duration in seconds (0=infinite)")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose SDK logging")
    args = parser.parse_args()

    if args.verbose:
        # Set SDK log level to Debug (using the sdk_gen1 module which controls general logging)
        sdk_gen1.set_log_level(sdk_gen1.Level.Debug)
        print("[streamer] Verbose logging enabled.")

    # Auto-switch to wifi if aria-ip is provided
    if args.aria_ip:
        args.interface = "wifi"

    # ── 1. Connect to Aria glasses ────────────────────────────────────────
    print("[streamer] Connecting to Aria Gen 2 glasses...")
    device_client = sdk_gen2.DeviceClient()
    config = sdk_gen2.DeviceClientConfig()
    if args.aria_ip:
        config.ip_v4_address = args.aria_ip
        print(f"[streamer] Using Wi-Fi connection to {args.aria_ip}")
    device_client.set_client_config(config)

    try:
        # The connect() call can hang if there's a network issue.
        # Ensure the phone app live-streaming is closed!
        device = device_client.connect()
    except Exception as e:
        print(f"[streamer] Connection failed: {e}")
        print("[streamer] Make sure your Aria glasses are:")
        print("  - Powered on and paired via companion app")
        print("  - Connected via USB (or use --interface wifi)")
        return

    print(f"[streamer] Connected!")
    try:
        status = device.status()
        print(f"[streamer] Battery: {status.battery_level}%")
    except Exception as e:
        print(f"[streamer] Could not get battery status: {e}")

    # ── 2. Configure streaming ────────────────────────────────────────────
    streaming_config = sdk_gen2.HttpStreamingConfig()
    streaming_config.profile_name = args.profile
    if args.interface == "usb":
        streaming_config.streaming_interface = sdk_gen2.StreamingInterface.USB_NCM
    else:
        streaming_config.streaming_interface = sdk_gen2.StreamingInterface.WIFI_STA
    
    try:
        device.set_streaming_config(streaming_config)
    except Exception as e:
        print(f"[streamer] Failed to set streaming config: {e}")
        return


    # ── 3. Start streaming from device ────────────────────────────────────
    try:
        device.start_streaming()
        print(f"[streamer] Device streaming started (profile={args.profile})")
    except Exception as e:
        print(f"[streamer] WARNING: start_streaming() failed: {e}")
        print("[streamer] Proceeding anyway, the device might already be streaming...")

    # ── 4. Set up stream receiver with hand tracking callback ─────────────
    server_config = sdk_gen2.HttpServerConfig()
    server_config.address = "0.0.0.0"
    server_config.port = 6768

    stream_receiver = receiver.StreamReceiver(
        enable_image_decoding=False,  # We don't need images
        enable_raw_stream=False,
    )
    stream_receiver.set_server_config(server_config)

    # Create UDP streamer and register callback
    udp_streamer = HandTrackingUDPStreamer(args.target_ip, args.target_port)
    stream_receiver.register_hand_pose_callback(udp_streamer.on_hand_tracking)
    print(f"[streamer] Hand tracking callback registered")
    print(f"[streamer] Sending data to {args.target_ip}:{args.target_port}")

    # Start the receiving server
    stream_receiver.start_server()
    print("[streamer] Stream receiver started. Waiting for hand data...")
    print("[streamer] Press Ctrl+C to stop.")

    try:
        if args.duration > 0:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[streamer] Shutting down...")
    finally:
        device.stop_streaming()
        time.sleep(1)
        stream_receiver.stop_server()
        device_client.disconnect(device)
        udp_streamer.sock.close()
        print("[streamer] Done.")


if __name__ == "__main__":
    main()
