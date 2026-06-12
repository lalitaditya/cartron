#!/usr/bin/env python3

import argparse
import sys


def device_id(info) -> str:
    if hasattr(info, "mxid"):
        return str(info.mxid)
    if hasattr(info, "deviceId"):
        return str(info.deviceId)
    if hasattr(info, "getDeviceId"):
        return str(info.getDeviceId())
    return "unknown"


def parse_args():
    parser = argparse.ArgumentParser(description="Check and view an OAK/DepthAI RGB camera.")
    parser.add_argument("--show", action="store_true", help="Open a live camera window.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    return parser.parse_args()


def show_frame(cv2, window_name, frame):
    # DepthAI gives RGB here; OpenCV windows expect BGR for natural colors.
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow(window_name, bgr)
    key = cv2.waitKey(1) & 0xFF
    return key in (ord("q"), 27)


def run_depthai_v2(dai, cv2, device_info, args):
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(args.width, args.height)
    cam.setFps(args.fps)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    with dai.Device(pipeline, device_info) as device:
        queue = device.getOutputQueue(name="rgb", maxSize=1, blocking=True)
        frame = queue.get().getCvFrame()
        print(f"RGB frame OK: shape={frame.shape}, dtype={frame.dtype}")

        if args.show:
            print("Showing live camera stream. Press q or Esc to quit.")
            while True:
                frame = queue.get().getCvFrame()
                if show_frame(cv2, "DepthAI RGB", frame):
                    break


def run_depthai_v3(dai, cv2, args):
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        rgb_output = cam.requestOutput(
            (args.width, args.height),
            type=dai.ImgFrame.Type.RGB888p,
            fps=args.fps,
        )
        queue = rgb_output.createOutputQueue()

        pipeline.start()
        frame = queue.get().getCvFrame()
        print(f"RGB frame OK: shape={frame.shape}, dtype={frame.dtype}")

        if args.show:
            print("Showing live camera stream. Press q or Esc to quit.")
            while pipeline.isRunning():
                frame = queue.get().getCvFrame()
                if show_frame(cv2, "DepthAI RGB", frame):
                    break


def main() -> int:
    args = parse_args()
    try:
        import depthai as dai
    except ModuleNotFoundError:
        print("depthai is not installed. Install it with: python3 -m pip install depthai")
        return 1
    try:
        import cv2
    except ModuleNotFoundError:
        print("opencv-python is not installed. Install it with: python3 -m pip install opencv-python")
        return 1

    devices = dai.Device.getAllAvailableDevices()
    print(f"DepthAI devices found: {len(devices)}")
    for info in devices:
        print(
            f"- name={info.name} id={device_id(info)} "
            f"state={info.state} protocol={info.protocol}"
        )

    if not devices:
        print(
            "\nNo usable DepthAI devices were found. If lsusb shows 03e7:2485, "
            "install the Luxonis udev rule, reload udev, then unplug/replug the camera."
        )
        return 2

    try:
        if hasattr(dai.node, "XLinkOut"):
            run_depthai_v2(dai, cv2, devices[0], args)
        else:
            run_depthai_v3(dai, cv2, args)
    finally:
        if args.show:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
