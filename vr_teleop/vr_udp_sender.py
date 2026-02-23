import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

try:
    import openvr
except ImportError:
    print("ERROR: openvr python package not installed. Run: pip install openvr")
    sys.exit(1)


@dataclass
class ControllerState:
    # Pose (meters) and orientation (quaternion)
    pos: Tuple[float, float, float]
    rot: Tuple[float, float, float, float]  # (x,y,z,w)
    vel: Tuple[float, float, float]
    ang_vel: Tuple[float, float, float]
    # Inputs
    trigger: float
    grip: float
    joystick_xy: Tuple[float, float]
    trackpad_xy: Tuple[float, float]
    buttons: Dict[str, bool]


def hmd_matrix34_to_pose(m):
    """
    Convert OpenVR HmdMatrix34_t to position + quaternion.
    m is a 3x4 row-major matrix: [[r00 r01 r02 tx],
                                 [r10 r11 r12 ty],
                                 [r20 r21 r22 tz]]
    """
    r00, r01, r02, tx = m[0]
    r10, r11, r12, ty = m[1]
    r20, r21, r22, tz = m[2]

    # Quaternion from rotation matrix (robust method)
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif (r00 > r11) and (r00 > r22):
        s = (1.0 + r00 - r11 - r22) ** 0.5 * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = (1.0 + r11 - r00 - r22) ** 0.5 * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = (1.0 + r22 - r00 - r11) ** 0.5 * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s

    return (tx, ty, tz), (qx, qy, qz, qw)


def get_role_from_index(vrsys, device_index: int) -> str:
    role = vrsys.getControllerRoleForTrackedDeviceIndex(device_index)
    if role == openvr.TrackedControllerRole_LeftHand:
        return "left"
    if role == openvr.TrackedControllerRole_RightHand:
        return "right"
    return "unknown"


def read_controller_inputs(vrsys, device_index: int) -> Tuple[float, float, Tuple[float, float], Tuple[float, float], Dict[str, bool]]:
    """
    Reads controller state (buttons/axes). Mappings vary by controller.
    This works for most SteamVR controllers; you mainly need trigger + a few buttons.
    """
    result, state = vrsys.getControllerState(device_index)
    if not result:
        return 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), {}

    trigger = float(getattr(state.rAxis[1], "x", 0.0))
    grip = float(getattr(state.rAxis[2], "x", 0.0))

    joy_xy = (float(getattr(state.rAxis[0], "x", 0.0)), float(getattr(state.rAxis[0], "y", 0.0)))
    pad_xy = (float(getattr(state.rAxis[2], "x", 0.0)), float(getattr(state.rAxis[2], "y", 0.0)))

    def pressed(btn_id: int) -> bool:
        mask = 1 << btn_id
        return (state.ulButtonPressed & mask) != 0

    def touched(btn_id: int) -> bool:
        mask = 1 << btn_id
        return (state.ulButtonTouched & mask) != 0

    buttons = {
        "system": pressed(openvr.k_EButton_System),
        "appmenu": pressed(openvr.k_EButton_ApplicationMenu),
        "grip_button": pressed(openvr.k_EButton_Grip),
        "dpad_left": pressed(openvr.k_EButton_DPad_Left),
        "dpad_right": pressed(openvr.k_EButton_DPad_Right),
        "dpad_up": pressed(openvr.k_EButton_DPad_Up),
        "dpad_down": pressed(openvr.k_EButton_DPad_Down),
        "a": pressed(openvr.k_EButton_A),
        "trigger_click": pressed(openvr.k_EButton_SteamVR_Trigger),
        "touchpad_click": pressed(openvr.k_EButton_SteamVR_Touchpad),
        "touchpad_touch": touched(openvr.k_EButton_SteamVR_Touchpad),
    }

    return trigger, grip, joy_xy, pad_xy, buttons


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1", help="Receiver IP (Ubuntu/Windows)")
    ap.add_argument("--port", type=int, default=5005, help="Receiver UDP port")
    ap.add_argument("--hz", type=float, default=90.0, help="Target send rate (e.g., 60 or 90)")
    ap.add_argument("--print", action="store_true", help="Print a short status line once per second")
    args = ap.parse_args()

    addr = (args.ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Init OpenVR
    openvr.init(openvr.VRApplication_Background)
    vrsys = openvr.VRSystem()

    t0 = time.perf_counter()
    last_print = t0
    sent = 0

    period = 1.0 / max(1e-6, args.hz)

    # Pre-allocate pose array for getDeviceToAbsoluteTrackingPose
    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses_array = poses_t()

    try:
        while True:
            loop_start = time.perf_counter()

            vrsys.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0.0,
                poses_array
            )
            poses = poses_array

            payload = {
                "ts": time.time(),
                "t_mono": loop_start,
                "seq": sent,
                "controllers": {},
            }

            for i in range(openvr.k_unMaxTrackedDeviceCount):
                if not vrsys.isTrackedDeviceConnected(i):
                    continue

                dev_class = vrsys.getTrackedDeviceClass(i)
                if dev_class != openvr.TrackedDeviceClass_Controller:
                    continue

                role = get_role_from_index(vrsys, i)
                pose = poses[i]

                if not pose.bPoseIsValid:
                    continue

                m = pose.mDeviceToAbsoluteTracking
                mat34 = [
                    [m[0][0], m[0][1], m[0][2], m[0][3]],
                    [m[1][0], m[1][1], m[1][2], m[1][3]],
                    [m[2][0], m[2][1], m[2][2], m[2][3]],
                ]

                pos, quat = hmd_matrix34_to_pose(mat34)

                vel = (pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2])
                ang_vel = (pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2])

                trigger, grip, joy_xy, pad_xy, buttons = read_controller_inputs(vrsys, i)

                payload["controllers"][role] = {
                    "device_index": i,
                    "pose_valid": True,
                    "pos_m": [pos[0], pos[1], pos[2]],
                    "quat_xyzw": [quat[0], quat[1], quat[2], quat[3]],
                    "vel_mps": [vel[0], vel[1], vel[2]],
                    "ang_vel_radps": [ang_vel[0], ang_vel[1], ang_vel[2]],
                    "trigger": trigger,
                    "grip": grip,
                    "joystick_xy": [joy_xy[0], joy_xy[1]],
                    "trackpad_xy": [pad_xy[0], pad_xy[1]],
                    "buttons": buttons,
                }

            msg = json.dumps(payload).encode("utf-8")
            sock.sendto(msg, addr)
            sent += 1

            now = time.perf_counter()
            if args.print and (now - last_print) >= 1.0:
                roles = list(payload["controllers"].keys())
                print(f"[sender] sent={sent} roles={roles} target_hz={args.hz}")
                last_print = now

            # rate control
            elapsed = time.perf_counter() - loop_start
            sleep_s = period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n[sender] Ctrl+C received, exiting.")
    finally:
        openvr.shutdown()


if __name__ == "__main__":
    main()
