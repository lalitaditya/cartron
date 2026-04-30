#!/usr/bin/env python3
"""Quick integration test for the Aria -> Piper pipeline.
Uses Aria Gen 2 landmark indices (THUMB_FINGERTIP=0, INDEX_FINGERTIP=1, WRIST=5).
"""
import sys
import os

# Path setup
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARTRON_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(CARTRON_ROOT, 'vr_teleop'))
sys.path.insert(0, os.path.join(CARTRON_ROOT, 'piper_teleop'))

import numpy as np
from hand_pose_mapper import HandPoseMapper, WRIST, THUMB_TIP, INDEX_TIP, INDEX_MCP, MIDDLE_MCP
from vr_piper_bridge import PiperIK

def test_ik_integration():
    """Test that hand landmarks produce valid IK solutions."""
    mapper = HandPoseMapper()
    ik = PiperIK()

    # Aria Gen 2 indices: WRIST=5, THUMB_TIP=0, INDEX_TIP=1, INDEX_MCP=8, MIDDLE_MCP=11
    landmarks = np.zeros((21, 3))
    landmarks[WRIST] = [0.0, -0.1, 0.35]
    landmarks[THUMB_TIP] = [0.02, -0.15, 0.38]
    landmarks[INDEX_TIP] = [-0.01, -0.16, 0.32]
    landmarks[INDEX_MCP] = [0.01, -0.16, 0.36]
    landmarks[MIDDLE_MCP] = [-0.005, -0.16, 0.35]

    target, grip, dbg = mapper.map_landmarks(landmarks)
    joints = ik.solve(target)
    ee = ik.get_ee_pos(joints)

    print(f"Target:  ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) mm")
    print(f"EE:      ({ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f}) mm")
    print(f"Grip:    {grip:.4f}")
    print(f"Pinch:   {dbg['pinch_dist_m']*100:.1f} cm")
    print(f"Joints:  " + " ".join(f"J{i+1}={joints[i]:.3f}" for i in range(6)))

    assert all(-3.15 < j < 3.15 for j in joints), "Joints out of range!"
    assert 0.0 <= grip <= 0.035, "Grip out of range!"
    print("PASS: IK integration")

def test_pinch_detection():
    """Test that pinch maps to correct gripper values."""
    mapper = HandPoseMapper(smoothing=0.0)

    # Open hand
    lm_open = np.zeros((21, 3))
    lm_open[WRIST] = [0.0, -0.1, 0.35]
    lm_open[THUMB_TIP] = [0.05, -0.12, 0.35]
    lm_open[INDEX_TIP] = [-0.03, -0.15, 0.35]
    lm_open[INDEX_MCP] = [0.01, -0.16, 0.36]
    lm_open[MIDDLE_MCP] = [-0.005, -0.16, 0.35]
    _, grip_open, dbg_open = mapper.map_landmarks(lm_open)

    mapper.reset_origin()

    # Closed pinch
    lm_closed = np.zeros((21, 3))
    lm_closed[WRIST] = [0.0, -0.1, 0.35]
    lm_closed[THUMB_TIP] = [0.01, -0.13, 0.35]
    lm_closed[INDEX_TIP] = [0.015, -0.135, 0.35]
    lm_closed[INDEX_MCP] = [0.01, -0.16, 0.36]
    lm_closed[MIDDLE_MCP] = [-0.005, -0.16, 0.35]
    _, grip_closed, dbg_closed = mapper.map_landmarks(lm_closed)

    print(f"Open:   dist={dbg_open['pinch_dist_m']*100:.1f}cm  grip={grip_open:.4f}")
    print(f"Closed: dist={dbg_closed['pinch_dist_m']*100:.1f}cm  grip={grip_closed:.4f}")

    assert grip_open < 0.01, f"Open grip should be near 0, got {grip_open}"
    assert grip_closed > 0.02, f"Closed grip should be >0.02, got {grip_closed}"
    print("PASS: Pinch detection")

def test_sim_tracker():
    """Test simulation tracker generates valid frames."""
    from sim_hand_tracker import SimHandTracker
    import time

    frames = []
    tracker = SimHandTracker(on_hand_update=frames.append, mode="circle", hz=30.0)
    tracker.start()
    time.sleep(0.5)
    tracker.stop()

    assert len(frames) > 5, f"Expected >5 frames, got {len(frames)}"
    assert frames[0].landmarks is not None
    assert frames[0].landmarks.shape == (21, 3)
    # WRIST (index 5) should be non-zero
    assert np.linalg.norm(frames[0].landmarks[5]) > 0, "Wrist should be non-zero"
    print(f"PASS: Sim tracker ({len(frames)} frames in 0.5s)")

if __name__ == "__main__":
    print("=" * 50)
    print("  Aria -> Piper Integration Tests")
    print("=" * 50)
    test_ik_integration()
    print()
    test_pinch_detection()
    print()
    test_sim_tracker()
    print()
    print("  ALL TESTS PASSED")
    print("=" * 50)
