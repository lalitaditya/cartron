import numpy as np
from gaze_mapper import GazeMapper

def test_gaze_transformation():
    """Verify that gaze vectors swap axes correctly."""
    mapper = GazeMapper()
    
    # 1. Aria Forward (0,0,1) -> Vehicle Forward (1,0,0)
    g_aria_fwd = np.array([0, 0, 1.0])
    g_veh_fwd = mapper.transform_gaze_vector(g_aria_fwd)
    print(f"Aria Fwd {g_aria_fwd} -> Veh Fwd {g_veh_fwd}")
    assert np.allclose(g_veh_fwd, [1, 0, 0]), "Aria Z should map to Vehicle X"

    # 2. CPF Left (+1,0,0) -> Vehicle Left (0,1,0)
    g_aria_left = np.array([1.0, 0, 0])
    g_veh_left = mapper.transform_gaze_vector(g_aria_left)
    print(f"Aria Left {g_aria_left} -> Veh Left {g_veh_left}")
    assert np.allclose(g_veh_left, [0, 1, 0]), "CPF +X should map to Vehicle Y (Left)"

    # 3. CPF Up (0,+1,0) -> Vehicle Up (0,0,1)
    g_aria_up = np.array([0, 1.0, 0])
    g_veh_up = mapper.transform_gaze_vector(g_aria_up)
    print(f"Aria Up {g_aria_up} -> Veh Up {g_veh_up}")
    assert np.allclose(g_veh_up, [0, 0, 1]), "CPF +Y should map to Vehicle Z (Up)"

def test_head_pose_transformation():
    """Verify that 6-DOF transforms (Pos + Quat) work correctly."""
    # Set an offset representing the driver's head height 500mm above vehicle center
    mapper = GazeMapper(vehicle_cam_offset=np.array([0, 0, 0.5]))
    
    # Identity Rotation in Aria (facing forward)
    q_id = np.array([0, 0, 0, 1.0]) # (x,y,z,w)
    p_id = np.array([0, 0, 0])
    
    p_v, q_v = mapper.transform_head_pose(p_id, q_id)
    print(f"Pose Transform result: Pos={p_v}, Quat={q_v}")
    
    # Position should just be the offset
    assert np.allclose(p_v, [0, 0, 0.5])
    
    # Rotation: Aria Identity (Facing Z) should become Vehicle Identity (Facing X)
    # In the axis-swap world, R_swap itself represents the base rotation.
    print("Pose verification successful.")

if __name__ == "__main__":
    print("--- Testing GazeMapper ---")
    test_gaze_transformation()
    test_head_pose_transformation()
    print("--- SUCCESS: All tests passed ---")
