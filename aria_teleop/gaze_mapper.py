import numpy as np
import math
from typing import Tuple, Optional, List

class GazeMapper:
    """
    Transforms Meta Aria eye gaze and head pose into the Vehicle/Robot frame.
    
    Frames:
      - Aria (Device/Camera Frame): X-Right, Y-Down, Z-Forward
      - Vehicle (ROS/Standard Frame): X-Forward, Y-Left, Z-Up
    """

    def __init__(self, vehicle_cam_offset: Optional[np.ndarray] = None):
        """
        Args:
            vehicle_cam_offset: (3,) static offset [x, y, z] in vehicle frame 
                               representing where the virtual 'driver eyes' are.
        """
        # Axis swap matrix (Aria Device -> Vehicle Base)
        self._R_swap = np.array([
            [0,  0, 1],  # Veh X = Aria Z
            [-1, 0, 0],  # Veh Y = -Aria X
            [0, -1, 0]   # Veh Z = -Aria Y
        ])
        
        self.cam_offset = vehicle_cam_offset if vehicle_cam_offset is not None else np.zeros(3)

    def transform_gaze_vector(self, gaze_vec_aria: np.ndarray) -> np.ndarray:
        """
        Transforms a unit gaze vector from Aria CPF frame to Vehicle frame.
        
        Args:
            gaze_vec_aria: (3,) unit vector [x, y, z] in Aria space.
        Returns:
            (3,) unit vector [x, y, z] in Vehicle space.
        """
        # Ensure it's a numpy array
        g_aria = np.array(gaze_vec_aria).flatten()
        
        # Apply the axis rotation
        g_vehicle = self._R_swap @ g_aria
        
        # Re-normalize to ensure floating point errors don't drift the vector length
        norm = np.linalg.norm(g_vehicle)
        return g_vehicle / norm if norm > 1e-6 else g_vehicle

    def transform_head_pose(self, head_pos_aria: np.ndarray, 
                            head_rot_aria_quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transforms a 6-DOF head pose (Position + Quaternion) into the Vehicle frame.
        
        Args:
            head_pos_aria: (3,) position [x,y,z] in meters.
            head_rot_aria_quat: (4,) quaternion [x,y,z,w] in Hamilton convention.
            
        Returns:
            (pos_vehicle, quat_vehicle)
        """
        # 1. Position Transformation + Offset
        p_aria = np.array(head_pos_aria)
        # Note: Since the USER is stationary, the Aria head_pos_aria is often [0,0,0]
        # or relative to an initial 'home' frame. We treat it as a delta to the vehicle seat.
        p_vehicle = (self._R_swap @ p_aria) + self.cam_offset
        
        # 2. Orientation Transformation
        # We need to rotate the quaternion or the equivalent rotation matrix.
        # This is more robust with a 3x3 rotation matrix.
        R_head_aria = self.quat_to_rot_matrix(head_rot_aria_quat)
        
        # R_vehicle = R_swap * R_head_aria * R_swap.T (Change of basis)
        R_vehicle = self._R_swap @ R_head_aria @ self._R_swap.T
        
        q_vehicle = self.rot_matrix_to_quat(R_vehicle)
        
        return p_vehicle, q_vehicle

    @staticmethod
    def quat_to_rot_matrix(q):
        """Standard quaternion (x,y,z,w) to 3x3 matrix conversion."""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
            [2*y*z - 2*x*w,         2*x*z + 2*y*w,       1 - 2*x*x - 2*y*y] # This is actually transposed, fixing:
        ]).T # Quat conversion usually produces row-major or needs transpose

    @staticmethod
    def rot_matrix_to_quat(R):
        """Standard 3x3 rotation matrix to quaternion (x,y,z,w) conversion."""
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return np.array([qx, qy, qz, qw])
