import numpy as np


class armClass:
    def __init__(self, arm_lengths: np.ndarray, arm_angles: np.ndarray):
        self.arm_lengths = np.asarray(arm_lengths, dtype=float)
        self.arm_angles = np.asarray(arm_angles, dtype=float)

    @staticmethod
    def _pure_get_end_effector(angles: np.ndarray, arm_lengths: np.ndarray) -> np.ndarray:
        """Pure NumPy forward kinematics matching the original behavior.

        angles: [base, q1, q2,...] where angles[1:] are plane joint angles
        arm_lengths: array of link lengths (n links)
        returns: 3-vector end-effector position
        """
        angles = np.asarray(angles, dtype=float)
        arm_lengths = np.asarray(arm_lengths, dtype=float)

        # 1. Get 2D vectors (original __get2dArmVectors__ logic)
        angles2d = angles[1:]
        globalAngles = np.cumsum(angles2d)
        xArray = arm_lengths * np.sin(globalAngles)
        yArray = arm_lengths * np.cos(globalAngles)
        vectors_2d = np.vstack([xArray, yArray]).T

        # 2. Rotate to 3D (original __rotate_arm_vectors__ logic)
        base_angle = angles[0]
        cos_theta = np.cos(base_angle)
        sin_theta = np.sin(base_angle)
        Rz = np.array([
            [cos_theta, -sin_theta, 0.0],
            [sin_theta,  cos_theta, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=float)

        x_coords = vectors_2d[:, 0]
        z_coords = vectors_2d[:, 1]
        y_coords = np.zeros(arm_lengths.shape[0], dtype=float)
        vectors_3d_flat = np.column_stack([x_coords, y_coords, z_coords])
        vectors_3d = (Rz @ vectors_3d_flat.T).T

        # 3. Sum for end effector position (original get_end_effector logic)
        return np.sum(vectors_3d, axis=0)

    def get_end_effector(self, angles) -> np.ndarray:
        """Public method to get end effector position."""
        return armClass._pure_get_end_effector(angles, self.arm_lengths)

    def get_jacobian(self) -> np.ndarray:
        """Finite-difference Jacobian (numpy) for robustness and no extra deps."""
        f = lambda angs: armClass._pure_get_end_effector(angs, self.arm_lengths)
        n = int(self.arm_angles.shape[0])
        eps = 1e-6

        base = f(self.arm_angles)
        J = np.zeros((3, n), dtype=float)
        for i in range(n):
            pert = self.arm_angles.copy()
            pert[i] += eps
            out = f(pert)
            J[:, i] = (out - base) / eps
        return J


def inv_kinematics_least_sqr(Jacobian: np.ndarray, error: np.ndarray, damping_c: float) -> np.ndarray:
    """Calculate delta q, damped least-squares (numpy)."""
    J = np.asarray(Jacobian, dtype=float)
    error = np.asarray(error, dtype=float)
    eye_size = J.shape[1]
    damping_matrix = np.linalg.inv(J.T @ J + (damping_c ** 2) * np.eye(eye_size))
    delta_q = damping_matrix @ J.T @ error
    return delta_q