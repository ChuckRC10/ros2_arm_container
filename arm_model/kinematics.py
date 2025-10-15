import jax
import jax.numpy as jnp
from jax.numpy import sin, cos

# (The armClass and inv_kinematics_least_sqr functions remain unchanged)
class armClass:
    def __init__(self, arm_lengths:jnp.array, arm_angles:jnp.array):
        self.arm_lengths = arm_lengths
        self.arm_angles = arm_angles

    def getArmVectors(self, angles) -> jnp.array:
        vectors_2d = self.__get2dArmVectors__(angles)
        vectors_3d = self.__rotate_arm_vectors__(vectors_2d, angles)
        return vectors_3d

    def __get2dArmVectors__(self, angles):
        angles2d = angles[1:]
        globalAngles = jnp.cumsum(angles2d)
        xArray = self.arm_lengths * sin(globalAngles)
        yArray = self.arm_lengths * cos(globalAngles)
        armVectorArray = jnp.array([xArray, yArray]).T
        return armVectorArray

    def __rotate_arm_vectors__(self, vectors_2d, angles):
        base_angle = angles[0]
        cos_theta = cos(base_angle)
        sin_theta = sin(base_angle)
        Rz = jnp.array([[cos_theta, -sin_theta, 0], [sin_theta,  cos_theta, 0], [0, 0, 1]])
        x_coords = vectors_2d[:, 0]
        z_coords = vectors_2d[:, 1]
        y_coords = jnp.zeros(self.arm_lengths.shape[0])
        vectors_3d_flat = jnp.column_stack([x_coords, y_coords, z_coords])
        return (Rz @ vectors_3d_flat.T).T

    def get_end_effector(self, angles) -> jnp.array:
        armVectorArray = self.getArmVectors(angles)
        armEndVector = jnp.sum(armVectorArray, axis=0)
        return armEndVector
    
    def get_jacobian(self) -> jnp.array:
        J = jax.jacrev(lambda angles: self.get_end_effector(angles))
        return J(self.arm_angles)
    
def inv_kinematics_least_sqr(Jacobian: jnp.array, error: jnp.array, damping_c: float) -> list:
    eye_size = Jacobian.shape[1]
    damping_matrix = jnp.linalg.inv(Jacobian.T @ Jacobian + damping_c**2 * jnp.eye(eye_size))
    delta_q = damping_matrix @ Jacobian.T @ error
    return delta_q