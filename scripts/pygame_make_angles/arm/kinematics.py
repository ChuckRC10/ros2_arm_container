import jax.numpy as jnp

def inv_kinematics_least_sqr(Jacobian: jnp.array, error: jnp.array, damping_c: float) -> list:
    """Calculate delta q, damped least-squares"""
    eye_size = Jacobian.shape[1]
    damping_matrix = jnp.linalg.inv(Jacobian.T @ Jacobian + damping_c**2 * jnp.eye(eye_size))
    delta_q = damping_matrix @ Jacobian.T @ error
    return delta_q