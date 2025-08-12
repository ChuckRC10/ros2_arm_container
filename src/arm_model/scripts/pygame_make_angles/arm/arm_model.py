import jax
import jax.numpy as jnp
from jax.numpy import sin, cos

class RobotArm:
    def __init__(self, armLengths:jnp.array):
        
        self.armLengths = armLengths
        self.armNumber = len(armLengths)
        self.armAngles = jnp.zeros(self.armNumber)

    def set_angles(self, armAngles: jnp.array):
        self.armAngles = armAngles

    def getArmVectors(self, angles) -> jnp.array:
        lens = self.armLengths
        globalAngles = jnp.cumsum(angles)

        # calculate vector coordinates
        xArray = lens * jnp.cos(globalAngles)
        yArray = lens * jnp.sin(globalAngles)
        
        armVectorArray = jnp.array([xArray, yArray]).T
        return armVectorArray

    def get_end_effector(self, angles) -> jnp.array:
        armVectorArray = self.getArmVectors(angles)
        armEndVector = jnp.sum(armVectorArray, axis=0)

        return armEndVector
    
    def get_jacobian(self) -> jnp.array:
        J = jax.jacrev(lambda angles: self.get_end_effector(angles))
        return J(self.armAngles)
    
    def get_error(self, wntd_pos: jnp.array) -> jnp.array:
        arm_pos = self.get_end_effector(self.armAngles)
        error = wntd_pos - arm_pos
        return error