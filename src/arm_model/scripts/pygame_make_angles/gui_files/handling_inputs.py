# Import pygame.locals for easier access to key coordinates
# Updated to conform to flake8 and black standards
import jax.numpy as jnp
from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
)

def get_movement(pressed_key, maxPositionDelta) -> jnp.array:
    '''
    returns how far the pointer will move on the screen based on key inputs
    '''
    xMovement = 0
    yMovement = 0
    # get user input
    if pressed_key[K_UP]:
        yMovement = -maxPositionDelta
    if pressed_key[K_DOWN]:
        yMovement = maxPositionDelta
    if pressed_key[K_LEFT]:
        xMovement = -maxPositionDelta
    if pressed_key[K_RIGHT]:
        xMovement = maxPositionDelta

    movementDeltaVector = jnp.array([xMovement, yMovement])
    return movementDeltaVector