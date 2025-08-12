import pygame
import jax.numpy as jnp
from gui_files.game_view import game_setup, paint_arm, paint_rect
from gui_files.handling_inputs import get_movement
from config import backgroundColor, maxPositionDelta, armLength, armNumber, dampingConstant
from arm.arm_model import RobotArm
from arm import kinematics

def calculate_angles_loop(output_queue):

    clock, screen = game_setup() # initialize window
    armLengths = armLength * jnp.ones((armNumber))
    arm = RobotArm(armLengths) # initialize arm class

    wntd_pos = arm.get_end_effector(arm.armAngles)

    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False  
            else:
                running = True

        screen.fill(backgroundColor)

        # get info to move arm
        pressed_key = pygame.key.get_pressed()
        movementDeltaVector = get_movement(pressed_key, maxPositionDelta)

        # update wanted position
        wntd_pos = wntd_pos + movementDeltaVector

        # get change in arm angles
        jacobian = arm.get_jacobian()
        error = arm.get_error(wntd_pos)
        delta_q = kinematics.inv_kinematics_least_sqr(jacobian, error, dampingConstant)

        # update arm angles
        arm.set_angles(arm.armAngles + delta_q)

        # set new arm vectors
        armVectors = arm.getArmVectors(arm.armAngles)

        # add arm angles to queue
        current_angles = [0, arm.armAngles[0], arm.armAngles[1]]
        output_queue.put(current_angles)
        # 2nd screen update
        paint_rect(screen, wntd_pos)
        paint_arm(screen, armVectors)

        # refresh screen
        pygame.display.flip()
        clock.tick(30) # ensure program maintains 30fps

    pygame.quit()

