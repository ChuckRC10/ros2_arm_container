import pygame
import jax.numpy as jnp
from config import armColor, screenSize

def game_setup():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(screenSize)
    return clock, screen

def get_origin():
    origin = jnp.array([screenSize[0]/2, screenSize[1]/2])
    return origin
        
def paint_rect(screen, wantedPosition):
    origin = get_origin()
    rectanglePosition = wantedPosition + origin
    pointerRectangle = pygame.Rect(float(rectanglePosition[0]), float(rectanglePosition[1]), 5, 5)
    pygame.draw.rect(screen, armColor, pointerRectangle) 

def paint_arm(screen, armVectors: jnp.array):
    origin = jnp.array(get_origin())

    for armNum in range(len(armVectors)):
        if armNum == 0:
            startingPosition = origin
        else:
          startingPosition = origin + jnp.sum(armVectors[0:armNum], 0)
        endingPosition = origin + jnp.sum(armVectors[0:armNum + 1], 0)
        pygame.draw.line(screen, armColor, startingPosition.tolist(), endingPosition.tolist(), 5)