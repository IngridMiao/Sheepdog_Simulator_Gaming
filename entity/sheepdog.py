from entity.base_agent import BaseAgent
import pygame

class SheepDog(BaseAgent):
    def __init__(self, x, y):
        super().__init__(x, y, (160, 82, 45)) # 棕色
        self.max_speed = 180.0

        # load dog image
        self.image = pygame.image.load("imgs/dog.png").convert_alpha()
        self.image = pygame.transform.scale(self.image, (50, 50))