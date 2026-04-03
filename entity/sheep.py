from entity.base_agent import BaseAgent
import pygame

class Sheep(BaseAgent):
    def __init__(self, x, y):
        super().__init__(x, y, (255, 255, 255)) # 白色
        self.max_speed = 100.0 # 羊跑得比狗慢一點

        # load sheep image
        self.image = pygame.image.load("imgs/sheep.png").convert_alpha()
        self.image = pygame.transform.scale(self.image, (35, 35))
