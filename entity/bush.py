import pygame
from utils.vector_math import Vector2

class Bush:
    def __init__(self, x, y):
        self.pos = Vector2(x, y)
        
        # 載入並縮放草叢圖片
        self.image = pygame.image.load("imgs/bamboo02.png").convert_alpha()
        self.image = pygame.transform.scale(self.image, (50, 50))

    def draw(self, screen):
        rect = self.image.get_rect(center=(int(self.pos.x), int(self.pos.y)))
        screen.blit(self.image, rect)
