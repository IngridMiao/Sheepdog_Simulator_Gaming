import pygame
from utils.vector_math import Vector2

class Obstacle:
    def __init__(self, x, y, width, height, obs_type="STONE"):
        self.pos = Vector2(x, y)
        self.width = width
        self.height = height
        self.type = obs_type.upper() # "STONE" 或 "FENCE"
        
        # 根據類型設定顏色
        if self.type == "STONE":
            self.color = (128, 128, 128)  # 灰色
            self.radius = max(width, height) / 2 # 石頭以圓形處理
        else: # FENCE
            self.color = (139, 69, 19)    # 棕色 (SaddleBrown)
            self.rect = pygame.Rect(x - width/2, y - height/2, width, height)

    def draw(self, screen):
        if self.type == "STONE":
            # 繪製石頭
            pygame.draw.circle(screen, self.color, (int(self.pos.x), int(self.pos.y)), int(self.radius))
            pygame.draw.circle(screen, (80, 80, 80), (int(self.pos.x), int(self.pos.y)), int(self.radius), 2)
        
        elif self.type == "FENCE":
            # 繪製柵欄
            pygame.draw.rect(screen, self.color, self.rect)
            pygame.draw.rect(screen, (60, 30, 10), self.rect, 2) # 深棕色

    def get_bounds(self):
        """用於碰撞偵測的邊界資訊"""
        if self.type == "STONE":
            return ("CIRCLE", self.pos, self.radius)
        return ("RECT", self.rect)