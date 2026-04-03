import pygame
import math
from utils.vector_math import Vector2, get_orientation

class BaseAgent:
    def __init__(self, x, y, color):
        self.pos = Vector2(x, y)
        self.vel = Vector2(0, 0)
        self.accel = Vector2(0, 0)
        self.orientation = 0.0 
        
        # 限制參數
        self.max_speed = 150.0  # 像素/秒
        self.max_force = 200.0
        
        # 視覺化屬性
        self.color = color
        self.radius = 15
        
        # Part 1: 設計過的軌跡記錄 
        self.breadcrumb_trail = []
        self.last_trail_pos = Vector2(x, y)
        self.trail_threshold = 20.0  # 每移動 20 像素才記錄一個點 (距離取樣)
        self.max_trail_points = 30   # 限制點數防止記憶體無限增長

    def update(self, dt):
        """時間驅動的物理更新 (s = v * dt) """
        # 1. 更新速度 (考慮加速度)
        self.vel += self.accel * dt
        
        # 2. 限制最大速度
        if self.vel.length() > self.max_speed:
            self.vel.scale_to_length(self.max_speed)
            
        # 3. 更新位置
        self.pos += self.vel * dt
        
        # 4. 更新朝向 (Orientation) 
        if self.vel.length_squared() > 0.1:
            self.orientation = get_orientation(self.vel)
            
        # 5. 更新軌跡 (Trajectory Visualization Logic)
        if self.pos.distance_to(self.last_trail_pos) > self.trail_threshold:
            self.breadcrumb_trail.append(Vector2(self.pos.x, self.pos.y))
            self.last_trail_pos = Vector2(self.pos.x, self.pos.y)
            # 保持軌跡長度固定
            if len(self.breadcrumb_trail) > self.max_trail_points:
                self.breadcrumb_trail.pop(0)
        
        # 重置加速度
        self.accel *= 0

    def draw(self, screen, show_debug=True):
        # 繪製軌跡
        if len(self.breadcrumb_trail) > 1:
            pygame.draw.lines(screen, (150, 150, 150), False, self.breadcrumb_trail, 2)

        # 繪製 Agent 本體
        if hasattr(self, 'image') and self.image:
            # 若有設定圖片，則繪製圖片
            rect = self.image.get_rect(center=(int(self.pos.x), int(self.pos.y)))
            screen.blit(self.image, rect)
        else:
            # 預設：繪製具有方向性的三角形
            points = [
                self.pos + Vector2(self.radius, 0).rotate_rad(self.orientation),
                self.pos + Vector2(-self.radius, -self.radius/2).rotate_rad(self.orientation),
                self.pos + Vector2(-self.radius, self.radius/2).rotate_rad(self.orientation)
            ]
            pygame.draw.polygon(screen, self.color, points)