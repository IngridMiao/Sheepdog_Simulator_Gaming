import pygame
import math

# 直接繼承或包裝 Pygame 的 Vector2 
Vector2 = pygame.math.Vector2

def limit_vector(vec, max_value):
    """限制向量的最大長度，用於速度或加速度限制"""
    if vec.length_squared() > max_value**2:
        return vec.normalize() * max_value
    return vec

def get_orientation(velocity):
    """根據當前速度向量計算朝向角度 (弧度) """
    if velocity.length_squared() > 0:
        return math.atan2(velocity.y, velocity.x)
    return 0