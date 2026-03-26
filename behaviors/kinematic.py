import random
import math
from utils.vector_math import Vector2

class KinematicBehaviors:
    @staticmethod
    def seek(agent, target_pos):
        """
        Kinematic Seek: 直接將速度指向目標
        """
        # 1. 計算朝向目標的向量
        desired_direction = target_pos - agent.pos
        
        if desired_direction.length_squared() > 0:
            # 2. 將速度設定為最大速度並指向目標
            desired_direction.scale_to_length(agent.max_speed)
            agent.vel = desired_direction
        else:
            agent.vel = Vector2(0, 0)

    @staticmethod
    def wander(agent, dt, wander_state):
        """
        Kinematic Wander: 隨機改變朝向
        wander_state: 一個字典，用來儲存上一次的隨機角度
        """
        # 取得目前的隨機角度，若無則初始化
        current_angle = wander_state.get("angle", agent.orientation)
        
        # 每幀隨機改變一點角度 (隨機抖動)
        change_rate = 0.5  # 弧度
        current_angle += random.uniform(-change_rate, change_rate)
        
        # 更新狀態
        wander_state["angle"] = current_angle
        
        # 根據角度計算速度向量
        new_vel = Vector2(1, 0).rotate_rad(current_angle)
        new_vel.scale_to_length(agent.max_speed * 0.5) # 漫遊時速度通常慢一點
        
        agent.vel = new_vel