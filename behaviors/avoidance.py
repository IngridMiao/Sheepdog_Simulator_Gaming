import pygame
from utils.vector_math import Vector2, limit_vector

class AvoidanceBehaviors:
    @staticmethod
    def obstacle_avoidance(agent, obstacles, look_ahead_dist=100.0):
        """
        避障邏輯：在 Agent 前方建立一條感測線
        """
        # 1. 計算感測向量 (基於目前速度方向)
        if agent.vel.length_squared() > 0:
            heading = agent.vel.normalize()
        else:
            heading = Vector2(1, 0).rotate_rad(agent.orientation)
            
        ahead = agent.pos + heading * look_ahead_dist
        
        # 2. 尋找最近的威脅障礙物
        most_threatening = None
        for obs in obstacles:
            collision = AvoidanceBehaviors._line_intersects_obstacle(agent.pos, ahead, obs)
            if collision:
                if most_threatening is None or \
                   agent.pos.distance_to(obs.pos) < agent.pos.distance_to(most_threatening.pos):
                    most_threatening = obs
        
        # 3. 計算避讓力 (Avoidance Force)
        avoidance = Vector2(0, 0)
        if most_threatening:
            # 產生一個遠離障礙物中心的側向力
            avoidance = ahead - most_threatening.pos
            avoidance = avoidance.normalize() * agent.max_force
            
        return avoidance, ahead # 回傳力與預測點位置(用於繪製 debug 線)

    @staticmethod
    def _line_intersects_obstacle(start, end, obs):
        """簡單判定：預測線上的點是否進入障礙物半徑/範圍"""
        # 針對圓形石頭的簡化判定
        if obs.type == "STONE":
            # 計算線段到圓心的最短距離 (這裡簡化為判斷端點與中點)
            mid = (start + end) / 2
            return end.distance_to(obs.pos) < obs.radius or mid.distance_to(obs.pos) < obs.radius
        
        # 針對長方形柵欄
        elif obs.type == "FENCE":
            return obs.rect.collidepoint(end.x, end.y)
            
        return False