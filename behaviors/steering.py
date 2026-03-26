from utils.vector_math import Vector2, limit_vector

class SteeringBehaviors:
    @staticmethod
    def seek(agent, target_pos):
        """
        Steering Seek: 計算轉向力 = 期望速度 - 當前速度
        """
        # 1. 期望速度 (Desired Velocity) = 直指目標的最大速度
        desired = target_pos - agent.pos
        if desired.length_squared() > 0:
            desired.scale_to_length(agent.max_speed)
            
            # 2. 轉向力 (Steering Force)
            steering = desired - agent.vel
            
            # 3. 限制轉向力的大小，模擬物理極限
            steering = limit_vector(steering, agent.max_force)
            return steering
        return Vector2(0, 0)

    @staticmethod
    def arrive(agent, target_pos, slow_radius=100.0):
        """
        Arrive: 進入 slow_radius 範圍後會逐漸減速
        """
        desired = target_pos - agent.pos
        distance = desired.length()
        
        if distance > 0:
            # 如果在減速範圍內，根據距離縮放速度
            if distance < slow_radius:
                speed = agent.max_speed * (distance / slow_radius)
            else:
                speed = agent.max_speed
                
            desired.scale_to_length(speed)
            steering = desired - agent.vel
            return limit_vector(steering, agent.max_force)
        return Vector2(0, 0)

    @staticmethod
    def flee(agent, target_pos, panic_radius=200.0):
        """
        Flee: 遠離目標，僅在進入 panic_radius 時觸發
        """
        desired = agent.pos - target_pos # 方向相反
        distance = desired.length()
        
        if distance < panic_radius and distance > 0:
            desired.scale_to_length(agent.max_speed)
            steering = desired - agent.vel
            return limit_vector(steering, agent.max_force)
        
        return Vector2(0, 0)