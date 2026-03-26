from utils.vector_math import Vector2, limit_vector
from behaviors.steering import SteeringBehaviors
from behaviors.avoidance import AvoidanceBehaviors

class BehaviorBlender:
    @staticmethod
    def blend_sheep_behaviors(sheep, dog, obstacles, wander_data):
        """
        混合綿羊行為：逃離狗狗 + 避障 + 隨機漫遊
        """
        # 1. 定義各項行為的權重 (Weights)
        W_FLEE = 1.5      # 逃命最重要
        W_AVOID = 2.5     # 撞到障礙物會卡住，權重最高
        W_WANDER = 0.3    # 沒事的時候才漫遊，權重最低
        
        # 2. 計算各別的力
        flee_force = SteeringBehaviors.flee(sheep, dog.pos, panic_radius=300.0)
        
        # 避障力
        avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(sheep, obstacles)
        
        # 漫遊力 (利用 Seek 往漫遊方向拉)
        wander_target = sheep.pos + Vector2(1, 0).rotate_rad(wander_data["angle"]) * 50
        wander_force = SteeringBehaviors.seek(sheep, wander_target)

        # 3. 加權總合
        total_force = (flee_force * W_FLEE) + \
                      (avoid_force * W_AVOID) + \
                      (wander_force * W_WANDER)
        
        # 4. 限制最終輸出的力
        return limit_vector(total_force, sheep.max_force)

    @staticmethod
    def blend_dog_behaviors(dog, target_pos, obstacles):
        """
        混合牧羊犬行為：抵達目標 + 避障
        """
        W_ARRIVE = 1.0
        W_AVOID = 2.0
        
        arrive_force = SteeringBehaviors.arrive(dog, target_pos)
        avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(dog, obstacles)
        
        total_force = (arrive_force * W_ARRIVE) + (avoid_force * W_AVOID)
        
        return limit_vector(total_force, dog.max_force)