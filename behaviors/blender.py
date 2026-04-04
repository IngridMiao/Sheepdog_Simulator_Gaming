from utils.vector_math import Vector2, limit_vector
from behaviors.steering import SteeringBehaviors
from behaviors.avoidance import AvoidanceBehaviors

class BehaviorBlender:
    @staticmethod
    def blend_sheep_behaviors(sheep, dog, obstacles, wander_data, target_bush=None):
        """
        混合綿羊行為：逃離狗狗 + 避障 + 閒置(隨機漫遊或尋找草叢)
        """
        # 1. 定義各項行為的權重 (Weights)
        W_FLEE = 1.5      # 逃命最重要
        W_AVOID = 2.5     # 撞到障礙物會卡住，權重最高
        W_IDLE = 0.3      # 沒事的時候才漫遊或找草叢，權重最低
        
        # 2. 計算各別的力
        flee_force = SteeringBehaviors.flee(sheep, dog.pos, panic_radius=200.0)
        
        # 避障力
        avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(sheep, obstacles)
        
        # 閒置力 (Seek 草叢 或 漫遊)
        if target_bush:
            idle_force = SteeringBehaviors.arrive(sheep, target_bush.pos, slow_radius=80.0)
            # 抵達草叢附近且沒被狗追時，增加一點摩擦力讓牠停下吃草
            if sheep.pos.distance_to(target_bush.pos) < 30 and flee_force.length_squared() == 0:
                sheep.vel *= 0.9
        else:
            wander_target = sheep.pos + Vector2(1, 0).rotate_rad(wander_data["angle"]) * 50
            idle_force = SteeringBehaviors.seek(sheep, wander_target)

        # 3. 加權總合
        total_force = (flee_force * W_FLEE) + \
                      (avoid_force * W_AVOID) + \
                      (idle_force * W_IDLE)
        
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