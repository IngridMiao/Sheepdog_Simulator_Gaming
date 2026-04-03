import pygame
from behaviors.avoidance import AvoidanceBehaviors
from entity.sheep import Sheep
from entity.sheepdog import SheepDog
from entity.obstacle import Obstacle
from utils.vector_math import Vector2
from behaviors.kinematic import KinematicBehaviors
from behaviors.steering import SteeringBehaviors
from behaviors.blender import BehaviorBlender


class Simulation:
    def __init__(self, screen_width, screen_height):
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # 1. 初始化實體 (Part 1 & 3: 至少兩個 Agent)
        self.dog = SheepDog(screen_width // 4, screen_height // 2)
        self.sheep = Sheep(screen_width // 2, screen_height // 2)
        self.agents = [self.dog, self.sheep]
        self.wander_data = {"angle": 0.0} # 儲存羊的隨機狀態
        
        # 2. 初始化障礙物 (Part 4: 至少三個障礙物，包含石頭與柵欄)
        self.obstacles = [
            Obstacle(300, 200, 60, 60, "STONE"),   # 石頭1
            Obstacle(400, 600, 80, 80, "STONE"),   # 石頭2
            
            # 建立一個 U 字型的羊圈 (開口朝左)，目標是讓牧羊犬把羊趕進去
            Obstacle(850, 250, 200, 20, "FENCE"),  # 上方柵欄 (寬200, 高20)
            Obstacle(940, 350, 20, 220, "FENCE"),  # 右方柵欄 (寬20, 高220，剛好連接上下方柵欄)
            Obstacle(850, 450, 200, 20, "FENCE"),  # 下方柵欄 (寬200, 高20)
        ]
        
        # 模式與偵錯狀態
        self.mode = "KINEMATIC"  # 預設模式
        self.show_debug = True   # Part 4: 視覺化偵錯開關

    def set_mode(self, mode_name):
        """切換移動模式 (Kinematic / Steering / Combined)"""
        self.mode = mode_name
        print(f"Current Mode: {self.mode}")

    def toggle_debug(self):
        """切換視覺化偵錯顯示"""
        self.show_debug = not self.show_debug

    def update(self, dt):
        """
        核心更新循環
        這裡未來會根據 self.mode 調用 behaviors/ 資料夾下的邏輯
        """
        mouse_pos = Vector2(pygame.mouse.get_pos()) # 取得滑鼠位置 (用於牧羊犬的目標)
        # A. 根據當前模式計算行為 (這裡先留白，待後續實作 behaviors 後填入)
        if self.mode == "KINEMATIC":
            # 呼叫 kinematic.py
            # 牧羊犬:seek 目前滑鼠位置
            KinematicBehaviors.seek(self.dog, mouse_pos)
            # 綿羊:wander
            KinematicBehaviors.wander(self.sheep, dt, self.wander_data)
            pass
        elif self.mode == "STEERING":
            # 呼叫 steering.py
            # 牧羊犬: arrive
            self.dog.accel += SteeringBehaviors.arrive(self.dog, mouse_pos)

            # 綿羊: flee
            sheep_force = SteeringBehaviors.flee(self.sheep, self.dog.pos, panic_radius=250.0)

            # 如果沒有被嚇到，羊就原地不動或緩慢摩擦力減速
            if sheep_force.length_squared() == 0:
                self.sheep.vel *= 0.95 # 模擬摩擦力
            else:
                self.sheep.accel += sheep_force
            pass
        elif self.mode == "COMBINED":
            # 呼叫 blender.py (Part 5)
            self.dog.accel += BehaviorBlender.blend_dog_behaviors(
                self.dog, mouse_pos, self.obstacles
            )
            self.sheep.accel += BehaviorBlender.blend_sheep_behaviors(
                self.sheep, self.dog, self.obstacles, self.wander_data
            )
            # 更新漫遊狀態的角度
            KinematicBehaviors.wander(self.sheep, dt, self.wander_data)
            pass

        # B. 邊界檢查 (防止 Agent 跑出視窗)
        for agent in self.agents:
            # 如果不是 COMBINED 模式，加上避障力
            if self.mode != "COMBINED":
                avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(agent, self.obstacles)
                agent.accel += avoid_force * 2.0 # 確保在普通 Steering 模式下也不會撞牆
            
            # 執行物理更新 (s = v * dt, v = a * dt)
            agent.update(dt) 
            
            # 最後進行邊界檢查，確保不跑出螢幕
            self._handle_boundaries(agent) 

    def _handle_boundaries(self, agent):
        """簡單的邊界處理：若超出螢幕則從另一邊出現或反彈"""
        if agent.pos.x < 0: agent.pos.x = self.screen_width
        elif agent.pos.x > self.screen_width: agent.pos.x = 0
        if agent.pos.y < 0: agent.pos.y = self.screen_height
        elif agent.pos.y > self.screen_height: agent.pos.y = 0

    def draw(self, screen):
        """繪製所有環境元素"""
        # 繪製障礙物
        for obs in self.obstacles:
            obs.draw(screen)
            
        # 繪製 Agent
        for agent in self.agents:
            agent.draw(screen, self.show_debug)
        
        # 繪製 UI 資訊 (當前模式)
        self._draw_ui(screen)

    def _draw_ui(self, screen):
        font = pygame.font.SysFont("Arial", 24)
        mode_text = font.render(f"Mode: {self.mode} (Press 1, 2, 3 to switch)", True, (255, 255, 255))
        debug_text = font.render(f"Debug: {'ON' if self.show_debug else 'OFF'} (Press D)", True, (255, 255, 255))
        screen.blit(mode_text, (20, 20))
        screen.blit(debug_text, (20, 50))