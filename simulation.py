import pygame
import random
from behaviors.avoidance import AvoidanceBehaviors
from entity.sheep import Sheep
from entity.sheepdog import SheepDog
from entity.obstacle import Obstacle
from entity.bush import Bush
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
            
            # 建立一個半封閉的羊圈 (開口朝左，且只留中間一半的空間)，目標是讓牧羊犬把羊趕進去
            Obstacle(850, 250, 200, 20, "FENCE"),  # 上方柵欄 (寬200, 高20)
            Obstacle(940, 350, 20, 220, "FENCE"),  # 右方柵欄 (寬20, 高220，剛好連接上下方柵欄)
            Obstacle(850, 450, 200, 20, "FENCE"),  # 下方柵欄 (寬200, 高20)
            Obstacle(760, 270, 20, 60, "FENCE"),   # 左上柵欄 (高60，封住上半部開口)
            Obstacle(760, 430, 20, 60, "FENCE"),   # 左下柵欄 (高60，封住下半部開口)
        ]
        
        # 3. 初始化草叢 (做為羊吃草的目標)
        self.bushes = [
            Bush(150, 600),
            Bush(550, 150),
            Bush(700, 600)
        ]
        self.target_bush = None # 儲存綿羊當前的隨機草叢目標
        self.idle_state = "WANDER" # 綿羊閒置狀態 ("WANDER" 或 "BUSH")
        self.idle_timer = 0.0      # 閒置狀態計時器
        
        # 計時與任務狀態
        self.elapsed_time = 0.0
        self.task_completed = False
        
        # 模式與偵錯狀態
        self.mode = "KINEMATIC"  # 預設模式
        self.show_debug = True   # Part 4: 視覺化偵錯開關

    def set_mode(self, mode_name):
        """切換移動模式 (Kinematic / Steering / Combined)"""
        self.mode = mode_name
        print(f"Current Mode: {self.mode}")
        self._reset_agents()

    def _reset_agents(self):
        """隨機重置所有 Agent 的位置與物理狀態"""
        self.target_bush = None # 重新開始時忘記草叢目標
        self.idle_state = "WANDER"
        self.idle_timer = 0.0
        self.elapsed_time = 0.0
        self.task_completed = False
        for agent in self.agents:
            agent.pos.x = random.randint(50, self.screen_width - 50)
            agent.pos.y = random.randint(50, self.screen_height - 50)
            agent.vel = Vector2(0, 0)
            agent.accel = Vector2(0, 0)
            agent.breadcrumb_trail.clear()
            agent.last_trail_pos = Vector2(agent.pos.x, agent.pos.y)

    def toggle_debug(self):
        """切換視覺化偵錯顯示"""
        self.show_debug = not self.show_debug

    def update(self, dt):
        """
        核心更新循環
        這裡未來會根據 self.mode 調用 behaviors/ 資料夾下的邏輯
        """
        # 更新計時器與檢查任務是否完成
        if not self.task_completed:
            self.elapsed_time += dt
            # 檢查羊是否進入柵欄內部範圍 (X: 770~930, Y: 260~440)
            if 770 < self.sheep.pos.x < 930 and 260 < self.sheep.pos.y < 440:
                self.task_completed = True

        mouse_pos = Vector2(pygame.mouse.get_pos()) # 取得滑鼠位置 (用於牧羊犬的目標)
        # A. 根據當前模式計算行為 (這裡先留白，待後續實作 behaviors 後填入)
        if self.mode == "KINEMATIC":
            # 呼叫 kinematic.py
            # 牧羊犬:seek 目前滑鼠位置
            KinematicBehaviors.seek(self.dog, mouse_pos)
            
            # 綿羊: 優先判斷是否需要躲避狗 (在半徑 100 內)，若無危險才隨機漫遊
            is_fleeing = KinematicBehaviors.flee(self.sheep, self.dog.pos, panic_radius=100.0)
            if not is_fleeing:
                KinematicBehaviors.wander(self.sheep, dt, self.wander_data)
            pass
        elif self.mode == "STEERING":
            # 呼叫 steering.py
            # 牧羊犬: arrive
            self.dog.accel += SteeringBehaviors.arrive(self.dog, mouse_pos)

            # 綿羊: flee
            sheep_force = SteeringBehaviors.flee(self.sheep, self.dog.pos, panic_radius=200.0)

            if sheep_force.length_squared() == 0:
                # 如果沒有被嚇到，尋找隨機一個草叢並靠近 (吃草)
                if self.bushes:
                    # 如果還沒有目標，就隨機挑選一個
                    if self.target_bush is None:
                        self.target_bush = random.choice(self.bushes)
                        
                    # 使用 arrive 行為，讓羊靠近草叢時會自動減速停下
                    self.sheep.accel += SteeringBehaviors.arrive(self.sheep, self.target_bush.pos, slow_radius=80.0)
                    
                    # 抵達草叢附近時額外加點摩擦力，讓牠安穩地停在旁邊吃草
                    if self.sheep.pos.distance_to(self.target_bush.pos) < 30:
                        self.sheep.vel *= 0.9
                else:
                    self.sheep.vel *= 0.95
            else:
                self.target_bush = None # 受到驚嚇，立刻放棄當前吃草目標
                self.sheep.accel += sheep_force
            pass
        elif self.mode == "COMBINED":
            # 決定羊的閒置狀態 (每隔幾秒隨機切換 Wander 或 Seek Bush)
            if self.sheep.pos.distance_to(self.dog.pos) < 300.0:
                # 被狗追時，放棄一切吃草與漫遊計畫
                self.target_bush = None
                self.idle_state = "WANDER"
            else:
                self.idle_timer -= dt
                if self.idle_timer <= 0:
                    # 每 3 到 6 秒隨機改變一次心意
                    self.idle_state = random.choice(["WANDER", "BUSH"])
                    self.idle_timer = random.uniform(3.0, 6.0) 
                    
                    if self.idle_state == "BUSH" and self.bushes:
                        self.target_bush = random.choice(self.bushes)
                    else:
                        self.target_bush = None

            # 呼叫 blender.py (Part 5)
            self.dog.accel += BehaviorBlender.blend_dog_behaviors(
                self.dog, mouse_pos, self.obstacles
            )
            self.sheep.accel += BehaviorBlender.blend_sheep_behaviors(
                self.sheep, self.dog, self.obstacles, self.wander_data, self.target_bush
            )
            # 更新漫遊狀態的角度 (自行更新避免 Kinematic 邏輯直接覆寫速度)
            current_angle = self.wander_data.get("angle", self.sheep.orientation)
            self.wander_data["angle"] = current_angle + random.uniform(-0.5, 0.5)
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
            
            # 強制處理穿模問題 (硬碰撞檢測與推擠)
            self._resolve_collisions(agent)

    def _handle_boundaries(self, agent):
        """邊界處理：限制在螢幕內，碰到邊界時只能沿著邊界滑動或往內走"""
        margin = agent.radius
        
        # X 軸邊界限制
        if agent.pos.x < margin:
            agent.pos.x = margin
            if agent.vel.x < 0: agent.vel.x = 0
        elif agent.pos.x > self.screen_width - margin:
            agent.pos.x = self.screen_width - margin
            if agent.vel.x > 0: agent.vel.x = 0
            
        # Y 軸邊界限制
        if agent.pos.y < margin:
            agent.pos.y = margin
            if agent.vel.y < 0: agent.vel.y = 0
        elif agent.pos.y > self.screen_height - margin:
            agent.pos.y = self.screen_height - margin
            if agent.vel.y > 0: agent.vel.y = 0

    def _resolve_collisions(self, agent):
        """處理硬碰撞，強制把 Agent 從障礙物中推出來並修正速度，完全防止穿模"""
        # 1. 處理代理人之間 (Agent vs Agent) 的重疊 (例如狗與羊)
        for other in self.agents:
            if agent is not other:
                dist_vec = agent.pos - other.pos
                dist = dist_vec.length()
                min_dist = agent.radius + other.radius + 5 # 加上 5 像素緩衝，避免圖片邊緣貼太緊
                if dist < min_dist and dist > 0:
                    overlap = min_dist - dist
                    normal = dist_vec.normalize()
                    
                    # 將兩者互相推開 (各分攤一半的重疊量)
                    agent.pos += normal * (overlap / 2)
                    other.pos -= normal * (overlap / 2)
                    
                    # 抵消互相靠近的法線方向速度，避免持續抖動或擠壓穿透
                    rel_vel = agent.vel - other.vel
                    vel_along_normal = rel_vel.x * normal.x + rel_vel.y * normal.y
                    if vel_along_normal < 0:
                        agent.vel.x -= normal.x * vel_along_normal * 0.5
                        agent.vel.y -= normal.y * vel_along_normal * 0.5
                        other.vel.x += normal.x * vel_along_normal * 0.5
                        other.vel.y += normal.y * vel_along_normal * 0.5

        # 2. 處理代理人與障礙物 (Agent vs Obstacle) 的重疊
        for obs in self.obstacles:
            if obs.type == "STONE":
                # 圓形對圓形的碰撞檢測 (Agent vs Stone)
                dist_vec = agent.pos - obs.pos
                dist = dist_vec.length()
                min_dist = agent.radius + obs.radius
                if dist < min_dist and dist > 0:
                    overlap = min_dist - dist
                    normal = dist_vec.normalize()
                    # 1. 瞬間將 Agent 推出障礙物
                    agent.pos += normal * overlap
                    # 2. 消除法線方向的速度 (讓 Agent 沿著石頭邊緣滑動，而不會一直抖動卡住)
                    dot_prod = agent.vel.x * normal.x + agent.vel.y * normal.y
                    if dot_prod < 0:
                        agent.vel.x -= dot_prod * normal.x
                        agent.vel.y -= dot_prod * normal.y
                        
            elif obs.type == "FENCE":
                # 圓形對矩形的碰撞檢測 (AABB)
                closest_x = max(obs.rect.left, min(agent.pos.x, obs.rect.right))
                closest_y = max(obs.rect.top, min(agent.pos.y, obs.rect.bottom))
                
                dist_vec = agent.pos - Vector2(closest_x, closest_y)
                dist = dist_vec.length()
                
                if dist < agent.radius:
                    if dist > 0:
                        overlap = agent.radius - dist
                        normal = dist_vec.normalize()
                        # 1. 瞬間將 Agent 推出柵欄
                        agent.pos += normal * overlap
                        # 2. 消除法線方向的速度 (讓 Agent 沿著柵欄平移滑走)
                        dot_prod = agent.vel.x * normal.x + agent.vel.y * normal.y
                        if dot_prod < 0:
                            agent.vel.x -= dot_prod * normal.x
                            agent.vel.y -= dot_prod * normal.y
                    else:
                        # 極端情況：Agent 中心剛好與邊界重疊，直接反轉速度退出
                        agent.vel.x *= -1
                        agent.vel.y *= -1

    def draw(self, screen):
        """繪製所有環境元素"""
        # 優先繪製草叢 (確保疊在最下層，狗跟羊經過時會蓋在上面)
        for bush in self.bushes:
            bush.draw(screen)
            
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
        time_text = font.render(f"Time: {self.elapsed_time:.1f} s", True, (255, 255, 255))
        
        screen.blit(mode_text, (20, 20))
        screen.blit(debug_text, (20, 50))
        screen.blit(time_text, (20, 80))
        
        # 繪製任務完成文字
        if self.task_completed:
            # 載入系統支援的中文字體
            win_font = pygame.font.SysFont(["microsoftjhenghei", "simhei", "pingfang"], 64, bold=True)
            win_text = win_font.render("任務完成！", True, (255, 215, 0)) # 金黃色
            shadow_text = win_font.render("任務完成！", True, (0, 0, 0))   # 黑色陰影
            
            rect = win_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
            screen.blit(shadow_text, (rect.x + 3, rect.y + 3)) # 畫陰影
            screen.blit(win_text, rect)                        # 畫本體