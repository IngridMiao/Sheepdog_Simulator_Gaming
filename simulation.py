"""
simulation.py  (Assignment 2 重構版)

統一管理所有實體與 navigation 系統，每幀協調：
  1. 牧羊犬：維持 Assignment 1 的 steering arrive（滑鼠控制）
  2. 羊：    狀態機 + pathfinding（A* / Dijkstra）+ path following
  3. 障礙物：不變
  4. 勝負判定

新增的按鍵：
  1        → Kinematic 模式（Assignment 1 相容，僅供示範）
  2        → Steering  模式（Assignment 1 相容，僅供示範）
  3        → Navigation 模式（Assignment 2 主模式）
  G        → 切換 Grid debug 視覺化
  W        → 切換 Waypoint Graph 視覺化
  C        → 切換 clearance heatmap
  P        → 切換算法（A* Euclidean → A* Clearance → Dijkstra → 循環）
  F        → 切換 path following 策略（Look-ahead ↔ Nearest-node）
  D        → 切換 debug 資訊（agent 狀態、路徑等）
  R        → 重置場景

地圖設計說明（Assignment 2 新地圖）：
  原本的地圖障礙物太少、路徑太直，不適合展示 pathfinding 差異。
  新地圖加入更多石牆段落，製造 3 條可行路線（上路、中路、下路），
  讓 Dijkstra 和 A* 在不同 heuristic 下走出可觀察到的不同路徑。

  草叢位置刻意安排在不同象限，羊必須穿越障礙才能到達每一個。
"""

import pygame
import sys
import random
from entity.sheep     import Sheep
from entity.sheepdog  import SheepDog
from entity.obstacle  import Obstacle
from entity.bush      import Bush
from utils.vector_math import Vector2
from behaviors.kinematic import KinematicBehaviors
from behaviors.steering  import SteeringBehaviors
from behaviors.avoidance import AvoidanceBehaviors
from behaviors.blender   import BehaviorBlender

# Navigation 模組
from navigation.grid_map        import GridMap
from navigation.astar           import AStar
from navigation.dijkstra        import Dijkstra
from navigation.path_follower   import PathFollower
from navigation.waypoint_graph  import WaypointGraph


class Simulation:

    # ── 算法循環清單 ─────────────────────────────────────────────────
    ALGO_CYCLE = [
        ("ASTAR",     "EUCLIDEAN",  "A* Euclidean"),
        ("ASTAR",     "CLEARANCE",  "A* Clearance"),
        ("DIJKSTRA",  None,         "Dijkstra"),
    ]

    def __init__(self, screen_width, screen_height):
        self.screen_width  = screen_width
        self.screen_height = screen_height

        # ── 地圖與障礙物 ─────────────────────────────────────────────
        self.obstacles = self._build_obstacles()

        # ── 草叢（四個象限）─────────────────────────────────────────
        self.bushes = [
            Bush(128, 160),   # 草叢 A：左上
            Bush(864, 160),   # 草叢 B：右上
            Bush(416, 416),   # 草叢 C：中央
            Bush(550, 608),   # 草叢 D：下中
        ]

        # ── Agent ────────────────────────────────────────────────────
        self.dog   = SheepDog(96, 480)
        self.sheep = Sheep(96, 480)
        self.agents = [self.dog, self.sheep]

        # ── Navigation 系統 ──────────────────────────────────────────
        self.grid_map = GridMap(screen_width, screen_height, cell_size=64)
        self.grid_map.build_from_obstacles(self.obstacles)

        self.astar_solver    = AStar(self.grid_map, clearance_weight=3.0)
        self.dijkstra_solver = Dijkstra(self.grid_map)
        self.follower        = PathFollower(self.grid_map, strategy="LOOK_AHEAD")
        self.waypoint_graph  = WaypointGraph()

        # 注入 navigation 系統給羊
        self.sheep.setup_navigation(
            self.grid_map,
            self.astar_solver,
            self.dijkstra_solver,
            self.follower,
            self.bushes
        )

        # ── 模式與算法狀態 ───────────────────────────────────────────
        self.mode       = "NAVIGATION"   # "KINEMATIC" | "STEERING" | "NAVIGATION"
        self.algo_index = 0              # ALGO_CYCLE 的索引
        self._apply_algo()               # 同步到 sheep

        # ── 視覺化開關 ───────────────────────────────────────────────
        self.show_debug       = True
        self.show_grid        = False
        self.show_clearance   = False
        self.show_waypoint    = False

        # ── 計時與勝負 ───────────────────────────────────────────────
        self.elapsed_time   = 0.0
        self.result         = None   # None | "DOG_WIN" | "SHEEP_WIN"

        # ── Assignment 1 相容用（KINEMATIC / STEERING 模式）─────────
        self.wander_data  = {"angle": 0.0}
        self.target_bush_a1 = None
        self.idle_state   = "WANDER"
        self.idle_timer   = 0.0

    # ------------------------------------------------------------------ #
    #  障礙物定義（Assignment 2 新地圖）                                   #
    # ------------------------------------------------------------------ #

    def _build_obstacles(self):
        """
        新地圖設計原則：
          - 左側石牆群：製造「上路」與「下路」的分岔
          - 中央縱向石牆：把地圖分成左右兩側，只留幾個通道
          - 右側羊圈：保留 Assignment 1 的羊圈結構
          - 所有障礙都對齊 64px 格子，方便 grid 標記
        """
        return [
            # ── 左側石牆群（製造上下分岔）──────────────────────────
            Obstacle(224, 256, 64, 64, "STONE"),   # 左上石頭
            Obstacle(256, 512, 64, 64, "STONE"),   # 左下石頭

            # ── 中央縱向石牆（只留上/中/下三個通道）────────────────
            Obstacle(480, 160, 20, 128, "FENCE"),  # 中牆上段
            Obstacle(480, 384, 20, 128, "FENCE"),  # 中牆中段
            Obstacle(480, 608, 20, 128, "FENCE"),  # 中牆下段
            # 通道：row2(128~192) 上通道, row5~6(320~448) 中通道, row9(576~640) 下通道

            # ── 右側羊圈（保留 Assignment 1 結構）───────────────────
            Obstacle(850, 250, 200, 20, "FENCE"),  # 上柵欄
            Obstacle(940, 350, 20, 220, "FENCE"),  # 右柵欄
            Obstacle(850, 450, 200, 20, "FENCE"),  # 下柵欄
            Obstacle(760, 270, 20, 60,  "FENCE"),  # 左上小柵欄
            Obstacle(760, 430, 20, 60,  "FENCE"),  # 左下小柵欄
        ]

    # ------------------------------------------------------------------ #
    #  公開控制介面                                                        #
    # ------------------------------------------------------------------ #

    def set_mode(self, mode_name):
        self.mode = mode_name
        print(f"[Simulation] Mode: {self.mode}")
        self._reset()

    def toggle_debug(self):
        self.show_debug = not self.show_debug

    def toggle_grid(self):
        self.show_grid = not self.show_grid
        self.show_clearance = False   # 兩個不同時開

    def toggle_clearance(self):
        self.show_clearance = not self.show_clearance
        self.show_grid = self.show_clearance  # clearance 同時開 grid

    def toggle_waypoint(self):
        self.show_waypoint = not self.show_waypoint

    def cycle_algo(self):
        """循環切換 pathfinding 算法，立刻重新規劃"""
        self.algo_index = (self.algo_index + 1) % len(self.ALGO_CYCLE)
        self._apply_algo()
        # 強制羊重新規劃（回到 CHOOSE_BUSH）
        self.sheep.current_path = []
        self.sheep.state = Sheep.STATE_CHOOSE
        print(f"[Simulation] Algo: {self.ALGO_CYCLE[self.algo_index][2]}")

    def cycle_follower(self):
        """切換 path following 策略"""
        cur = self.follower.strategy
        self.follower.strategy = "NEAREST_NODE" if cur == "LOOK_AHEAD" else "LOOK_AHEAD"
        self.sheep.current_path = []
        self.sheep.state = Sheep.STATE_CHOOSE
        print(f"[Simulation] Follower: {self.follower.strategy}")

    def _apply_algo(self):
        """把 algo_index 同步到 sheep"""
        algo, heuristic, _ = self.ALGO_CYCLE[self.algo_index]
        self.sheep.pathfind_algo = algo
        self.sheep.heuristic     = heuristic or "EUCLIDEAN"

    def _reset(self):
        """重置所有 agent 與遊戲狀態"""
        if not hasattr(self, 'sheep'):
            return
        self.elapsed_time  = 0.0
        self.result        = None
        self.wander_data   = {"angle": 0.0}
        self.target_bush_a1 = None
        self.idle_state    = "WANDER"
        self.idle_timer    = 0.0

        # 重置 agent 位置
        self.dog.pos   = Vector2(96, 480)
        self.dog.vel   = Vector2(0, 0)
        self.dog.accel = Vector2(0, 0)
        self.dog.breadcrumb_trail.clear()
        self.dog.last_trail_pos = Vector2(96, 480)

        self.sheep.pos   = Vector2(96, 480)
        self.sheep.vel   = Vector2(0, 0)
        self.sheep.accel = Vector2(0, 0)
        self.sheep.breadcrumb_trail.clear()
        self.sheep.last_trail_pos = Vector2(96, 480)

        # 重置草叢（重新建立）
        self.bushes = [
            Bush(128, 160),
            Bush(864, 160),
            Bush(416, 416),
            Bush(550, 608),
        ]

        # 重置 navigation
        self.sheep.setup_navigation(
            self.grid_map,
            self.astar_solver,
            self.dijkstra_solver,
            self.follower,
            self.bushes
        )
        self._apply_algo()

    # ------------------------------------------------------------------ #
    #  主更新                                                              #
    # ------------------------------------------------------------------ #

    def update(self, dt):
        # 遊戲結束後不更新邏輯
        if self.result is not None:
            return

        self.elapsed_time += dt
        mouse_pos = Vector2(*pygame.mouse.get_pos())

        # ── 牧羊犬（所有模式都用 steering arrive 追滑鼠）────────────
        if self.mode == "KINEMATIC":
            KinematicBehaviors.seek(self.dog, mouse_pos)
        else:
            self.dog.accel += SteeringBehaviors.arrive(self.dog, mouse_pos)

        # ── 羊的行為 ─────────────────────────────────────────────────
        if self.mode == "NAVIGATION":
            # Assignment 2 主模式：狀態機 + pathfinding
            nav_force = self.sheep.update_navigation(dt, self.dog.pos)
            self.sheep.accel += nav_force

        elif self.mode == "KINEMATIC":
            # Assignment 1 相容
            is_fleeing = KinematicBehaviors.flee(
                self.sheep, self.dog.pos, panic_radius=100.0)
            if not is_fleeing:
                KinematicBehaviors.wander(self.sheep, dt, self.wander_data)

        elif self.mode == "STEERING":
            # Assignment 1 相容
            sheep_force = SteeringBehaviors.flee(
                self.sheep, self.dog.pos, panic_radius=200.0)
            if sheep_force.length_squared() == 0:
                if self.bushes:
                    if self.target_bush_a1 is None:
                        self.target_bush_a1 = random.choice(self.bushes)
                    self.sheep.accel += SteeringBehaviors.arrive(
                        self.sheep, self.target_bush_a1.pos, slow_radius=80.0)
                    if self.sheep.pos.distance_to(self.target_bush_a1.pos) < 30:
                        self.sheep.vel *= 0.9
                else:
                    self.sheep.vel *= 0.95
            else:
                self.target_bush_a1 = None
                self.sheep.accel += sheep_force

        # ── 避障（非 NAVIGATION 模式，NAVIGATION 由 pathfinding 處理）
        if self.mode != "NAVIGATION":
            for agent in self.agents:
                avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(
                    agent, self.obstacles)
                agent.accel += avoid_force * 2.0

        # ── 狗的避障（NAVIGATION 模式也需要）───────────────────────
        if self.mode == "NAVIGATION":
            avoid_force, _ = AvoidanceBehaviors.obstacle_avoidance(
                self.dog, self.obstacles)
            self.dog.accel += avoid_force * 2.0

        # ── 物理更新 ─────────────────────────────────────────────────
        for agent in self.agents:
            agent.update(dt)
            self._handle_boundaries(agent)
            self._resolve_collisions(agent)

        # ── 勝負判定 ─────────────────────────────────────────────────
        self._check_result()

    # ------------------------------------------------------------------ #
    #  勝負判定                                                            #
    # ------------------------------------------------------------------ #

    def _check_result(self):
        # 羊獲勝：吃完所有草叢
        if (self.mode == "NAVIGATION"
                and self.sheep.state == Sheep.STATE_WIN):
            self.result = "SHEEP_WIN"
            return

        # 狗獲勝：羊進入羊圈範圍（X:770~930, Y:260~440）
        sx, sy = self.sheep.pos.x, self.sheep.pos.y
        if 770 < sx < 930 and 260 < sy < 440:
            self.result = "DOG_WIN"

    # ------------------------------------------------------------------ #
    #  繪製                                                                #
    # ------------------------------------------------------------------ #

    def draw(self, screen):
        # ── Grid / Clearance debug（最底層）─────────────────────────
        if self.show_grid:
            self.grid_map.draw_debug(screen, show_clearance=self.show_clearance)

        # ── Waypoint Graph ───────────────────────────────────────────
        if self.show_waypoint:
            self.waypoint_graph.draw_graph(screen)

        # ── 草叢 ─────────────────────────────────────────────────────
        for bush in self.bushes:
            if id(bush) not in self.sheep.eaten_bushes:
                bush.draw(screen)

        # ── 障礙物 ───────────────────────────────────────────────────
        for obs in self.obstacles:
            obs.draw(screen)

        # ── Agent ────────────────────────────────────────────────────
        for agent in self.agents:
            agent.draw(screen, self.show_debug)

        # ── UI ───────────────────────────────────────────────────────
        self._draw_ui(screen)

        # ── 結果畫面 ─────────────────────────────────────────────────
        if self.result:
            self._draw_result(screen)

    # ------------------------------------------------------------------ #
    #  UI                                                                 #
    # ------------------------------------------------------------------ #

    def _draw_ui(self, screen):
        font  = pygame.font.SysFont("Arial", 20)
        small = pygame.font.SysFont("Arial", 16)

        _, _, algo_label = self.ALGO_CYCLE[self.algo_index]

        lines = [
            (f"Mode: {self.mode}  (1=Kinematic 2=Steering 3=Navigation)", (255,255,255)),
            (f"Algo: {algo_label}  Follower: {self.follower.strategy}  (P=cycle  F=toggle)", (200,230,255)),
            (f"Time: {self.elapsed_time:.1f}s   "
             f"Bushes eaten: {self.sheep.eaten_count}/{len(self.bushes)}", (200,255,200)),
            (f"Debug(D)  Grid(G)  Clearance(C)  Waypoint(W)  Reset(R)", (180,180,180)),
        ]

        for i, (text, color) in enumerate(lines):
            surf = (font if i < 3 else small).render(text, True, color)
            screen.blit(surf, (12, 12 + i * 24))

        # NAVIGATION 模式下顯示羊的狀態
        if self.mode == "NAVIGATION" and self.show_debug:
            state_surf = font.render(
                f"Sheep: {self.sheep.state}", True,
                (255, 220, 80) if self.sheep.state == Sheep.STATE_FLEE
                else (100, 255, 100))
            screen.blit(state_surf, (12, 108))

    def _draw_result(self, screen):
        try:
            win_font = pygame.font.SysFont(
                ["microsoftjhenghei", "simhei", "pingfang"], 72, bold=True)
        except Exception:
            win_font = pygame.font.SysFont("Arial", 72, bold=True)

        if self.result == "DOG_WIN":
            msg   = "任務完成！羊進圈了"
            color = (255, 215, 0)
        else:
            msg   = "羊吃完了！牧羊犬失敗"
            color = (100, 220, 100)

        shadow = win_font.render(msg, True, (0, 0, 0))
        text   = win_font.render(msg, True, color)
        rect   = text.get_rect(center=(self.screen_width//2, self.screen_height//2))
        screen.blit(shadow, (rect.x+3, rect.y+3))
        screen.blit(text,   rect)

        sub_font = pygame.font.SysFont("Arial", 28)
        sub = sub_font.render("Press R to restart", True, (200, 200, 200))
        screen.blit(sub, sub.get_rect(center=(self.screen_width//2,
                                               self.screen_height//2 + 80)))

    # ------------------------------------------------------------------ #
    #  物理輔助                                                            #
    # ------------------------------------------------------------------ #

    def _handle_boundaries(self, agent):
        margin = agent.radius
        if agent.pos.x < margin:
            agent.pos.x = margin
            if agent.vel.x < 0: agent.vel.x = 0
        elif agent.pos.x > self.screen_width - margin:
            agent.pos.x = self.screen_width - margin
            if agent.vel.x > 0: agent.vel.x = 0
        if agent.pos.y < margin:
            agent.pos.y = margin
            if agent.vel.y < 0: agent.vel.y = 0
        elif agent.pos.y > self.screen_height - margin:
            agent.pos.y = self.screen_height - margin
            if agent.vel.y > 0: agent.vel.y = 0

    def _resolve_collisions(self, agent):
        # Agent vs Agent
        for other in self.agents:
            if agent is not other:
                dist_vec = agent.pos - other.pos
                dist     = dist_vec.length()
                min_dist = agent.radius + other.radius + 5
                if dist < min_dist and dist > 0:
                    overlap = min_dist - dist
                    normal  = dist_vec.normalize()
                    agent.pos += normal * (overlap / 2)
                    other.pos -= normal * (overlap / 2)
                    rel_vel = agent.vel - other.vel
                    vn = rel_vel.x*normal.x + rel_vel.y*normal.y
                    if vn < 0:
                        agent.vel.x -= normal.x * vn * 0.5
                        agent.vel.y -= normal.y * vn * 0.5
                        other.vel.x += normal.x * vn * 0.5
                        other.vel.y += normal.y * vn * 0.5

        # Agent vs Obstacle
        for obs in self.obstacles:
            if obs.type == "STONE":
                dist_vec = agent.pos - obs.pos
                dist     = dist_vec.length()
                min_dist = agent.radius + obs.radius
                if dist < min_dist and dist > 0:
                    overlap = min_dist - dist
                    normal  = dist_vec.normalize()
                    agent.pos += normal * overlap
                    dp = agent.vel.x*normal.x + agent.vel.y*normal.y
                    if dp < 0:
                        agent.vel.x -= dp * normal.x
                        agent.vel.y -= dp * normal.y
            elif obs.type == "FENCE":
                cx = max(obs.rect.left,  min(agent.pos.x, obs.rect.right))
                cy = max(obs.rect.top,   min(agent.pos.y, obs.rect.bottom))
                dist_vec = agent.pos - Vector2(cx, cy)
                dist     = dist_vec.length()
                if dist < agent.radius:
                    if dist > 0:
                        overlap = agent.radius - dist
                        normal  = dist_vec.normalize()
                        agent.pos += normal * overlap
                        dp = agent.vel.x*normal.x + agent.vel.y*normal.y
                        if dp < 0:
                            agent.vel.x -= dp * normal.x
                            agent.vel.y -= dp * normal.y
                    else:
                        agent.vel.x *= -1
                        agent.vel.y *= -1