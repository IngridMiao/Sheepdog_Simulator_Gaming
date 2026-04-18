"""
entity/sheep.py  (Assignment 2 修改版)

新增狀態機，驅動羊的 pathfinding 行為：

  CHOOSE_BUSH  → 從未吃過的草叢中 random 選一個，觸發路徑規劃
  PATHFINDING  → 呼叫 A* 計算路徑（同幀完成，不需等待）
  FOLLOWING    → 用 PathFollower + SteeringBehaviors.arrive 沿路徑移動
  EATING       → 停在草叢旁 eat_duration 秒，計時完成後標記草叢為已吃
  FLEEING      → 短暫 flee 離開狗，flee_duration 秒後回到 CHOOSE_BUSH
  WIN          → 吃完所有草叢，羊獲勝

與 Assignment 1 的差異：
  - 移除直接的 flee/wander 力（那些留給 simulation.py 的 STEERING 模式）
  - 加入 nav_system 參考（GridMap + AStar + PathFollower）
  - 每次 flee 結束後重新選草叢並重新規劃，不沿用舊路徑
  - 保留 draw() 介面不變，額外增加狀態文字 debug 顯示
"""

import pygame
import random
from entity.base_agent import BaseAgent
from utils.vector_math import Vector2
from behaviors.steering import SteeringBehaviors


class Sheep(BaseAgent):

    # 狀態常數
    STATE_CHOOSE   = "CHOOSE_BUSH"
    STATE_PATH     = "PATHFINDING"
    STATE_FOLLOW   = "FOLLOWING"
    STATE_EAT      = "EATING"
    STATE_FLEE     = "FLEEING"
    STATE_WIN      = "WIN"

    # 行為參數
    EAT_DURATION   = 2.0    # 秒：在草叢旁停留多久才算吃完
    FLEE_DURATION  = 1.2    # 秒：flee 持續時間
    PANIC_RADIUS   = 120.0  # px：狗進入此範圍才觸發 flee
    ARRIVE_SLOW_R  = 60.0   # px：SteeringBehaviors.arrive 的減速半徑
    BUSH_REACH_R   = 40.0   # px：距草叢多近算「抵達」

    def __init__(self, x, y):
        super().__init__(x, y, (255, 255, 255))
        self.max_speed = 110.0

        # 圖片
        self.image = pygame.image.load("imgs/sheep.png").convert_alpha()
        self.image = pygame.transform.scale(self.image, (35, 35))
        self.trail_image = pygame.image.load("imgs/sheepprint.png").convert_alpha()
        self.trail_image = pygame.transform.scale(self.trail_image, (15, 15))

        # ── Navigation 系統（由 Simulation 注入）────────────────────
        # 在 simulation.py 建立後呼叫 sheep.setup_navigation(grid, astar, follower)
        self.grid_map   = None
        self.astar      = None
        self.follower   = None   # PathFollower 實例

        # ── 狀態機 ───────────────────────────────────────────────────
        self.state        = self.STATE_CHOOSE
        self._state_timer = 0.0   # 通用計時器（EATING / FLEEING 用）

        # ── 草叢管理 ─────────────────────────────────────────────────
        self.all_bushes      = []   # 所有草叢（Bush 物件列表，由 Simulation 設定）
        self.eaten_bushes    = set()  # 已吃過的 Bush id（用 id() 區分）
        self.target_bush     = None   # 當前目標草叢

        # ── 算法選擇（可在 Simulation 切換，預設 A* Euclidean）────────
        self.pathfind_algo  = "ASTAR"       # "ASTAR" | "DIJKSTRA"
        self.heuristic      = "EUCLIDEAN"   # "EUCLIDEAN" | "CLEARANCE"

        # ── 當前路徑（供視覺化用）───────────────────────────────────
        self.current_path   = []

    # ------------------------------------------------------------------ #
    #  初始化介面（由 Simulation 呼叫）                                     #
    # ------------------------------------------------------------------ #

    def setup_navigation(self, grid_map, astar, dijkstra, follower, bushes):
        """
        注入 navigation 系統。在 Simulation.__init__ 建立完所有物件後呼叫。

        grid_map  : GridMap
        astar     : AStar
        dijkstra  : Dijkstra
        follower  : PathFollower
        bushes    : [Bush, ...] 所有草叢列表
        """
        self.grid_map  = grid_map
        self.astar     = astar
        self.dijkstra  = dijkstra
        self.follower  = follower
        self.all_bushes = bushes
        self.eaten_bushes.clear()
        self.target_bush = None
        self.state = self.STATE_CHOOSE

    def reset_navigation(self):
        """重置草叢記憶與狀態（切換 Demo 模式時用）"""
        self.eaten_bushes.clear()
        self.target_bush  = None
        self.current_path = []
        self.state        = self.STATE_CHOOSE
        self._state_timer = 0.0
        if self.follower:
            self.follower.set_path([])

    # ------------------------------------------------------------------ #
    #  主更新（每幀由 Simulation 呼叫）                                     #
    # ------------------------------------------------------------------ #

    def update_navigation(self, dt, dog_pos):
        """
        更新狀態機與 steering force。
        回傳 steering force (Vector2)，由 Simulation 加到 self.accel。

        dog_pos : Vector2，牧羊犬當前位置
        """
        if self.grid_map is None:
            return Vector2(0, 0)

        force = Vector2(0, 0)

        # ── 任何狀態下：偵測狗是否進入 panic radius ────────────────
        dog_dist = self.pos.distance_to(dog_pos)
        if (self.state not in (self.STATE_FLEE, self.STATE_WIN)
                and dog_dist < self.PANIC_RADIUS):
            self._enter_flee()

        # ── 狀態分派 ─────────────────────────────────────────────
        if self.state == self.STATE_CHOOSE:
            force = self._update_choose()

        elif self.state == self.STATE_PATH:
            force = self._update_pathfinding()

        elif self.state == self.STATE_FOLLOW:
            force = self._update_following(dt)

        elif self.state == self.STATE_EAT:
            force = self._update_eating(dt)

        elif self.state == self.STATE_FLEE:
            force = self._update_fleeing(dt, dog_pos)

        elif self.state == self.STATE_WIN:
            # 原地停止
            self.vel *= 0.9

        return force

    # ------------------------------------------------------------------ #
    #  各狀態的更新邏輯                                                     #
    # ------------------------------------------------------------------ #

    def _update_choose(self):
        """選擇下一個未吃過的草叢，立刻進入 PATHFINDING"""
        remaining = [b for b in self.all_bushes if id(b) not in self.eaten_bushes]

        if not remaining:
            self.state = self.STATE_WIN
            return Vector2(0, 0)

        self.target_bush = random.choice(remaining)
        self.state = self.STATE_PATH
        return Vector2(0, 0)  # 路徑規劃下一幀執行（避免同幀過重）

    def _update_pathfinding(self):
        """
        呼叫 A* / Dijkstra 規劃路徑，並載入 PathFollower。
        規劃完成後立刻切換到 FOLLOWING。

        
        若目標草叢所在格子剛好是 blocked（草叢座標與障礙重疊），
        會用 BFS 向外找最近的可通行格作為替代終點，
        而不是直接放棄這個草叢。
        """
        if self.target_bush is None:
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        # 起點：羊當前位置
        sc, sr = self.grid_map.cell_of(self.pos.x, self.pos.y)
        # 終點：目標草叢位置
        gc, gr = self.grid_map.cell_of(
            self.target_bush.pos.x, self.target_bush.pos.y
        )

        # ── 若終點格是 blocked，BFS 找最近的可通行鄰格 ──────────────
        if self.grid_map.is_blocked(gc, gr):
            gc, gr = self._nearest_open_cell(gc, gr)
 
        # 找不到任何可通行格（極端情況：草叢被完全封死）
        if gc is None:
            self.eaten_bushes.add(id(self.target_bush))
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        # 執行 pathfinding
        if self.pathfind_algo == "ASTAR":
            path, cost = self.astar.search(sc, sr, gc, gr,
                                           heuristic=self.heuristic)
        else:
            path, cost = self.dijkstra.search(sc, sr, gc, gr)

        if not path:
            # 找不到路徑（目標被封死）→ 跳過這個草叢
            self.eaten_bushes.add(id(self.target_bush))
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        self.current_path = path
        self.follower.set_path(path)
        self.state = self.STATE_FOLLOW
        return Vector2(0, 0)
    
    def _nearest_open_cell(self, col, row, max_radius=3):
        """
        BFS 從 (col, row) 向外擴展，找到最近的可通行格。
        max_radius：最多往外找幾格（預設 3，約 192px）。
        找不到則回傳 (None, None)。
        """
        from collections import deque
        visited = set()
        queue = deque([(col, row, 0)])
        visited.add((col, row))
 
        while queue:
            c, r, dist = queue.popleft()
            if dist > max_radius:
                break
            if not self.grid_map.is_blocked(c, r):
                return c, r
            for dc, dr in [(0,-1),(0,1),(-1,0),(1,0)]:
                nc, nr = c+dc, r+dr
                if (nc, nr) not in visited and self.grid_map.is_valid(nc, nr):
                    visited.add((nc, nr))
                    queue.append((nc, nr, dist+1))
 
        return None, None

    def _update_following(self, dt):
        """
        用 PathFollower 取得本幀目標點，
        用 SteeringBehaviors.arrive 計算 steering force。
        """
        if self.follower is None:
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        target = self.follower.update(self.pos.x, self.pos.y)

        # PathFollower 說已完成，或已夠接近草叢
        if target is None or self.follower.is_finished:
            self._enter_eating()
            return Vector2(0, 0)

        # 額外檢查：已夠接近草叢本體（不只是路徑終點格）
        if self.pos.distance_to(self.target_bush.pos) < self.BUSH_REACH_R:
            self._enter_eating()
            return Vector2(0, 0)

        return SteeringBehaviors.arrive(
            self, Vector2(target[0], target[1]),
            slow_radius=self.ARRIVE_SLOW_R
        )

    def _update_eating(self, dt):
        """停在草叢旁計時，時間到後標記草叢已吃並選下一個"""
        self._state_timer -= dt
        # 吃草時緩慢減速停下
        self.vel *= (1 - 4.0 * dt)

        if self._state_timer <= 0:
            # 吃完：標記此草叢
            self.eaten_bushes.add(id(self.target_bush))
            self.target_bush = None
            self.current_path = []
            self.state = self.STATE_CHOOSE

        return Vector2(0, 0)

    def _update_fleeing(self, dt, dog_pos):
        """
        短暫全速逃離狗，flee_duration 秒後切換回 CHOOSE_BUSH 並重新規劃。
        """
        self._state_timer -= dt

        desired = self.pos - dog_pos
        if desired.length_squared() > 0:
            desired.scale_to_length(self.max_speed)
            force = desired - self.vel
            if force.length() > self.max_force:
                force.scale_to_length(self.max_force)
        else:
            force = Vector2(0, 0)

        if self._state_timer <= 0:
            # flee 結束 → 重新選草叢（可能換一個，也可能同一個）
            self.current_path = []
            if self.follower:
                self.follower.set_path([])
            self.state = self.STATE_CHOOSE

        return force

    # ------------------------------------------------------------------ #
    #  狀態切換輔助                                                        #
    # ------------------------------------------------------------------ #

    def _enter_flee(self):
        self.state        = self.STATE_FLEE
        self._state_timer = self.FLEE_DURATION
        self.current_path = []
        if self.follower:
            self.follower.set_path([])

    def _enter_eating(self):
        self.state        = self.STATE_EAT
        self._state_timer = self.EAT_DURATION
        self.vel          = Vector2(0, 0)

    # ------------------------------------------------------------------ #
    #  屬性查詢                                                            #
    # ------------------------------------------------------------------ #

    @property
    def eaten_count(self):
        return len(self.eaten_bushes)

    @property
    def remaining_count(self):
        return len(self.all_bushes) - len(self.eaten_bushes)

    # ------------------------------------------------------------------ #
    #  繪製（保留 BaseAgent 的 draw，額外疊加狀態資訊）                      #
    # ------------------------------------------------------------------ #

    def draw(self, screen, show_debug=True):
        # 呼叫父類別的軌跡 + 本體繪製
        super().draw(screen, show_debug)

        if not show_debug:
            return

        # 路徑視覺化（金色連線）—— 只在 NAVIGATION 模式下顯示
        # STEERING / KINEMATIC 模式下 follower 可能殘留舊路徑，不應顯示
        if (self.follower and len(self.follower.path) >= 2
                and self.state in (self.STATE_FOLLOW, self.STATE_EAT,
                                   self.STATE_CHOOSE, self.STATE_PATH)):
            self.follower.draw_debug(
                screen, self.pos.x, self.pos.y,
                path_color=(255, 215, 0),
                target_color=(255, 140, 0),
                node_color=(200, 200, 50)
            )

        # 狀態文字（只在 NAVIGATION 狀態機的狀態下顯示）
        nav_states = {self.STATE_CHOOSE, self.STATE_PATH, self.STATE_FOLLOW,
                      self.STATE_EAT, self.STATE_FLEE, self.STATE_WIN}
        if self.state in nav_states:
            font = pygame.font.SysFont("Arial", 13)
            state_colors = {
                self.STATE_CHOOSE : (200, 200, 200),
                self.STATE_PATH   : (100, 200, 255),
                self.STATE_FOLLOW : (100, 255, 100),
                self.STATE_EAT    : (255, 200, 50),
                self.STATE_FLEE   : (255, 80,  80),
                self.STATE_WIN    : (255, 215, 0),
            }
            color = state_colors.get(self.state, (255, 255, 255))
            label = font.render(self.state, True, color)
            screen.blit(label, (int(self.pos.x) - 30, int(self.pos.y) - 35))

        # 目標草叢連線（虛線用短線段模擬）
        if self.target_bush and self.state in (self.STATE_FOLLOW, self.STATE_EAT):
            tx, ty = int(self.target_bush.pos.x), int(self.target_bush.pos.y)
            sx, sy = int(self.pos.x), int(self.pos.y)
            pygame.draw.line(screen, (150, 255, 150), (sx, sy), (tx, ty), 1)

        # PANIC_RADIUS 圓（淡紅色）
        pygame.draw.circle(screen, (255, 80, 80),
                           (int(self.pos.x), int(self.pos.y)),
                           int(self.PANIC_RADIUS), 1)

        # 吃草進度條
        if self.state == self.STATE_EAT:
            progress = 1.0 - (self._state_timer / self.EAT_DURATION)
            bar_w = 40
            pygame.draw.rect(screen, (80, 80, 80),
                             (int(self.pos.x) - bar_w//2,
                              int(self.pos.y) - 45, bar_w, 5))
            pygame.draw.rect(screen, (100, 220, 100),
                             (int(self.pos.x) - bar_w//2,
                              int(self.pos.y) - 45,
                              int(bar_w * progress), 5))