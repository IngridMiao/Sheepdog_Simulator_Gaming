"""
在 Assignment 2 的狀態機基礎上，新增三種決策架構來選擇目標草叢：

  RANDOM   ── 原版隨機選擇（baseline）
  RULE     ── 規則式決策（RuleDecision）
  UTILITY  ── 效用函數（UtilitySystem）
  ADAPTIVE ── 自適應效用（AdaptiveUtility，包裝 UtilitySystem）

飽腹感（SatiationTracker）在四種架構下都存在，
差別在於不同架構對飽腹感的使用方式：
  RANDOM   → 完全忽略
  RULE     → 觸發規則 R2 / R3 的門檻條件
  UTILITY  → 直接作為效用分項（w_satiation * s_satiation）
  ADAPTIVE → 同 UTILITY，但 w_satiation 會隨飢餓/逃跑事件動態調整

新增按鍵（由 simulation.py 傳遞）：
  T → 循環切換決策架構
"""

import pygame
import random as py_random
from entity.base_agent import BaseAgent
from utils.vector_math import Vector2
from behaviors.steering import SteeringBehaviors

# ── 決策模組 ──────────────────────────────────────────────────────────────
from decision.satiation      import SatiationTracker
from decision.utility_system import UtilitySystem
from decision.rule_decision  import RuleDecision
from decision.adaptive_utility import AdaptiveUtility
from decision.decision_logger  import DecisionLogger


class Sheep(BaseAgent):

    # ── 狀態常數 ─────────────────────────────────────────────────────
    STATE_CHOOSE   = "CHOOSE_BUSH"
    STATE_PATH     = "PATHFINDING"
    STATE_FOLLOW   = "FOLLOWING"
    STATE_EAT      = "EATING"
    STATE_FLEE     = "FLEEING"
    STATE_WIN      = "WIN"

    # ── 決策架構清單 ─────────────────────────────────────────────────
    DECISION_ARCHS = ["RANDOM", "RULE", "UTILITY", "ADAPTIVE"]

    # ── 行為參數 ─────────────────────────────────────────────────────
    EAT_DURATION   = 2.0
    FLEE_DURATION  = 1.2
    PANIC_RADIUS   = 120.0
    ARRIVE_SLOW_R  = 60.0
    BUSH_REACH_R   = 40.0

    def __init__(self, x, y):
        super().__init__(x, y, (255, 255, 255))
        self.max_speed = 110.0

        self.image = pygame.image.load("imgs/sheep.png").convert_alpha()
        self.image = pygame.transform.scale(self.image, (35, 35))
        self.trail_image = pygame.image.load("imgs/sheepprint.png").convert_alpha()
        self.trail_image = pygame.transform.scale(self.trail_image, (15, 15))

        # ── Navigation 系統（由 Simulation 注入）─────────────────────
        self.grid_map  = None
        self.astar     = None
        self.dijkstra  = None
        self.follower  = None

        # ── 狀態機 ───────────────────────────────────────────────────
        self.state        = self.STATE_CHOOSE
        self._state_timer = 0.0

        # ── 草叢管理 ─────────────────────────────────────────────────
        self.all_bushes   = []
        self.eaten_bushes = set()
        self.target_bush  = None

        # ── Pathfinding 設定 ─────────────────────────────────────────
        self.pathfind_algo = "ASTAR"
        self.heuristic     = "EUCLIDEAN"
        self.current_path  = []

        # ── 決策系統（在 setup_navigation 後才建立）──────────────────
        self.decision_arch  = "UTILITY"   # 預設架構
        self._arch_idx      = self.DECISION_ARCHS.index("UTILITY")

        self.satiation_tracker = None
        self.utility_system    = None
        self.rule_system       = None
        self.adaptive_system   = None
        self.decision_logger   = DecisionLogger()

        # ── 供 adaptive 用：追蹤「剛吃完」旗標 ──────────────────────
        self._just_ate = False

        # ── 遊戲時間（由 update_navigation 累積）────────────────────
        self._game_time = 0.0

    # ------------------------------------------------------------------ #
    #  初始化介面                                                          #
    # ------------------------------------------------------------------ #

    def setup_navigation(self, grid_map, astar, dijkstra, follower, bushes):
        self.grid_map   = grid_map
        self.astar      = astar
        self.dijkstra   = dijkstra
        self.follower   = follower
        self.all_bushes = bushes
        self.eaten_bushes.clear()
        self.target_bush  = None
        self.current_path = []
        self.state        = self.STATE_CHOOSE
        self._game_time   = 0.0

        # ── 建立決策系統 ─────────────────────────────────────────────
        self.satiation_tracker = SatiationTracker(bushes)

        self.utility_system = UtilitySystem(self.satiation_tracker)
        self.rule_system    = RuleDecision(self.satiation_tracker)
        self.adaptive_system = AdaptiveUtility(self.utility_system)

        self.decision_logger.reset()

    def reset_navigation(self):
        self.eaten_bushes.clear()
        self.target_bush  = None
        self.current_path = []
        self.state        = self.STATE_CHOOSE
        self._state_timer = 0.0
        self._game_time   = 0.0
        if self.follower:
            self.follower.set_path([])
        if self.satiation_tracker and self.all_bushes:
            self.satiation_tracker.reset(self.all_bushes)
        if self.adaptive_system:
            self.adaptive_system._hunger_timer = 0.0
        self.decision_logger.reset()

    # ------------------------------------------------------------------ #
    #  決策架構切換                                                        #
    # ------------------------------------------------------------------ #

    def cycle_decision_arch(self):
        self._arch_idx    = (self._arch_idx + 1) % len(self.DECISION_ARCHS)
        self.decision_arch = self.DECISION_ARCHS[self._arch_idx]
        print(f"[Sheep] Decision arch: {self.decision_arch}")

    # ------------------------------------------------------------------ #
    #  主更新                                                              #
    # ------------------------------------------------------------------ #

    def update_navigation(self, dt, dog_pos):
        if self.grid_map is None:
            return Vector2(0, 0)

        self._game_time += dt
        self._just_ate   = False
        force = Vector2(0, 0)

        # 自適應更新（每幀都跑，與狀態無關）
        if self.adaptive_system:
            self.adaptive_system.update(dt, just_ate=False)  # just_ate 在 EATING 完成時單獨觸發

        # 任何狀態下偵測 panic
        dog_dist = self.pos.distance_to(dog_pos)
        if (self.state not in (self.STATE_FLEE, self.STATE_WIN)
                and dog_dist < self.PANIC_RADIUS):
            self._enter_flee()

        if   self.state == self.STATE_CHOOSE: force = self._update_choose(dog_pos)
        elif self.state == self.STATE_PATH:   force = self._update_pathfinding()
        elif self.state == self.STATE_FOLLOW: force = self._update_following(dt)
        elif self.state == self.STATE_EAT:    force = self._update_eating(dt)
        elif self.state == self.STATE_FLEE:   force = self._update_fleeing(dt, dog_pos)
        elif self.state == self.STATE_WIN:    self.vel *= 0.9

        return force

    # ------------------------------------------------------------------ #
    #  狀態更新                                                            #
    # ------------------------------------------------------------------ #

    def _update_choose(self, dog_pos):
        remaining = [b for b in self.all_bushes if id(b) not in self.eaten_bushes]
        if not remaining:
            self.state = self.STATE_WIN
            return Vector2(0, 0)

        chosen = self._decide(remaining, dog_pos)
        self.target_bush = chosen
        self.state = self.STATE_PATH
        return Vector2(0, 0)

    def _decide(self, remaining, dog_pos):
        """
        根據當前決策架構從 remaining 中選出目標草叢。
        同時把決策記錄進 decision_logger。
        """
        arch = self.decision_arch

        if arch == "RANDOM":
            chosen = py_random.choice(remaining)
            rule_fired     = None
            utility_scores = None

        elif arch == "RULE":
            chosen = self.rule_system.choose(self.pos, remaining, dog_pos)
            rule_fired     = self.rule_system.last_rule
            utility_scores = None

        elif arch == "UTILITY":
            chosen = self.utility_system.choose(self.pos, remaining, dog_pos)
            rule_fired     = None
            utility_scores = [s["total"] for s in self.utility_system.last_scores]

        elif arch == "ADAPTIVE":
            # ADAPTIVE 共用同一個 UtilitySystem 實例（已被 AdaptiveUtility 調整過權重）
            chosen = self.utility_system.choose(self.pos, remaining, dog_pos)
            rule_fired     = None
            utility_scores = [s["total"] for s in self.utility_system.last_scores]

        else:
            chosen = py_random.choice(remaining)
            rule_fired     = None
            utility_scores = None

        # 記錄
        self.decision_logger.log_choice(
            t               = self._game_time,
            arch            = arch,
            chosen_bush     = chosen,
            all_bushes      = self.all_bushes,
            satiation_tracker = self.satiation_tracker,
            sheep_pos       = self.pos,
            dog_pos         = dog_pos,
            rule_fired      = rule_fired,
            utility_scores  = utility_scores,
        )
        return chosen

    def _update_pathfinding(self):
        if self.target_bush is None:
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        sc, sr = self.grid_map.cell_of(self.pos.x, self.pos.y)
        gc, gr = self.grid_map.cell_of(self.target_bush.pos.x, self.target_bush.pos.y)

        if self.grid_map.is_blocked(gc, gr):
            gc, gr = self._nearest_open_cell(gc, gr)

        if gc is None:
            self.eaten_bushes.add(id(self.target_bush))
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        if self.pathfind_algo == "ASTAR":
            path, cost = self.astar.search(sc, sr, gc, gr, heuristic=self.heuristic)
        else:
            path, cost = self.dijkstra.search(sc, sr, gc, gr)

        if not path:
            self.eaten_bushes.add(id(self.target_bush))
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        self.current_path = path
        self.follower.set_path(path)
        self.state = self.STATE_FOLLOW
        return Vector2(0, 0)

    def _nearest_open_cell(self, col, row, max_radius=3):
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
        if self.follower is None:
            self.state = self.STATE_CHOOSE
            return Vector2(0, 0)

        target = self.follower.update(self.pos.x, self.pos.y)

        if target is None or self.follower.is_finished:
            self._enter_eating()
            return Vector2(0, 0)

        if self.pos.distance_to(self.target_bush.pos) < self.BUSH_REACH_R:
            self._enter_eating()
            return Vector2(0, 0)

        return SteeringBehaviors.arrive(
            self, Vector2(target[0], target[1]),
            slow_radius=self.ARRIVE_SLOW_R
        )

    def _update_eating(self, dt):
        self._state_timer -= dt
        self.vel *= (1 - 4.0 * dt)

        if self._state_timer <= 0:
            # 通知飽腹感系統
            if self.satiation_tracker and self.target_bush:
                self.satiation_tracker.register_eaten(self.target_bush)

            # 通知 adaptive 系統「剛吃完」
            if self.adaptive_system:
                self.adaptive_system.update(dt=0, just_ate=True)

            self.eaten_bushes.add(id(self.target_bush))
            self.target_bush  = None
            self.current_path = []
            self.state        = self.STATE_CHOOSE

        return Vector2(0, 0)

    def _update_fleeing(self, dt, dog_pos):
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
        # 通知 adaptive 系統
        if self.adaptive_system:
            self.adaptive_system.on_flee()
        # 記錄 flee 事件
        # dog_pos 此時不在這裡，flee 事件由 simulation.py 呼叫 log_flee

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
    #  繪製                                                                #
    # ------------------------------------------------------------------ #

    def draw(self, screen, show_debug=True):
        super().draw(screen, show_debug)

        if not show_debug:
            return

        # 路徑視覺化
        if (self.follower and len(self.follower.path) >= 2
                and self.state in (self.STATE_FOLLOW, self.STATE_EAT,
                                   self.STATE_CHOOSE, self.STATE_PATH)):
            self.follower.draw_debug(
                screen, self.pos.x, self.pos.y,
                path_color=(255, 215, 0),
                target_color=(255, 140, 0),
                node_color=(200, 200, 50)
            )

        # 狀態文字
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

        # 目標草叢連線
        if self.target_bush and self.state in (self.STATE_FOLLOW, self.STATE_EAT):
            tx, ty = int(self.target_bush.pos.x), int(self.target_bush.pos.y)
            sx, sy = int(self.pos.x), int(self.pos.y)
            pygame.draw.line(screen, (150, 255, 150), (sx, sy), (tx, ty), 1)

        # PANIC_RADIUS 圓
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

        # 飽腹感小標籤（在目標草叢旁顯示）
        if self.target_bush and self.satiation_tracker and show_debug:
            sat = self.satiation_tracker.satiation(self.target_bush)
            count = self.satiation_tracker.eaten_count(self.target_bush)
            sfont = pygame.font.SysFont("Arial", 11)
            sat_label = sfont.render(f"sat={sat:.2f}(×{count})", True, (255, 220, 100))
            screen.blit(sat_label, (int(self.target_bush.pos.x) - 25,
                                     int(self.target_bush.pos.y) - 32))