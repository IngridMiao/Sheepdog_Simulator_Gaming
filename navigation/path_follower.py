"""
navigation/path_follower.py

把 A* / Dijkstra 回傳的格子路徑 [(col,row), ...]
轉換成連續的移動目標點，供 SteeringBehaviors.arrive() 使用。

實作兩種策略：

  NEAREST_NODE（最近節點追蹤）
  ─────────────────────────────
  維護一個「當前目標節點索引」。
  每幀檢查 agent 是否已進入目標節點的接受半徑內，
  若是則切換到下一個節點。

  優點：實作簡單，行為可預期
  缺點：轉角處容易出現「先衝過頭再折返」的抖動，
        因為 agent 要先精確通過每個節點才切換

  LOOK_AHEAD（前瞻追蹤）
  ─────────────────────────────
  在路徑上往前看固定距離（look_ahead_dist），
  找到路徑上距離 agent 最近、且在前瞻距離內的目標點。
  目標點是路徑線段上的連續插值點，不侷限在節點上。

  優點：轉角平滑，不會因為「剛好沒踩到節點」而卡住，
        高速移動時也能提前轉向
  缺點：前瞻距離需要調整；若路徑有急彎，可能切角

  兩者都透過 get_target() 回傳一個 (x, y) 像素座標，
  直接傳給 SteeringBehaviors.arrive() 使用。
"""

import math


class PathFollower:
    # 預設參數
    ARRIVAL_RADIUS    = 28.0   # 多近算「抵達節點」（px），NEAREST_NODE 用
    LOOK_AHEAD_DIST   = 96.0   # 前瞻距離（px），LOOK_AHEAD 用，約 1.5 個格子

    def __init__(self, grid_map, strategy="LOOK_AHEAD"):
        """
        grid_map : GridMap 實例（用來做格子↔像素轉換）
        strategy : "NEAREST_NODE" | "LOOK_AHEAD"
        """
        self.grid     = grid_map
        self.strategy = strategy

        # 路徑狀態
        self._path        = []     # [(col, row), ...]
        self._waypoints   = []     # [(px, py), ...] 格子中心的像素座標
        self._node_index  = 0      # 當前目標節點索引（NEAREST_NODE 用）
        self.is_finished  = False  # 是否已走到終點

    # ------------------------------------------------------------------ #
    #  公開介面                                                             #
    # ------------------------------------------------------------------ #

    def set_path(self, path):
        """
        載入新路徑。
        path: [(col, row), ...] A*/Dijkstra 的輸出

        每次重新規劃路徑時呼叫，會重置所有狀態。
        """
        self._path       = path
        self._waypoints  = [self.grid.world_pos(c, r) for c, r in path]
        self._node_index = 0
        self.is_finished = len(path) == 0

    def get_target(self, agent_x, agent_y):
        """
        根據當前策略，回傳本幀的移動目標點 (px, py)。
        若路徑已完成或為空，回傳 None。
        """
        if self.is_finished or not self._waypoints:
            return None

        if self.strategy == "NEAREST_NODE":
            return self._target_nearest_node(agent_x, agent_y)
        else:
            return self._target_look_ahead(agent_x, agent_y)

    def update(self, agent_x, agent_y):
        """
        更新內部狀態（NEAREST_NODE 需要每幀呼叫以推進節點索引）。
        回傳目標點，是 get_target() 的組合版本。
        """
        target = self.get_target(agent_x, agent_y)
        return target

    @property
    def path(self):
        """目前路徑（唯讀）"""
        return self._path

    @property
    def current_node_index(self):
        return self._node_index

    # ------------------------------------------------------------------ #
    #  策略一：NEAREST_NODE                                                #
    # ------------------------------------------------------------------ #

    def _target_nearest_node(self, agent_x, agent_y):
        """
        目標 = path[_node_index] 的像素座標。
        當 agent 距離目標節點 < ARRIVAL_RADIUS 時，推進到下一個節點。
        """
        # 推進：如果已夠近，切換到下一個節點
        while self._node_index < len(self._waypoints):
            tx, ty = self._waypoints[self._node_index]
            dx = agent_x - tx
            dy = agent_y - ty
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.ARRIVAL_RADIUS:
                self._node_index += 1
            else:
                break   # 還沒到，繼續朝這個節點走

        # 檢查是否走完
        if self._node_index >= len(self._waypoints):
            self.is_finished = True
            return None

        return self._waypoints[self._node_index]

    # ------------------------------------------------------------------ #
    #  策略二：LOOK_AHEAD                                                  #
    # ------------------------------------------------------------------ #

    def _target_look_ahead(self, agent_x, agent_y):
        """
        在路徑的線段上找「前瞻目標點」：

        步驟：
          1. 找到路徑上距離 agent 最近的點（closest point on path）
          2. 從該點沿路徑方向前進 LOOK_AHEAD_DIST
          3. 回傳那個點作為目標

        這樣做的效果：
          - agent 永遠追著「前方一段距離的路徑上的點」跑
          - 轉角時會提前開始轉向，不需要先「踩到」節點
        """
        if not self._waypoints:
            return None

        # 只有一個點
        if len(self._waypoints) == 1:
            wp = self._waypoints[0]
            dx = agent_x - wp[0]
            dy = agent_y - wp[1]
            if math.sqrt(dx*dx + dy*dy) < self.ARRIVAL_RADIUS:
                self.is_finished = True
                return None
            return wp

        # ── 步驟 1：找最近線段及其上的最近點 ──────────────────────────
        best_seg   = 0          # 最近線段的起點索引
        best_t     = 0.0        # 線段參數 t ∈ [0, 1]
        best_dist2 = float('inf')

        for i in range(len(self._waypoints) - 1):
            ax, ay = self._waypoints[i]
            bx, by = self._waypoints[i + 1]

            # 線段向量
            abx = bx - ax
            aby = by - ay
            ab_len2 = abx*abx + aby*aby

            if ab_len2 < 1e-9:
                # 退化線段（兩點重疊），直接用端點
                t = 0.0
            else:
                # agent 投影到線段上
                t = ((agent_x - ax)*abx + (agent_y - ay)*aby) / ab_len2
                t = max(0.0, min(1.0, t))

            # 最近點
            cx = ax + t * abx
            cy = ay + t * aby
            d2 = (agent_x - cx)**2 + (agent_y - cy)**2

            if d2 < best_dist2:
                best_dist2 = d2
                best_seg   = i
                best_t     = t

        # ── 步驟 2：從最近點沿路徑前進 LOOK_AHEAD_DIST ────────────────
        # 從 best_seg 的 best_t 處開始，累積走過的距離
        remaining = self.LOOK_AHEAD_DIST
        seg = best_seg
        t   = best_t

        while seg < len(self._waypoints) - 1:
            ax, ay = self._waypoints[seg]
            bx, by = self._waypoints[seg + 1]

            # 這條線段從 t 到 1.0 的長度
            seg_len = math.sqrt((bx-ax)**2 + (by-ay)**2)
            seg_remaining = seg_len * (1.0 - t)

            if remaining <= seg_remaining:
                # 目標點就在這條線段上
                ratio = t + (remaining / seg_len)
                target_x = ax + ratio * (bx - ax)
                target_y = ay + ratio * (by - ay)
                return (target_x, target_y)
            else:
                # 前瞻距離超過這條線段，繼續到下一段
                remaining -= seg_remaining
                seg += 1
                t = 0.0

        # 前瞻超過路徑末端 → 以終點為目標
        last = self._waypoints[-1]

        # 判斷是否已抵達終點
        dx = agent_x - last[0]
        dy = agent_y - last[1]
        if math.sqrt(dx*dx + dy*dy) < self.ARRIVAL_RADIUS:
            self.is_finished = True
            return None

        return last

    # ------------------------------------------------------------------ #
    #  Debug 視覺化                                                        #
    # ------------------------------------------------------------------ #

    def draw_debug(self, screen, agent_x, agent_y,
                   path_color=(255, 215, 0),
                   target_color=(255, 100, 0),
                   node_color=(200, 200, 0)):
        """
        疊加視覺化資訊：
          - 金色連線：完整路徑
          - 橘色圓圈：當前目標點
          - 小圓點：路徑節點
        需要在 pygame 環境下使用。
        """
        import pygame

        if len(self._waypoints) < 2:
            return

        # 路徑連線
        pygame.draw.lines(screen, path_color, False,
                          [(int(x), int(y)) for x, y in self._waypoints], 3)

        # 節點小圓點
        for i, (wx, wy) in enumerate(self._waypoints):
            r = 5 if i == self._node_index else 3
            pygame.draw.circle(screen, node_color, (int(wx), int(wy)), r)

        # 當前目標點
        target = self.get_target(agent_x, agent_y)
        if target:
            tx, ty = target
            pygame.draw.circle(screen, target_color, (int(tx), int(ty)), 8, 2)
            # 從 agent 到目標的連線
            pygame.draw.line(screen, target_color,
                             (int(agent_x), int(agent_y)),
                             (int(tx), int(ty)), 1)
