"""
navigation/grid_map.py

把場景切成 64x64 px 的格子，提供：
  1. 障礙標記（is_blocked）
  2. 4 向 neighbors 查詢（給 Dijkstra / A* 用）
  3. Clearance 預計算（每格到最近障礙的格子距離，給 clearance heuristic 用）
  4. Debug 視覺化（可疊加在 Pygame 畫面上）

座標系說明：
  - (col, row) = 格子座標，col 對應 x 軸，row 對應 y 軸
  - world_pos(col, row) 回傳該格中心的像素座標 (x, y)
  - cell_of(x, y) 把像素座標轉回格子座標
"""

import pygame
import math
from collections import deque


class GridMap:
    def __init__(self, screen_width, screen_height, cell_size=64):
        self.cell_size = cell_size
        self.cols = screen_width  // cell_size   # 1024 // 64 = 16
        self.rows = screen_height // cell_size   # 768  // 64 = 12

        # blocked[row][col] = True 代表該格不可通行
        self.blocked = [[False] * self.cols for _ in range(self.rows)]

        # clearance[row][col] = 到最近障礙格的 BFS 距離（格子數）
        # 障礙格本身為 0，開放格為 1 以上
        self.clearance = [[0] * self.cols for _ in range(self.rows)]

        # 視覺化用：儲存每格的顏色 overlay（可選）
        self._debug_surface = None

    # ------------------------------------------------------------------ #
    #  座標轉換                                                             #
    # ------------------------------------------------------------------ #

    def cell_of(self, x, y):
        """像素座標 → 格子座標 (col, row)，自動 clamp 在合法範圍內"""
        col = int(x // self.cell_size)
        row = int(y // self.cell_size)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return col, row

    def world_pos(self, col, row):
        """格子座標 → 格子中心的像素座標 (x, y)"""
        x = col * self.cell_size + self.cell_size / 2
        y = row * self.cell_size + self.cell_size / 2
        return (x, y)

    def is_valid(self, col, row):
        """格子座標是否在地圖範圍內"""
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_blocked(self, col, row):
        """該格是否為障礙（超出範圍也視為障礙）"""
        if not self.is_valid(col, row):
            return True
        return self.blocked[row][col]

    # ------------------------------------------------------------------ #
    #  從 Obstacle 列表建立 Grid                                            #
    # ------------------------------------------------------------------ #

    def build_from_obstacles(self, obstacles):
        """
        把 Assignment 1 的 Obstacle 列表轉換成格子障礙標記。
        - STONE：圓形，以圓心 + 半徑判斷格子中心是否在圓內
        - FENCE：矩形，以 pygame.Rect 判斷格子中心是否在矩形內
        建完後自動計算 clearance。
        """
        # 先清空
        for row in range(self.rows):
            for col in range(self.cols):
                self.blocked[row][col] = False

        for obs in obstacles:
            if obs.type == "STONE":
                self._mark_circle(obs.pos.x, obs.pos.y, obs.radius)
            elif obs.type == "FENCE":
                self._mark_rect(obs.rect)

        # 障礙建好後計算 clearance
        self._compute_clearance()

    def _mark_circle(self, cx, cy, radius):
        """把圓形障礙覆蓋到的格子標記為 blocked"""
        # 只掃描包圍盒範圍內的格子，避免全圖掃描
        col_min, row_min = self.cell_of(cx - radius, cy - radius)
        col_max, row_max = self.cell_of(cx + radius, cy + radius)

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                # 格子中心
                wx, wy = self.world_pos(col, row)
                dist_sq = (wx - cx) ** 2 + (wy - cy) ** 2
                # 加一點 padding（半格），讓羊不要緊貼石頭邊緣走
                if dist_sq <= (radius + self.cell_size * 0.3) ** 2:
                    self.blocked[row][col] = True

    def _mark_rect(self, rect):
        """
        把矩形障礙覆蓋到的格子標記為 blocked。
        使用 AABB overlap 判斷：只要格子矩形與障礙矩形有任何重疊就標記。
        這樣能正確處理高度或寬度小於 cell_size 的細長柵欄。
        """
        cs = self.cell_size
        # 加一點 padding，讓羊不要緊貼柵欄邊緣
        pad = cs * 0.15
        obs_left   = rect.left   - pad
        obs_top    = rect.top    - pad
        obs_right  = rect.right  + pad
        obs_bottom = rect.bottom + pad

        # 計算可能重疊的格子範圍（多掃一格保險）
        col_min = max(0, int(obs_left  // cs))
        row_min = max(0, int(obs_top   // cs))
        col_max = min(self.cols - 1, int(obs_right  // cs))
        row_max = min(self.rows - 1, int(obs_bottom // cs))

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                # 格子矩形範圍
                cell_left   = col * cs
                cell_top    = row * cs
                cell_right  = cell_left + cs
                cell_bottom = cell_top  + cs

                # AABB overlap：兩個矩形在 X 和 Y 軸上都有重疊才算
                overlap = (cell_left   < obs_right  and
                           cell_right  > obs_left   and
                           cell_top    < obs_bottom and
                           cell_bottom > obs_top)
                if overlap:
                    self.blocked[row][col] = True

    # ------------------------------------------------------------------ #
    #  手動標記（用於羊圈、特殊格子）                                         #
    # ------------------------------------------------------------------ #

    def mark_blocked(self, col, row, blocked=True):
        """手動設定某格的通行性，用於羊圈牆壁等特殊區域"""
        if self.is_valid(col, row):
            self.blocked[row][col] = blocked

    # ------------------------------------------------------------------ #
    #  Neighbors（給 Dijkstra / A* 用）                                    #
    # ------------------------------------------------------------------ #

    # 4 向：上下左右
    _DIRS_4 = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    def neighbors(self, col, row):
        """
        回傳 (neighbor_col, neighbor_row, cost) 的列表。
        4 向移動，cost 一律為 1.0（可通行格）。
        羊圈入口格子不標為 blocked，所以路徑可以通過。
        """
        result = []
        for dc, dr in self._DIRS_4:
            nc, nr = col + dc, row + dr
            if not self.is_blocked(nc, nr):
                result.append((nc, nr, 1.0))
        return result

    # ------------------------------------------------------------------ #
    #  Clearance 計算（Multi-source BFS）                                  #
    # ------------------------------------------------------------------ #

    def _compute_clearance(self):
        """
        用 Multi-source BFS 計算每個可通行格到最近障礙格的距離。
        障礙格本身 clearance = 0。
        開放格的 clearance = 離最近障礙的格子數（Manhattan BFS 距離）。

        這個值之後給 A* 的 clearance heuristic 使用：
          clearance 越低代表越靠近牆，cost 應該越高。
        """
        INF = 10**9
        dist = [[INF] * self.cols for _ in range(self.rows)]
        queue = deque()

        # 所有障礙格入隊，距離設為 0
        for row in range(self.rows):
            for col in range(self.cols):
                if self.blocked[row][col]:
                    dist[row][col] = 0
                    queue.append((col, row))

        # BFS 向外擴張
        while queue:
            col, row = queue.popleft()
            for dc, dr in self._DIRS_4:
                nc, nr = col + dc, row + dr
                if self.is_valid(nc, nr) and dist[nr][nc] == INF:
                    dist[nr][nc] = dist[row][col] + 1
                    queue.append((nc, nr))

        # 寫回 clearance
        for row in range(self.rows):
            for col in range(self.cols):
                self.clearance[row][col] = dist[row][col]

    def get_clearance(self, col, row):
        """取得某格的 clearance 值（格子數），超出範圍回傳 0"""
        if not self.is_valid(col, row):
            return 0
        return self.clearance[row][col]

    # ------------------------------------------------------------------ #
    #  Debug 視覺化                                                        #
    # ------------------------------------------------------------------ #

    def draw_debug(self, screen, show_clearance=False):
        """
        在畫面上疊加格子顏色：
          - 紅色半透明：障礙格
          - 藍色漸層（可選）：clearance 越低越深藍，顯示「危險區域」
          - 灰色格線：格子邊界
        按 G 鍵切換顯示。
        """
        cs = self.cell_size

        for row in range(self.rows):
            for col in range(self.cols):
                x = col * cs
                y = row * cs

                if self.blocked[row][col]:
                    # 障礙格：紅色半透明
                    s = pygame.Surface((cs, cs), pygame.SRCALPHA)
                    s.fill((200, 50, 50, 100))
                    screen.blit(s, (x, y))

                elif show_clearance:
                    # clearance 視覺化：clearance=1 最深藍，越大越淡
                    c = self.clearance[row][col]
                    max_c = 5  # 超過 5 格就不顯示
                    if c <= max_c:
                        alpha = int(80 * (1 - (c - 1) / max_c))
                        s = pygame.Surface((cs, cs), pygame.SRCALPHA)
                        s.fill((50, 50, 200, alpha))
                        screen.blit(s, (x, y))

        # 格線
        for col in range(self.cols + 1):
            pygame.draw.line(screen, (180, 180, 180),
                             (col * cs, 0), (col * cs, self.rows * cs), 1)
        for row in range(self.rows + 1):
            pygame.draw.line(screen, (180, 180, 180),
                             (0, row * cs), (self.cols * cs, row * cs), 1)

    def draw_path(self, screen, path, color=(255, 215, 0), width=3):
        """
        把 A* / Dijkstra 回傳的路徑畫成連線。
        path: [(col, row), ...] 的列表
        """
        if len(path) < 2:
            return
        points = [self.world_pos(col, row) for col, row in path]
        pygame.draw.lines(screen, color, False, points, width)

        # 在每個節點畫小圓點
        for px, py in points:
            pygame.draw.circle(screen, color, (int(px), int(py)), 4)
