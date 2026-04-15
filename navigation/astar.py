"""
navigation/astar.py

A* 最短路徑算法，支援兩種 heuristic：

  1. EUCLIDEAN（標準）
     h(n) = 直線距離到終點
     - 在 4 向移動的 grid 上是 admissible（不高估），但比 Manhattan 更寬鬆
     - 路徑傾向往終點方向走，但不完全是 Manhattan 的「L 形」

  2. CLEARANCE（創意）
     h(n) = euclidean_distance + penalty
     penalty = w / (clearance(n) + 1)
       clearance(n) = 該格到最近障礙的距離（格子數），由 GridMap 預計算
       w            = clearance_weight（可調，預設 3.0）
     - clearance 越低（越靠近牆）→ penalty 越大 → 搜尋傾向走開闊通道
     - 讓羊在規劃路徑時自然避開貼牆走法，符合動物直覺
     - 注意：加入 penalty 後 heuristic 不再保證 admissible，
       但在實務上路徑品質更好，可作為報告的 admissibility vs. quality

設計說明：
  A* 的搜尋函式接受 heuristic="EUCLIDEAN" 或 "CLEARANCE" 參數，
  方便在 simulation 裡切換做比較，不需要重複建立多個物件。

  額外記錄：
    - explored_count：擴展節點數
    - explored_cells：探索順序（視覺化用）
"""

import heapq
import math


class AStar:
    def __init__(self, grid_map, clearance_weight=3.0):
        """
        grid_map         : GridMap 實例
        clearance_weight : clearance heuristic 的 penalty 權重（越大越傾向走開闊路）
        """
        self.grid = grid_map
        self.clearance_weight = clearance_weight

        # 上一次搜尋的統計資料
        self.explored_count = 0
        self.explored_cells = []

    # ------------------------------------------------------------------ #
    #  公開介面                                                             #
    # ------------------------------------------------------------------ #

    def search(self, start_col, start_row, goal_col, goal_row,
               heuristic="EUCLIDEAN"):
        """
        執行 A* 搜尋。

        heuristic: "EUCLIDEAN" | "CLEARANCE"

        回傳 (path, cost)：
          path: [(col, row), ...] 從起點到終點
          cost: g_cost（實際走過的 edge cost 總和，不含 heuristic 部分）
          找不到則回傳 ([], inf)
        """
        self.explored_count = 0
        self.explored_cells = []

        start = (start_col, start_row)
        goal  = (goal_col,  goal_row)

        if self.grid.is_blocked(*start) or self.grid.is_blocked(*goal):
            return [], float('inf')
        if start == goal:
            return [start], 0.0

        # 選擇 heuristic 函式
        h_func = self._h_euclidean if heuristic == "EUCLIDEAN" else self._h_clearance

        INF = float('inf')
        g_cost = [[INF] * self.grid.cols for _ in range(self.grid.rows)]
        g_cost[start_row][start_col] = 0.0

        came_from = [[None] * self.grid.cols for _ in range(self.grid.rows)]

        # Priority queue：(f = g + h, g, col, row)
        # 同 f 時以 g 大的優先（已走越遠越優先），避免大量 tie-breaking
        h0 = h_func(start_col, start_row, goal_col, goal_row)
        pq = [(h0, 0.0, start_col, start_row)]

        while pq:
            f, g, col, row = heapq.heappop(pq)

            # Lazy deletion
            if g > g_cost[row][col]:
                continue

            self.explored_count += 1
            self.explored_cells.append((col, row))

            if (col, row) == goal:
                path = self._reconstruct(came_from, start, goal)
                return path, g_cost[goal_row][goal_col]

            for nc, nr, edge_cost in self.grid.neighbors(col, row):
                new_g = g + edge_cost
                if new_g < g_cost[nr][nc]:
                    g_cost[nr][nc] = new_g
                    came_from[nr][nc] = (col, row)
                    h = h_func(nc, nr, goal_col, goal_row)
                    heapq.heappush(pq, (new_g + h, new_g, nc, nr))

        return [], float('inf')

    # ------------------------------------------------------------------ #
    #  Heuristic 函式                                                      #
    # ------------------------------------------------------------------ #

    def _h_euclidean(self, col, row, goal_col, goal_row):
        """
        Heuristic 1：Euclidean Distance
        h = sqrt((Δcol)² + (Δrow)²)
        在 4 向移動中是 admissible（不高估實際 cost）。
        """
        dc = col - goal_col
        dr = row - goal_row
        return math.sqrt(dc * dc + dr * dr)

    def _h_clearance(self, col, row, goal_col, goal_row):
        """
        Heuristic 2：Clearance-weighted（環境感知型）
        h = euclidean_distance + w / (clearance + 1)

        clearance = 該格到最近障礙的距離（格子數）
          - clearance = 0 → 障礙格本身，penalty = w / 1 = w（最大）
          - clearance = 1 → 緊鄰障礙，penalty = w / 2
          - clearance = 5 → 開闊地帶，penalty ≈ w / 6（接近 0）

        效果：
          - 讓 A* 在路徑相近時優先選擇離牆較遠的格子
          - 羊的路徑自然走開闊通道，不貼著石頭或柵欄邊緣擠過去
          - 此 heuristic 不保證 admissible（可能高估），
            但路徑品質（安全性）更好，可作報告中 admissibility vs. quality 的討論點
        """
        h_base = self._h_euclidean(col, row, goal_col, goal_row)
        clearance = self.grid.get_clearance(col, row)
        penalty = self.clearance_weight / (clearance + 1)
        return h_base + penalty

    # ------------------------------------------------------------------ #
    #  路徑重建                                                             #
    # ------------------------------------------------------------------ #

    def _reconstruct(self, came_from, start, goal):
        path = []
        current = goal
        while current != start:
            path.append(current)
            col, row = current
            current = came_from[row][col]
        path.append(start)
        path.reverse()
        return path
