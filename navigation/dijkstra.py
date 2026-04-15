"""
navigation/dijkstra.py

Dijkstra 最短路徑算法。
- 輸入：GridMap、起點格子、終點格子
- 輸出：[(col, row), ...] 的路徑列表（含起點和終點）
         若無法到達則回傳空列表

設計說明：
  Dijkstra 不使用 heuristic，保證找到 cost 最小的路徑。
  這裡的 edge cost 全為 1.0（4向等距移動），
  所以 Dijkstra 等同於 BFS，但保留 cost 結構以便和 A* 做公平比較。
  
  為了報告分析，額外記錄：
    - explored_count：探索過的節點數
    - explored_cells：探索順序（可視覺化搜尋擴散過程）
"""

import heapq


class Dijkstra:
    def __init__(self, grid_map):
        """
        grid_map: GridMap 實例
        """
        self.grid = grid_map

        # 上一次搜尋的統計資料（給報告用）
        self.explored_count = 0
        self.explored_cells = []   # [(col, row), ...] 依探索順序

    def search(self, start_col, start_row, goal_col, goal_row):
        """
        執行 Dijkstra 搜尋。

        回傳 (path, cost)：
          path: [(col, row), ...] 從起點到終點，找不到則回傳 ([], inf)
          cost: 路徑總 cost
        """
        # 重置統計
        self.explored_count = 0
        self.explored_cells = []

        start = (start_col, start_row)
        goal  = (goal_col,  goal_row)

        # 起點或終點是障礙 → 直接失敗
        if self.grid.is_blocked(*start) or self.grid.is_blocked(*goal):
            return [], float('inf')

        # 若起終點相同
        if start == goal:
            return [start], 0.0

        # dist[row][col] = 從起點到該格的最短已知 cost
        INF = float('inf')
        dist = [[INF] * self.grid.cols for _ in range(self.grid.rows)]
        dist[start_row][start_col] = 0.0

        # came_from[row][col] = 前驅格子 (col, row)，用於重建路徑
        came_from = [[None] * self.grid.cols for _ in range(self.grid.rows)]

        # Priority queue：(累積cost, col, row)
        pq = [(0.0, start_col, start_row)]

        while pq:
            current_cost, col, row = heapq.heappop(pq)

            # 已找到更短路徑的節點，跳過（lazy deletion）
            if current_cost > dist[row][col]:
                continue

            self.explored_count += 1
            self.explored_cells.append((col, row))

            # 到達終點
            if (col, row) == goal:
                path = self._reconstruct(came_from, start, goal)
                return path, current_cost

            # 擴展 neighbors
            for nc, nr, edge_cost in self.grid.neighbors(col, row):
                new_cost = current_cost + edge_cost
                if new_cost < dist[nr][nc]:
                    dist[nr][nc] = new_cost
                    came_from[nr][nc] = (col, row)
                    heapq.heappush(pq, (new_cost, nc, nr))

        # 無法到達
        return [], float('inf')

    def _reconstruct(self, came_from, start, goal):
        """從 came_from 反向追蹤，重建路徑"""
        path = []
        current = goal
        while current != start:
            path.append(current)
            col, row = current
            current = came_from[row][col]
        path.append(start)
        path.reverse()
        return path
