"""
navigation/waypoint_graph.py

手動定義的 Waypoint Graph 表示法，作為 Grid 的比較用替代表示法。

設計說明：
  Grid 把場景切成均勻格子，節點多（192 個）但路徑折線明顯。
  Waypoint Graph 只在「有意義的位置」放節點（路口、草叢旁、羊圈入口等），
  節點少（~15 個），搜尋快，但路徑較粗糙、細節區域走不到。

  兩者的差異是 Part 2 報告的核心分析材料：
    - 路徑形狀：Grid 折線 vs Waypoint 直線段
    - 節點數量：192 vs ~15
    - 搜尋效率：Waypoint 快，因為 branching factor 低
    - 覆蓋精度：Grid 可走到任何開放格，Waypoint 只能走到預定節點

  Waypoint Graph 也實作 search() 介面（Dijkstra 與 A*），
  讓 Simulation 可以用同樣的方式呼叫，方便切換比較。

節點佈局（對應 1024×768 場景，見 simulation.py 的障礙配置）：

  ID  位置說明                  pixel (x, y)
   0  羊起始區（左中）           (96,  480)
   1  左側開放區（上）           (96,  160)
   2  草叢 A 旁（左上）          (128, 160)   ← 草叢 A
   3  中央上方路口               (384, 160)
   4  草叢 B 旁（右上）          (864, 160)   ← 草叢 B
   5  中上通道（石牆右側）        (384, 256)
   6  中央區                    (384, 448)
   7  草叢 C 旁（中央）          (416, 416)   ← 草叢 C
   8  中下通道                  (384, 576)
   9  草叢 D 旁（下中）          (480, 608)   ← 草叢 D
  10  左下開放區                (160, 608)
  11  右側中央（柵欄左外）        (672, 352)
  12  羊圈入口（左側開口）        (672, 480)   ← 羊圈目標
  13  右上通道                  (672, 192)
  14  中右區                    (576, 352)
"""

import math
import heapq
import pygame
from utils.vector_math import Vector2


class WaypointGraph:

    def __init__(self):
        # nodes[i] = (x, y) 像素座標
        self.nodes = []
        # edges[i] = [(j, cost), ...]  有向邊（雙向各存一次）
        self.edges = []

        self._build_default_graph()

        # 上一次搜尋的統計
        self.explored_count = 0
        self.explored_nodes = []

    # ------------------------------------------------------------------ #
    #  圖的建構                                                            #
    # ------------------------------------------------------------------ #

    def _build_default_graph(self):
        """
        建立對應 simulation.py 場景的預設 waypoint graph。
        節點位置根據地圖障礙物手動調整，確保所有邊都不穿過障礙。
        """
        # ── 定義節點 ─────────────────────────────────────────────────
        self.nodes = [
            (96,  480),   #  0 羊起始區
            (96,  160),   #  1 左側上方
            (128, 160),   #  2 草叢 A（左上）
            (384, 160),   #  3 中央上方路口
            (864, 160),   #  4 草叢 B（右上）
            (384, 256),   #  5 中上通道
            (384, 448),   #  6 中央區
            (416, 416),   #  7 草叢 C（中央）
            (384, 576),   #  8 中下通道
            (480, 608),   #  9 草叢 D（下中）
            (160, 608),   # 10 左下開放區
            (672, 352),   # 11 右側中央（柵欄左外）
            (672, 480),   # 12 羊圈入口
            (672, 192),   # 13 右上通道
            (576, 352),   # 14 中右區
        ]

        # edges 初始化（每個節點一個空列表）
        self.edges = [[] for _ in range(len(self.nodes))]

        # ── 定義邊（雙向）────────────────────────────────────────────
        # 格式：(節點A, 節點B)  → 自動計算 Euclidean cost
        connections = [
            ( 0,  1),   # 羊起始 ↔ 左側上方
            ( 0, 10),   # 羊起始 ↔ 左下
            ( 0,  6),   # 羊起始 ↔ 中央區
            ( 1,  2),   # 左上 ↔ 草叢 A
            ( 1,  3),   # 左上 ↔ 中央上方
            ( 2,  3),   # 草叢 A ↔ 中央上方
            ( 3,  4),   # 中央上方 ↔ 草叢 B（右上，長邊，需確認不穿牆）
            ( 3,  5),   # 中央上方 ↔ 中上通道
            ( 3, 13),   # 中央上方 ↔ 右上通道
            ( 4, 13),   # 草叢 B ↔ 右上通道
            ( 5,  6),   # 中上通道 ↔ 中央區
            ( 6,  7),   # 中央區 ↔ 草叢 C
            ( 6,  8),   # 中央區 ↔ 中下通道
            ( 6, 14),   # 中央區 ↔ 中右區
            ( 7,  8),   # 草叢 C ↔ 中下通道（近似）
            ( 8,  9),   # 中下通道 ↔ 草叢 D
            ( 8, 10),   # 中下通道 ↔ 左下
            (10,  0),   # 左下 ↔ 羊起始（冗餘但讓圖更連通）
            (11, 12),   # 右側中央 ↔ 羊圈入口
            (11, 13),   # 右側中央 ↔ 右上通道
            (11, 14),   # 右側中央 ↔ 中右區
            (12, 14),   # 羊圈入口 ↔ 中右區
            (13, 14),   # 右上通道 ↔ 中右區
            (14,  6),   # 中右區 ↔ 中央區
        ]

        for a, b in connections:
            cost = self._euclidean(a, b)
            self.edges[a].append((b, cost))
            self.edges[b].append((a, cost))

    def _euclidean(self, i, j):
        ax, ay = self.nodes[i]
        bx, by = self.nodes[j]
        return math.sqrt((ax-bx)**2 + (ay-by)**2)

    # ------------------------------------------------------------------ #
    #  節點查詢                                                            #
    # ------------------------------------------------------------------ #

    def nearest_node(self, x, y):
        """
        找離像素座標 (x, y) 最近的 waypoint 節點索引。
        用於把 agent 的當前位置 snap 到圖上的起點。
        """
        best_i    = 0
        best_dist = float('inf')
        for i, (nx, ny) in enumerate(self.nodes):
            d = math.sqrt((x-nx)**2 + (y-ny)**2)
            if d < best_dist:
                best_dist = d
                best_i    = i
        return best_i

    def node_pos(self, i):
        """回傳節點 i 的像素座標 (x, y)"""
        return self.nodes[i]

    def node_count(self):
        return len(self.nodes)

    def edge_count(self):
        return sum(len(e) for e in self.edges) // 2

    # ------------------------------------------------------------------ #
    #  Dijkstra                                                           #
    # ------------------------------------------------------------------ #

    def dijkstra(self, start_node, goal_node):
        """
        在 Waypoint Graph 上執行 Dijkstra。

        start_node, goal_node : 節點索引（int）
        回傳 (path, cost)
          path : [node_index, ...] 節點索引序列
          cost : 路徑總 cost（Euclidean 距離總和）
        """
        self.explored_count = 0
        self.explored_nodes = []

        n = len(self.nodes)
        dist = [float('inf')] * n
        dist[start_node] = 0.0
        came_from = [None] * n

        pq = [(0.0, start_node)]

        while pq:
            cost, u = heapq.heappop(pq)
            if cost > dist[u]:
                continue

            self.explored_count += 1
            self.explored_nodes.append(u)

            if u == goal_node:
                return self._reconstruct(came_from, start_node, goal_node), dist[goal_node]

            for v, edge_cost in self.edges[u]:
                new_cost = dist[u] + edge_cost
                if new_cost < dist[v]:
                    dist[v] = new_cost
                    came_from[v] = u
                    heapq.heappush(pq, (new_cost, v))

        return [], float('inf')

    # ------------------------------------------------------------------ #
    #  A*                                                                 #
    # ------------------------------------------------------------------ #

    def astar(self, start_node, goal_node):
        """
        在 Waypoint Graph 上執行 A*（Euclidean heuristic）。
        Waypoint 間距離本身就是 Euclidean，heuristic 自然 admissible。
        """
        self.explored_count = 0
        self.explored_nodes = []

        n = len(self.nodes)
        g = [float('inf')] * n
        g[start_node] = 0.0
        came_from = [None] * n

        gx, gy = self.nodes[goal_node]

        def h(i):
            nx, ny = self.nodes[i]
            return math.sqrt((nx-gx)**2 + (ny-gy)**2)

        pq = [(h(start_node), 0.0, start_node)]

        while pq:
            f, cost, u = heapq.heappop(pq)
            if cost > g[u]:
                continue

            self.explored_count += 1
            self.explored_nodes.append(u)

            if u == goal_node:
                return self._reconstruct(came_from, start_node, goal_node), g[goal_node]

            for v, edge_cost in self.edges[u]:
                new_g = g[u] + edge_cost
                if new_g < g[v]:
                    g[v] = new_g
                    came_from[v] = u
                    heapq.heappush(pq, (new_g + h(v), new_g, v))

        return [], float('inf')

    # ------------------------------------------------------------------ #
    #  高階搜尋介面（供 Simulation 直接呼叫）                               #
    # ------------------------------------------------------------------ #

    def search(self, start_x, start_y, goal_x, goal_y, algo="ASTAR"):
        """
        從像素座標搜尋到像素座標。
        自動 snap 到最近節點，回傳像素座標路徑。

        algo : "ASTAR" | "DIJKSTRA"
        回傳 (pixel_path, cost)
          pixel_path : [(x, y), ...] 像素座標（可直接傳給 PathFollower 或畫線）
          cost       : 路徑 Euclidean 總距離
        """
        start_i = self.nearest_node(start_x, start_y)
        goal_i  = self.nearest_node(goal_x,  goal_y)

        if start_i == goal_i:
            return [self.nodes[start_i]], 0.0

        if algo == "ASTAR":
            node_path, cost = self.astar(start_i, goal_i)
        else:
            node_path, cost = self.dijkstra(start_i, goal_i)

        pixel_path = [self.nodes[i] for i in node_path]
        return pixel_path, cost

    # ------------------------------------------------------------------ #
    #  路徑重建                                                            #
    # ------------------------------------------------------------------ #

    def _reconstruct(self, came_from, start, goal):
        path = []
        cur = goal
        while cur != start:
            path.append(cur)
            cur = came_from[cur]
        path.append(start)
        path.reverse()
        return path

    # ------------------------------------------------------------------ #
    #  Debug 視覺化                                                        #
    # ------------------------------------------------------------------ #

    def draw_graph(self, screen,
                   node_color=(80, 160, 255),
                   edge_color=(80, 120, 200),
                   label_color=(200, 220, 255)):
        """
        繪製整個 waypoint graph（節點 + 邊）。
        按 W 鍵切換顯示。
        """
        font = pygame.font.SysFont("Arial", 12)

        # 邊
        for i, nbrs in enumerate(self.edges):
            ax, ay = self.nodes[i]
            for j, _ in nbrs:
                if j > i:   # 每條邊只畫一次
                    bx, by = self.nodes[j]
                    pygame.draw.line(screen, edge_color, (ax, ay), (bx, by), 1)

        # 節點
        for i, (x, y) in enumerate(self.nodes):
            pygame.draw.circle(screen, node_color, (x, y), 6)
            label = font.render(str(i), True, label_color)
            screen.blit(label, (x + 7, y - 6))

    def draw_path(self, screen, pixel_path,
                  path_color=(255, 140, 0), width=3):
        """繪製 search() 回傳的像素座標路徑"""
        if len(pixel_path) < 2:
            return
        pygame.draw.lines(screen, path_color, False,
                          [(int(x), int(y)) for x, y in pixel_path], width)
        for x, y in pixel_path:
            pygame.draw.circle(screen, path_color, (int(x), int(y)), 5)