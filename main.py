import pygame
import sys
from simulation import Simulation

"""
新增按鍵：
  1 / 2 / 3  → 切換模式（Kinematic / Steering / Navigation）
  P          → 循環切換 pathfinding 算法
  F          → 切換 path following 策略
  G          → 切換 Grid debug 視覺化
  C          → 切換 Clearance heatmap
  W          → 切換 Waypoint Graph 視覺化
  D          → 切換 debug 資訊
  R          → 重置場景
"""

def main():
    pygame.init()
    screen_width, screen_height = 1024, 768
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Sheepdog Simulation: HW2")

    # 建立模擬引擎
    sim = Simulation(screen_width, screen_height)

    # 控制影格率(FPS)；取得 delta time 以實現時間驅動
    clock = pygame.time.Clock()

    while True:

        # 計算兩幀之間的時間差(s)
        dt = clock.tick(60) / 1000.0

        # 處理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            
            # 運行時切換模式
            if event.type == pygame.KEYDOWN:
                if   event.key == pygame.K_1: sim.set_mode("KINEMATIC")
                elif event.key == pygame.K_2: sim.set_mode("STEERING")
                elif event.key == pygame.K_3: sim.set_mode("NAVIGATION")
                elif event.key == pygame.K_d: sim.toggle_debug()
                elif event.key == pygame.K_g: sim.toggle_grid()
                elif event.key == pygame.K_c: sim.toggle_clearance()
                elif event.key == pygame.K_w: sim.toggle_waypoint()
                elif event.key == pygame.K_p: sim.cycle_algo()
                elif event.key == pygame.K_f: sim.cycle_follower()
                elif event.key == pygame.K_r: sim.set_mode(sim.mode) 
        
        # 更新邏輯
        sim.update(dt)

        # 繪製
        screen.fill((100,180,100)) # 草地背景 (淺綠色)# 144, 238, 144
        sim.draw(screen)
        pygame.display.flip()
        
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()