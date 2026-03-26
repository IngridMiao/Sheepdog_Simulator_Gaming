import pygame
import sys
from simulation import Simulation

def main():
    pygame.init()
    screen_width, screen_height = 1024, 768
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Sheepdog Simulation: HW1")

    # 建立模擬引擎
    sim = Simulation(screen_width, screen_height)

    # 控制影格率(FPS)；取得 delta time 以實現時間驅動
    clock = pygame.time.Clock()
    running = True

    while running:

        # 計算兩幀之間的時間差(s)
        dt = clock.tick(60) / 1000.0

        # 處理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # 運行時切換模式
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    sim.set_mode("KINEMATIC") 
                elif event.key == pygame.K_2:
                    sim.set_mode("STEERING") 
                elif event.key == pygame.K_3:
                    sim.set_mode("COMBINED")
                elif event.key == pygame.K_d:
                    sim.toggle_debug() # on/off 視覺偵錯
        
        # 更新邏輯
        sim.update(dt)

        # 繪製
        screen.fill((40, 80, 40)) # 草地背景
        sim.draw(screen)
        pygame.display.flip()
        
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()