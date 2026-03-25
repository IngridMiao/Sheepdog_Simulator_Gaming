from entity.base_agent import BaseAgent

class Sheep(BaseAgent):
    def __init__(self, x, y):
        super().__init__(x, y, (255, 255, 255)) # 白色
        self.max_speed = 100.0 # 羊跑得比狗慢一點