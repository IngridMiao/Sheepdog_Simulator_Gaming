from entity.base_agent import BaseAgent

class SheepDog(BaseAgent):
    def __init__(self, x, y):
        super().__init__(x, y, (160, 82, 45)) # 棕色
        self.max_speed = 180.0