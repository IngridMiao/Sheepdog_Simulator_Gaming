"""
decision/satiation.py

飽腹感系統（Satiation System）
────────────────────────────────────────────────────────────────────────────
每棵草叢有獨立的飽腹感值 satiation ∈ [0.0, 1.0]：
  - 初始值 1.0（草叢很新鮮，羊很想吃）
  - 每吃一次下降 DECAY_RATE
  - 永遠不會降到 MIN_SATIATION（羊肚子再飽也還是會吃，只是不那麼想）

這個模組只負責「記憶」，不做決策。
決策邏輯在 utility_system.py 和 rule_decision.py 中使用這裡的值。
"""


class SatiationTracker:
    """
    追蹤每棵草叢的飽腹感。

    Parameters
    ----------
    decay_rate   : float  每吃一次飽腹感下降多少（預設 0.35）
    min_satiation: float  飽腹感下限（預設 0.1，確保草叢不會被完全忽略）
    """

    DECAY_RATE    = 0.35
    MIN_SATIATION = 0.10

    def __init__(self, bushes, decay_rate=None, min_satiation=None):
        self.decay_rate    = decay_rate    if decay_rate    is not None else self.DECAY_RATE
        self.min_satiation = min_satiation if min_satiation is not None else self.MIN_SATIATION

        # bush_id → 被吃次數
        self._eaten_count: dict[int, int] = {id(b): 0 for b in bushes}

    # ------------------------------------------------------------------ #

    def register_eaten(self, bush):
        """
        通知系統：bush 剛被吃了一次。
        在 sheep.py 的 _update_eating 完成後呼叫。
        """
        key = id(bush)
        if key in self._eaten_count:
            self._eaten_count[key] += 1
        else:
            self._eaten_count[key] = 1

    def satiation(self, bush) -> float:
        """
        回傳 bush 的飽腹感值 ∈ [min_satiation, 1.0]。
        公式：max(min, 1.0 - eaten_count * decay_rate)
        """
        count = self._eaten_count.get(id(bush), 0)
        return max(self.min_satiation, 1.0 - count * self.decay_rate)

    def eaten_count(self, bush) -> int:
        """回傳 bush 被吃了幾次"""
        return self._eaten_count.get(id(bush), 0)

    def reset(self, bushes):
        """重置所有飽腹感（場景重置時呼叫）"""
        self._eaten_count = {id(b): 0 for b in bushes}

    # ------------------------------------------------------------------ #
    #  Debug 用                                                           #
    # ------------------------------------------------------------------ #

    def summary(self, bushes) -> str:
        parts = []
        for i, b in enumerate(bushes):
            s = self.satiation(b)
            n = self.eaten_count(b)
            parts.append(f"Bush{chr(65+i)}(×{n} s={s:.2f})")
        return "  ".join(parts)
