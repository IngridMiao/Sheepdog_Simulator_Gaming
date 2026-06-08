"""
decision/utility_system.py

效用決策系統（Utility-based Decision System）
────────────────────────────────────────────────────────────────────────────
對每棵剩餘草叢計算效用分數，選分數最高的。

效用公式：
    U(bush) = w_dist     * score_distance(bush)
            + w_safety   * score_safety(bush)
            + w_satiation* score_satiation(bush)

各分項都正規化到 [0, 1]：
  score_distance  : 距離越近越高（1 - norm_dist）
  score_safety    : 草叢離狗越遠越高（norm_dog_dist）
  score_satiation : 直接用 SatiationTracker.satiation() 的值

Weights 可從外部動態調整（adaptive component 就是調這裡）。

設計考量
────────
- 當只剩一棵草叢時直接選它（不需要比較）。
- 正規化用「最大可能距離」（地圖對角線）避免除以 0。
- dog_pos 可以是 None（用於測試 / 狗不存在的實驗）。
"""

import math


class UtilitySystem:

    # 預設權重
    DEFAULT_W_DIST     = 0.40
    DEFAULT_W_SAFETY   = 0.30
    DEFAULT_W_SATIATION= 0.30

    # 正規化用的最大距離（1024×768 地圖對角線 ≈ 1280 px）
    MAX_DIST = math.hypot(1024, 768)

    def __init__(self, satiation_tracker,
                 w_dist=None, w_safety=None, w_satiation=None):
        """
        Parameters
        ----------
        satiation_tracker : SatiationTracker
        w_dist, w_safety, w_satiation : float | None
            初始權重；None 時使用預設值。
        """
        self.satiation = satiation_tracker

        self.w_dist      = w_dist      if w_dist      is not None else self.DEFAULT_W_DIST
        self.w_safety    = w_safety    if w_safety    is not None else self.DEFAULT_W_SAFETY
        self.w_satiation = w_satiation if w_satiation is not None else self.DEFAULT_W_SATIATION

        # 供 debug / 記錄用：上一次選擇的詳細分數
        self.last_scores: list[dict] = []

    # ------------------------------------------------------------------ #
    #  主要介面                                                            #
    # ------------------------------------------------------------------ #

    def choose(self, sheep_pos, remaining_bushes, dog_pos=None):
        """
        從 remaining_bushes 中選出效用最高的草叢。

        Parameters
        ----------
        sheep_pos        : Vector2  羊的當前位置
        remaining_bushes : list[Bush]  尚未吃完的草叢
        dog_pos          : Vector2 | None  狗的當前位置

        Returns
        -------
        Bush  最佳草叢（若 remaining_bushes 為空則回傳 None）
        """
        if not remaining_bushes:
            return None
        if len(remaining_bushes) == 1:
            return remaining_bushes[0]

        self.last_scores = []
        best_bush  = None
        best_score = -1.0

        for bush in remaining_bushes:
            score, breakdown = self._score(sheep_pos, bush, dog_pos)
            self.last_scores.append(breakdown)
            if score > best_score:
                best_score = score
                best_bush  = bush

        return best_bush

    # ------------------------------------------------------------------ #
    #  分數計算                                                            #
    # ------------------------------------------------------------------ #

    def _score(self, sheep_pos, bush, dog_pos):
        """
        計算單棵草叢的效用分數。
        回傳 (total_score, breakdown_dict)。
        """
        # ── Distance score（越近越高）────────────────────────────────
        dist_to_bush = sheep_pos.distance_to(bush.pos)
        s_dist = 1.0 - min(dist_to_bush / self.MAX_DIST, 1.0)

        # ── Safety score（草叢離狗越遠越高）──────────────────────────
        if dog_pos is not None:
            dist_to_dog = bush.pos.distance_to(dog_pos)
            s_safety = min(dist_to_dog / self.MAX_DIST, 1.0)
        else:
            s_safety = 1.0   # 沒有狗，安全分全滿

        # ── Satiation score（飽腹感越高越想去）───────────────────────
        s_satiation = self.satiation.satiation(bush)

        # ── 加權總分 ─────────────────────────────────────────────────
        total = (self.w_dist      * s_dist
               + self.w_safety   * s_safety
               + self.w_satiation * s_satiation)

        breakdown = {
            "bush_pos"   : (bush.pos.x, bush.pos.y),
            "s_dist"     : round(s_dist,      3),
            "s_safety"   : round(s_safety,    3),
            "s_satiation": round(s_satiation, 3),
            "total"      : round(total,        3),
            "w_dist"     : round(self.w_dist,      3),
            "w_safety"   : round(self.w_safety,    3),
            "w_satiation": round(self.w_satiation, 3),
        }
        return total, breakdown

    # ------------------------------------------------------------------ #
    #  權重查詢 / 設定（供 adaptive component 呼叫）                       #
    # ------------------------------------------------------------------ #

    def set_weights(self, w_dist=None, w_safety=None, w_satiation=None):
        """直接設定一個或多個權重"""
        if w_dist      is not None: self.w_dist      = w_dist
        if w_safety    is not None: self.w_safety    = w_safety
        if w_satiation is not None: self.w_satiation = w_satiation

    def get_weights(self) -> dict:
        return {
            "w_dist"     : self.w_dist,
            "w_safety"   : self.w_safety,
            "w_satiation": self.w_satiation,
        }

    # ------------------------------------------------------------------ #
    #  Debug                                                              #
    # ------------------------------------------------------------------ #

    def debug_str(self) -> str:
        """回傳上一次選擇的分數摘要（供 HUD 顯示）"""
        if not self.last_scores:
            return "no scores yet"
        parts = []
        for i, s in enumerate(self.last_scores):
            parts.append(
                f"B{chr(65+i)}:"
                f"d={s['s_dist']:.2f} "
                f"sf={s['s_safety']:.2f} "
                f"sa={s['s_satiation']:.2f} "
                f"→{s['total']:.2f}"
            )
        return "  |  ".join(parts)
