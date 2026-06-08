"""
decision/rule_decision.py

規則式決策系統（Rule-based Decision System）
────────────────────────────────────────────────────────────────────────────
用一組優先級規則選出目標草叢，作為 utility system 的對照組。

規則（依優先級由高到低）：
  R1  危險規則：若狗距離 < DANGER_RADIUS
               → 選「距狗最遠」的草叢（逃跑導向）

  R2  飽腹規則：若所有剩餘草叢的飽腹感都 < SATIATION_LOW_THR
               → 選「飽腹感最高」的草叢（強制選最好的）

  R3  近距規則：若有草叢在 NEAR_DIST 範圍內
               → 選「最近且飽腹感 ≥ SATIATION_MIN_THR」的草叢

  R4  預設規則：選「飽腹感最高」的草叢（回退）
               若飽腹感全相同（例如全部首次）→ 選最近的

說明
────
- 規則是互斥的，命中第一條就停止。
- 每次選擇後記錄觸發了哪條規則（last_rule），供實驗分析。
- dog_pos 可以是 None（規則 R1 自動跳過）。
"""


class RuleDecision:

    DANGER_RADIUS      = 250.0   # px：狗進入此距離才觸發 R1
    NEAR_DIST          = 350.0   # px：草叢在此距離內才觸發 R3
    SATIATION_LOW_THR  = 0.40    # 所有草叢飽腹感都低於此才觸發 R2
    SATIATION_MIN_THR  = 0.20    # R3 需要草叢飽腹感至少這麼高

    def __init__(self, satiation_tracker):
        self.satiation = satiation_tracker
        self.last_rule = None          # 上次觸發的規則名稱
        self.last_rule_log: list[str] = []   # 歷史記錄

    # ------------------------------------------------------------------ #

    def choose(self, sheep_pos, remaining_bushes, dog_pos=None):
        """
        從 remaining_bushes 中依規則選出目標草叢。
        """
        if not remaining_bushes:
            return None
        if len(remaining_bushes) == 1:
            self.last_rule = "ONLY_ONE"
            return remaining_bushes[0]

        # ── R1：危險規則 ─────────────────────────────────────────────
        if dog_pos is not None:
            dog_dist = sheep_pos.distance_to(dog_pos)
            if dog_dist < self.DANGER_RADIUS:
                chosen = max(remaining_bushes,
                             key=lambda b: b.pos.distance_to(dog_pos))
                self._log("R1_DANGER")
                return chosen

        # ── R2：飽腹規則 ─────────────────────────────────────────────
        max_sat = max(self.satiation.satiation(b) for b in remaining_bushes)
        if max_sat < self.SATIATION_LOW_THR:
            chosen = max(remaining_bushes,
                         key=lambda b: self.satiation.satiation(b))
            self._log("R2_SATIATION_FORCE")
            return chosen

        # ── R3：近距規則 ─────────────────────────────────────────────
        near = [b for b in remaining_bushes
                if sheep_pos.distance_to(b.pos) < self.NEAR_DIST
                and self.satiation.satiation(b) >= self.SATIATION_MIN_THR]
        if near:
            chosen = min(near, key=lambda b: sheep_pos.distance_to(b.pos))
            self._log("R3_NEAR")
            return chosen

        # ── R4：預設規則（飽腹感最高；同值時選最近）────────────────
        best_sat = max(self.satiation.satiation(b) for b in remaining_bushes)
        candidates = [b for b in remaining_bushes
                      if self.satiation.satiation(b) >= best_sat - 0.01]
        chosen = min(candidates, key=lambda b: sheep_pos.distance_to(b.pos))
        self._log("R4_DEFAULT")
        return chosen

    # ------------------------------------------------------------------ #

    def _log(self, rule_name: str):
        self.last_rule = rule_name
        self.last_rule_log.append(rule_name)

    def rule_counts(self) -> dict:
        """回傳各規則觸發次數統計（供實驗分析）"""
        counts = {"R1_DANGER": 0, "R2_SATIATION_FORCE": 0,
                  "R3_NEAR": 0, "R4_DEFAULT": 0, "ONLY_ONE": 0}
        for r in self.last_rule_log:
            counts[r] = counts.get(r, 0) + 1
        return counts

    def reset_log(self):
        self.last_rule_log.clear()

    def debug_str(self) -> str:
        counts = self.rule_counts()
        total  = sum(counts.values())
        if total == 0:
            return "no decisions yet"
        return (f"Last: {self.last_rule}  |  "
                + "  ".join(f"{k}:{v}" for k, v in counts.items() if v > 0))
