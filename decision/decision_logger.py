"""
decision/decision_logger.py

決策記錄器（Decision Logger）
────────────────────────────────────────────────────────────────────────────
記錄每次草叢選擇事件，供實驗分析使用。

記錄內容：
  - 時間戳（遊戲內秒數）
  - 決策架構名稱（RANDOM / UTILITY / RULE / ADAPTIVE）
  - 被選中的草叢索引
  - 當時各草叢的飽腹感
  - utility system 的詳細分數（若有）
  - rule system 觸發的規則（若有）
  - 當時羊的位置、狗的位置

用法：
    logger = DecisionLogger()
    logger.log_choice(...)       # 每次選草叢時記錄
    logger.log_flee(...)         # 每次觸發 FLEE 時記錄
    logger.summary()             # 印出統計摘要
"""

import json
from dataclasses import dataclass, field, asdict
from typing import Optional


@dataclass
class ChoiceEvent:
    t            : float           # 遊戲時間（秒）
    arch         : str             # 決策架構
    chosen_idx   : int             # 被選草叢的索引（0~3）
    satiations   : list            # 各草叢當時的飽腹感 [float, ...]
    sheep_pos    : tuple           # (x, y)
    dog_pos      : Optional[tuple] # (x, y) 或 None
    rule_fired   : Optional[str]   # Rule 架構：觸發的規則名稱
    utility_scores: Optional[list] # Utility 架構：各草叢分數


@dataclass
class FleeEvent:
    t         : float
    sheep_pos : tuple
    dog_pos   : tuple
    flee_count: int   # 這是第幾次 flee


class DecisionLogger:

    def __init__(self):
        self.choices : list[ChoiceEvent] = []
        self.flees   : list[FleeEvent]   = []

    # ------------------------------------------------------------------ #

    def log_choice(self, t, arch, chosen_bush, all_bushes,
                   satiation_tracker, sheep_pos, dog_pos=None,
                   rule_fired=None, utility_scores=None):
        """
        Parameters
        ----------
        chosen_bush      : Bush  被選中的草叢物件
        all_bushes       : list[Bush]  所有草叢（含已吃完的，用於索引對齊）
        satiation_tracker: SatiationTracker
        """
        idx = all_bushes.index(chosen_bush) if chosen_bush in all_bushes else -1
        sats = [round(satiation_tracker.satiation(b), 3) for b in all_bushes]

        event = ChoiceEvent(
            t             = round(t, 2),
            arch          = arch,
            chosen_idx    = idx,
            satiations    = sats,
            sheep_pos     = (round(sheep_pos.x, 1), round(sheep_pos.y, 1)),
            dog_pos       = (round(dog_pos.x, 1), round(dog_pos.y, 1)) if dog_pos else None,
            rule_fired    = rule_fired,
            utility_scores= utility_scores,
        )
        self.choices.append(event)

    def log_flee(self, t, sheep_pos, dog_pos):
        event = FleeEvent(
            t          = round(t, 2),
            sheep_pos  = (round(sheep_pos.x, 1), round(sheep_pos.y, 1)),
            dog_pos    = (round(dog_pos.x, 1), round(dog_pos.y, 1)),
            flee_count = len(self.flees) + 1,
        )
        self.flees.append(event)

    # ------------------------------------------------------------------ #

    def reset(self):
        self.choices.clear()
        self.flees.clear()

    # ------------------------------------------------------------------ #

    def summary(self) -> str:
        lines = [
            f"=== DecisionLogger Summary ===",
            f"Total choices : {len(self.choices)}",
            f"Total flees   : {len(self.flees)}",
        ]

        if self.choices:
            # 各草叢被選中幾次
            counts = [0, 0, 0, 0]
            for c in self.choices:
                if 0 <= c.chosen_idx < 4:
                    counts[c.chosen_idx] += 1
            lines.append("Bush selection counts: " +
                         "  ".join(f"Bush{chr(65+i)}×{counts[i]}" for i in range(4)))

            # 飽腹感在被選中時的平均值
            sat_sums = [0.0]*4
            sat_cnt  = [0]*4
            for c in self.choices:
                for i, s in enumerate(c.satiations):
                    sat_sums[i] += s
                    sat_cnt[i]  += 1
            avgs = [sat_sums[i]/sat_cnt[i] if sat_cnt[i] else 0 for i in range(4)]
            lines.append("Avg satiation at choice time: " +
                         "  ".join(f"Bush{chr(65+i)}={avgs[i]:.2f}" for i in range(4)))

        if self.flees:
            avg_flee_t = sum(f.t for f in self.flees) / len(self.flees)
            lines.append(f"Average flee time: {avg_flee_t:.1f}s")

        return "\n".join(lines)

    def to_json(self) -> str:
        """序列化成 JSON 字串（可用於輸出到檔案）"""
        data = {
            "choices": [asdict(c) for c in self.choices],
            "flees"  : [asdict(f) for f in self.flees],
        }
        return json.dumps(data, indent=2)
