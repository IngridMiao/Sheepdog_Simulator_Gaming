"""
decision/adaptive_utility.py

自適應效用調整器（Adaptive Utility Weight Adjuster）
────────────────────────────────────────────────────────────────────────────
包裝 UtilitySystem，在以下事件後動態調整權重：

  事件 A：羊被狗追（觸發 FLEE）
      → w_safety   += SAFETY_BOOST     （更重視安全）
      → w_dist     -= SAFETY_BOOST/2   （少追近的）
      → w_satiation-= SAFETY_BOOST/2   （少挑剔飽腹感）

  事件 B：羊長時間沒吃到東西（hunger_timer 超過 HUNGER_TIMEOUT）
      → w_satiation -= HUNGER_DECAY    （飢餓讓羊不那麼挑剔）
      → w_dist      += HUNGER_DECAY/2  （更傾向選近的）
      → w_safety    += HUNGER_DECAY/2  （還是不想送死）

所有調整都有 clamp，保證三個權重總和 ≈ 1.0（每次調整後正規化）。

「adaptation」的語意
──────────────────────
- 被追越多 → 優先逃到安全的草叢
- 越久沒吃 → 開始降低飽腹感門檻（肚子餓了就沒那麼挑）
這兩個效果可以同時累積，產生觀察上有意義的行為變化。
"""


class AdaptiveUtility:

    # 權重調整幅度
    SAFETY_BOOST   = 0.06   # 每次被追後 safety 提升多少
    HUNGER_DECAY   = 0.05   # 每次「太餓事件」後 satiation 重要性下降多少

    # 各權重的上下限（避免過度極端）
    W_MIN = 0.10
    W_MAX = 0.70

    # 「太久沒吃」的判定時間（秒）
    HUNGER_TIMEOUT = 12.0

    def __init__(self, utility_system):
        """
        Parameters
        ----------
        utility_system : UtilitySystem  被包裝的效用系統
        """
        self.utility = utility_system

        # 飢餓計時器（上次吃完草到現在的秒數）
        self._hunger_timer = 0.0

        # 統計（供實驗分析）
        self.flee_events   = 0
        self.hunger_events = 0
        self.weight_history: list[dict] = []  # 每次調整都記一筆

        # 記錄初始權重
        self._snapshot()

    # ------------------------------------------------------------------ #
    #  每幀更新（由 sheep.py 呼叫）                                        #
    # ------------------------------------------------------------------ #

    def update(self, dt, just_ate: bool):
        """
        Parameters
        ----------
        dt       : float  幀時間差（秒）
        just_ate : bool   這一幀羊剛吃完一棵草（由 sheep.py 傳入）
        """
        if just_ate:
            self._hunger_timer = 0.0
            return

        self._hunger_timer += dt
        if self._hunger_timer >= self.HUNGER_TIMEOUT:
            self._on_hunger_event()
            self._hunger_timer = 0.0   # 重置計時，避免連續觸發

    # ------------------------------------------------------------------ #
    #  事件回呼                                                            #
    # ------------------------------------------------------------------ #

    def on_flee(self):
        """羊進入 FLEE 狀態時由 sheep.py 呼叫"""
        self.flee_events += 1
        w = self.utility.get_weights()

        w["w_safety"]    += self.SAFETY_BOOST
        w["w_dist"]      -= self.SAFETY_BOOST / 2
        w["w_satiation"] -= self.SAFETY_BOOST / 2

        self._apply_and_normalize(w)

    def _on_hunger_event(self):
        """飢餓超時自動觸發"""
        self.hunger_events += 1
        w = self.utility.get_weights()

        w["w_satiation"] -= self.HUNGER_DECAY
        w["w_dist"]      += self.HUNGER_DECAY / 2
        w["w_safety"]    += self.HUNGER_DECAY / 2

        self._apply_and_normalize(w)

    # ------------------------------------------------------------------ #
    #  內部工具                                                            #
    # ------------------------------------------------------------------ #

    def _apply_and_normalize(self, w: dict):
        """Clamp 後正規化，讓三個權重總和 = 1.0，再寫回 UtilitySystem"""
        for k in w:
            w[k] = max(self.W_MIN, min(self.W_MAX, w[k]))

        total = w["w_dist"] + w["w_safety"] + w["w_satiation"]
        if total > 0:
            w = {k: v / total for k, v in w.items()}

        self.utility.set_weights(**w)
        self._snapshot()

    def _snapshot(self):
        """記錄當前權重快照"""
        self.weight_history.append(self.utility.get_weights().copy())

    # ------------------------------------------------------------------ #
    #  查詢                                                                #
    # ------------------------------------------------------------------ #

    def hunger_timer(self) -> float:
        return self._hunger_timer

    def debug_str(self) -> str:
        w = self.utility.get_weights()
        return (
            f"w_dist={w['w_dist']:.2f} "
            f"w_safe={w['w_safety']:.2f} "
            f"w_sat={w['w_satiation']:.2f}  |  "
            f"flee×{self.flee_events} "
            f"hunger×{self.hunger_events} "
            f"hunger_t={self._hunger_timer:.1f}s"
        )

    def weight_log_str(self) -> str:
        """回傳全部權重變化歷史（換行分隔）"""
        lines = []
        for i, snap in enumerate(self.weight_history):
            lines.append(
                f"#{i:02d}  d={snap['w_dist']:.2f} "
                f"sf={snap['w_safety']:.2f} "
                f"sa={snap['w_satiation']:.2f}"
            )
        return "\n".join(lines)
