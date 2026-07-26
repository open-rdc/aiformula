import numpy as np

NUM_BINS = 5


class BinWeighter:
    """舵角の大きさ |δ| をビンに分け、ビンごとの出現頻度から損失の重みを決める。

    重みは w_k ∝ n_k^(-beta)。beta=0 で全ビン 1.0（重み付けなしと数値的に一致）、
    beta=1 で各ビンの損失寄与が均等になる。データセット全体での平均重みが 1 に
    なるよう正規化するため、beta を変えても損失のスケールは保たれる。

    flip 拡張後の train targets に対して fit すること。val にも同じ重み関数を
    適用する（val 自身の分布で fit すると run 間で比較できない指標になる）。
    """

    def __init__(self, edges: np.ndarray, weights: np.ndarray):
        self.edges = np.asarray(edges, dtype=np.float64)
        self.weights = np.asarray(weights, dtype=np.float64)

    @classmethod
    def fit(cls, targets: np.ndarray, beta: float) -> 'BinWeighter':
        edges = np.linspace(0.0, 1.0, NUM_BINS + 1)
        blank = cls(edges, np.ones(NUM_BINS))
        counts = np.bincount(blank.bin_indices(targets), minlength=NUM_BINS).astype(float)

        weights = np.zeros(NUM_BINS, dtype=np.float64)
        filled = counts > 0
        weights[filled] = np.power(counts[filled], -float(beta))
        total = float((counts * weights).sum())
        if total > 0:
            weights *= counts.sum() / total
        return cls(edges, weights)

    def bin_indices(self, targets: np.ndarray) -> np.ndarray:
        magnitude = np.clip(np.abs(np.asarray(targets)[:, 0]), 0.0, 1.0)
        return np.clip(np.digitize(magnitude, self.edges[1:-1]), 0, NUM_BINS - 1)

    def weights_for(self, targets: np.ndarray) -> np.ndarray:
        return self.weights[self.bin_indices(targets)].astype(np.float32)
