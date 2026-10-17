"""Permutation ABC solver restored from the original Path Planning Software.

See docs/architecture/ABC_INTEGRATION.md for provenance and compatibility fixes.
The legacy objective includes an XY-projection crossing penalty; it is not a
3D collision or robot-reachability check. NumPy is the only required dependency.
"""

import os
import random
from numbers import Integral

import numpy as np


def _positive_integer(name, value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 1:
        raise ValueError("{} must be a positive integer".format(name))
    return int(value)


def _optional_progressbar():
    # The progressbar2 distribution exposes the `progressbar` module.
    try:
        import progressbar
        return progressbar
    except ImportError:
        return None


def _optional_figure_class():
    try:
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
    except ImportError:
        return None

    def make_figure(**kwargs):
        figure = Figure(**kwargs)
        FigureCanvasAgg(figure)
        return figure

    return make_figure


def angle_between(v1: np.ndarray, v2: np.ndarray, eps=1e-9) -> float:
    # 计算两向量夹角（弧度），返回 [0, pi]
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < eps or n2 < eps:
        return 0.0
    cosang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.arccos(cosang))


def is_path_intersect(points, i, j, k, l):
    """
    按原版规则检查路径段(i,j)和(k,l)的 XY 投影是否相交。
    """

    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    if i == k or i == l or j == k or j == l:
        return False  # 相邻边不认为相交

    A = points[i]
    B = points[j]
    C = points[k]
    D = points[l]

    return intersect(A, B, C, D)


class TSPConfig:
    def __init__(
        self,
        points,                # np.ndarray, shape (N,3)
        FOOD_NUMBER=40,        # 种群规模（食物源数）
        LIMIT=150,             # trial 上限
        MAXIMUM_EVALUATION=50000,
        SHOW_PROGRESS=False,
        RANDOM_SEED=False,
        SEED=2024,
        CLOSED_TOUR=False,       # True=闭环（回到起点），False=开环
        start_index=None,        # 可选的固定起点索引（仅开环）
        end_index=None,          # 可选的固定终点索引（仅开环）
        # 拐点设置
        angle_threshold_deg=45.0,
        weights=(1/50, 0.2),  # 显著增加路径长度权重，同时增加折点数量权重
        # 可视化
        VISUALIZE=False,
        VIS_SAVE_DIR="viz_frames",
        VIS_INTERVAL_EVALS=2000,  # 每隔多少评估保存一次图
        PLOT_HISTORY=True
    ):
        self.points = np.asarray(points, dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 3
                or not len(self.points) or not np.all(np.isfinite(self.points))):
            raise ValueError("points must be a nonempty finite (N,3) array")
        self.N = self.points.shape[0]     # 城市数
        self.FOOD_NUMBER = _positive_integer("FOOD_NUMBER", FOOD_NUMBER)
        self.LIMIT = _positive_integer("LIMIT", LIMIT)
        self.MAXIMUM_EVALUATION = _positive_integer("MAXIMUM_EVALUATION", MAXIMUM_EVALUATION)
        if self.MAXIMUM_EVALUATION < self.FOOD_NUMBER:
            raise ValueError("MAXIMUM_EVALUATION must be at least FOOD_NUMBER")
        self.progressbar_module = _optional_progressbar() if SHOW_PROGRESS else None
        self.SHOW_PROGRESS = bool(SHOW_PROGRESS and self.progressbar_module is not None)
        self.RANDOM_SEED = RANDOM_SEED
        self.SEED = SEED
        self.CLOSED_TOUR = CLOSED_TOUR
        # 开环模式可以独立指定起点或终点，也可以都不固定
        self.start_index = start_index
        self.end_index = end_index
        for name, index in (("start_index", start_index), ("end_index", end_index)):
            if index is not None and (isinstance(index, bool)
                    or not isinstance(index, Integral) or not 0 <= index < self.N):
                raise ValueError("{} must be a valid point index".format(name))
        if CLOSED_TOUR and (start_index is not None or end_index is not None):
            raise ValueError("fixed endpoints require CLOSED_TOUR=False")
        if start_index is not None and start_index == end_index:
            raise ValueError("start_index and end_index must be distinct")

        self.angle_threshold_deg = float(angle_threshold_deg)
        self.angle_threshold_rad = np.deg2rad(self.angle_threshold_deg)
        if len(weights) != 2:
            raise ValueError("weights must contain length and corner weights")
        self.alpha = float(weights[0])
        self.beta = float(weights[1])
        if (not np.isfinite(self.angle_threshold_deg) or not 0 <= self.angle_threshold_deg <= 180
                or not np.all(np.isfinite([self.alpha, self.beta]))
                or min(self.alpha, self.beta) < 0 or self.alpha + self.beta == 0):
            raise ValueError("use a finite angle in [0,180] and nonnegative, nonzero weights")

        self.make_figure = _optional_figure_class() if VISUALIZE or PLOT_HISTORY else None
        self.VISUALIZE = bool(VISUALIZE and self.make_figure is not None)
        self.VIS_SAVE_DIR = VIS_SAVE_DIR
        self.VIS_INTERVAL_EVALS = _positive_integer("VIS_INTERVAL_EVALS", VIS_INTERVAL_EVALS)
        self.PLOT_HISTORY = bool(PLOT_HISTORY and self.make_figure is not None)

        if self.VISUALIZE or self.PLOT_HISTORY:
            os.makedirs(self.VIS_SAVE_DIR, exist_ok=True)

class ABC_TSP3D:

    def __init__(self, conf: TSPConfig):
        self.conf = conf

        # 起点、终点可独立固定；其余节点参与排列
        self.is_open = not self.conf.CLOSED_TOUR

        if self.conf.CLOSED_TOUR:
            # 闭环：排列所有节点
            self.var_indices = np.arange(self.conf.N, dtype=int)
        elif self.conf.start_index is not None or self.conf.end_index is not None:
            # 开环且指定端点：只排列未固定的城市
            all_idx = np.arange(self.conf.N, dtype=int)
            mask = np.ones(self.conf.N, dtype=bool)
            if self.conf.start_index is not None:
                mask[self.conf.start_index] = False
            if self.conf.end_index is not None:
                mask[self.conf.end_index] = False
            self.variable_nodes = all_idx[mask]  # 需要排列的中间节点
            self.var_count = len(self.variable_nodes)
            self.var_indices = np.arange(self.var_count, dtype=int)
        else:
            # 开环且不固定起点终点：排列所有节点
            self.var_indices = np.arange(self.conf.N, dtype=int)

        # 种群存储：每个个体是对 variable_nodes 的排列（开环）；闭环则对全体点排列
        self.FOOD_NUMBER = self.conf.FOOD_NUMBER
        self.foods = np.zeros((self.FOOD_NUMBER, len(self.var_indices)), dtype=int)

        # 目标值与适应度
        self.f = np.ones((self.FOOD_NUMBER), dtype=float) * np.inf    # 组合目标：alpha*length + beta*corners
        self.fitness = np.ones((self.FOOD_NUMBER), dtype=float) * np.finfo(float).eps
        self.trial = np.zeros((self.FOOD_NUMBER), dtype=int)
        self.prob = [0.0 for _ in range(self.conf.FOOD_NUMBER)]

        # 当前/全局最优
        self.solution = np.zeros((len(self.var_indices),), dtype=int)
        self.globalParams = None  # 保存的是“全路径排列”（包含起终点的完整序列）
        self.globalTime = 0
        self.evalCount = 0
        self.cycle = 0
        self.experimentID = 0
        self.globalOpts = list()  # 历史最优 objective
        self.globalLengths = list()
        self.globalCorners = list()
        self.has_intersection_penalty = 1.5  # 路径交叉惩罚因子

        if self.conf.SHOW_PROGRESS:
            # 适配不同版本的progressbar库
            try:
                # 尝试progressbar2的方式
                self.progressbar = self.conf.progressbar_module.ProgressBar(max_value=self.conf.MAXIMUM_EVALUATION)
                # progressbar2不需要单独调用start
            except TypeError:
                # 尝试旧版progressbar的方式
                progressbar = self.conf.progressbar_module
                self.progressbar = progressbar.ProgressBar(widgets=[progressbar.Percentage(), ' ', progressbar.Bar()],
                                                           maxval=self.conf.MAXIMUM_EVALUATION)
                # 旧版progressbar需要调用start
                self.progressbar.start()
        # Keep the original seeded sequences without resetting application RNGs.
        seed = None if self.conf.RANDOM_SEED else self.conf.SEED
        self.random = random.Random(seed)
        self.np_random = np.random.RandomState(seed)

        # 预计算距离矩阵（加速评估）
        self.dist = self._compute_distance_matrix(self.conf.points)
        self.globalOpt = np.inf
        self.last_viz_eval = -1

    @staticmethod
    def _compute_distance_matrix(points: np.ndarray):
        # 欧氏距离矩阵 (N,N)
        diff = points[:, None, :] - points[None, :, :]
        dist = np.sqrt(np.sum(diff * diff, axis=2))
        return dist

    # ---------- 路径组装 ----------
    def _assemble_full_tour(self, perm_var: np.ndarray) -> np.ndarray:
        if self.is_open:
            if self.conf.start_index is not None or self.conf.end_index is not None:
                # 开环且固定起点终点：start -> perm(variable_nodes) -> end
                full = [] if self.conf.start_index is None else [self.conf.start_index]
                full += list(self.variable_nodes[perm_var])
                if self.conf.end_index is not None:
                    full += [self.conf.end_index]
                return np.array(full, dtype=int)
            else:
                # 开环且不固定起点终点：直接返回排列的所有节点
                return np.array(perm_var, dtype=int)
        else:
            # 闭环：perm over all nodes
            return np.array(perm_var, dtype=int)

    # ---------- 目标构成 ----------
    def _tour_length(self, full_perm: np.ndarray) -> float:
        d = 0.0
        if self.is_open or not self.conf.CLOSED_TOUR:
            for i in range(len(full_perm) - 1):
                d += self.dist[full_perm[i], full_perm[i + 1]]
        else:
            # 闭环
            for i in range(len(full_perm) - 1):
                d += self.dist[full_perm[i], full_perm[i + 1]]
            d += self.dist[full_perm[-1], full_perm[0]]
        return d

    def _has_intersections(self, full_perm: np.ndarray) -> bool:
        """
        检查路径是否有交叉
        """
        if len(full_perm) < 4:
            return False

        # 检查相邻边之间是否有交叉
        for i in range(len(full_perm) - 1):
            for j in range(i + 2, len(full_perm) - 1):
                if i == 0 and j == len(full_perm) - 2 and self.is_open:
                    continue  # 开环情况下不检查首尾
                if is_path_intersect(self.conf.points, full_perm[i], full_perm[i + 1], full_perm[j], full_perm[j + 1]):
                    return True

        # 对于闭环情况，检查连接起点和终点的边
        if not self.is_open:
            for i in range(1, len(full_perm) - 2):
                if is_path_intersect(self.conf.points, full_perm[0], full_perm[-1], full_perm[i], full_perm[i + 1]):
                    return True

        return False

    def _corner_count(self, full_perm: np.ndarray) -> int:
        # 统计折点数：连续三点 A->B->C，角度 > 阈值，则 B 为折点
        pts = self.conf.points
        k = 0
        # 开环不包含两端；闭环则三元组环绕
        if self.is_open:
            # i from 1 to n-2: (i-1, i, i+1)
            for i in range(1, len(full_perm) - 1):
                A = pts[full_perm[i-1]]
                B = pts[full_perm[i]]
                C = pts[full_perm[i+1]]
                ang = angle_between(B - A, C - B)
                if ang > self.conf.angle_threshold_rad:
                    k += 1
        else:
            n = len(full_perm)
            for i in range(n):
                A = pts[full_perm[(i - 1) % n]]
                B = pts[full_perm[i]]
                C = pts[full_perm[(i + 1) % n]]
                ang = angle_between(B - A, C - B)
                if ang > self.conf.angle_threshold_rad:
                    k += 1
        return int(k)

    def _objective(self, full_perm: np.ndarray):
        L = self._tour_length(full_perm)
        K = self._corner_count(full_perm)
        # 检查路径是否有交叉，如果有则增加惩罚
        if self._has_intersections(full_perm):
            L *= self.has_intersection_penalty
        obj = self.conf.alpha * L + self.conf.beta * K
        return obj, L, K

    def calculate_function(self, perm_var: np.ndarray):
        # 兼容原接口，返回 [objective]
        if self.conf.SHOW_PROGRESS:
            self.progressbar.update(self.evalCount)
        full = self._assemble_full_tour(perm_var)
        val, L, K = self._objective(full)
        # 将最近一次完整评估的分量存一下，以备可视化/记录
        self._last_eval_L = L
        self._last_eval_K = K
        self._last_eval_full_perm = full
        return np.array([val], dtype=float)

    def calculate_fitness(self, fun: float):
        # fun 越小越好；映射到较大 fitness
        self.increase_eval()
        if fun >= 0:
            return 1.0 / (fun + 1.0)
        else:
            return 1.0 + abs(fun)

    def increase_eval(self):
        self.evalCount += 1

    def stopping_condition(self):
        status = bool(self.evalCount >= self.conf.MAXIMUM_EVALUATION)
        if self.conf.SHOW_PROGRESS:
            if status and hasattr(self, "progressbar"):
                # 检查progressbar的完成状态，兼容不同版本
                finished = False
                if hasattr(self.progressbar, '_finished'):
                    finished = self.progressbar._finished
                elif hasattr(self.progressbar, 'finished'):
                    finished = self.progressbar.finished

                if not finished:
                    try:
                        self.progressbar.finish()
                    except Exception:
                        # 忽略可能的错误
                        pass
        return status

    def memorize_best_source(self):
        for i in range(self.FOOD_NUMBER):
            if self.f[i] < self.globalOpt:
                self.globalOpt = float(self.f[i])
                # 保存完整路径
                best_full = self._assemble_full_tour(self.foods[i])
                self.globalParams = best_full
                # 记录对应的 L/K
                bestL, bestK = self._tour_length(best_full), self._corner_count(best_full)
                self._global_L = bestL
                self._global_K = bestK

    # ---------- 初始化 ----------
    def _random_perm_vars(self):
        perm = np.arange(len(self.var_indices), dtype=int)
        self.np_random.shuffle(perm)
        return perm

    def init(self, index):
        if not self.stopping_condition():
            self.foods[index] = self._random_perm_vars()
            self.solution = np.copy(self.foods[index])
            self.f[index] = self.calculate_function(self.solution)[0]
            self.fitness[index] = self.calculate_fitness(self.f[index])
            self.trial[index] = 0

    def initial(self):
        for i in range(self.FOOD_NUMBER):
            self.init(i)
        self.globalOpt = float(self.f[0])
        self.globalParams = self._assemble_full_tour(self.foods[0])
        self._global_L = self._tour_length(self.globalParams)
        self._global_K = self._corner_count(self.globalParams)

    # ---------- 邻域算子（对变量部分）  ----------
    def _swap(self, perm: np.ndarray):
        n = len(perm)
        i, j = self.np_random.choice(n, 2, replace=False)
        perm2 = perm.copy()
        perm2[i], perm2[j] = perm2[j], perm2[i]
        return perm2

    def _insert(self, perm: np.ndarray):
        n = len(perm)
        i, j = self.np_random.choice(n, 2, replace=False)
        perm2 = perm.copy().tolist()
        city = perm2.pop(i)
        perm2.insert(j, city)
        return np.array(perm2, dtype=int)

    def _two_opt(self, perm: np.ndarray):
        n = len(perm)
        i, j = sorted(self.np_random.choice(n, 2, replace=False))
        if i == j:
            return perm.copy()
        perm2 = perm.copy()
        perm2[i:j+1] = perm2[i:j+1][::-1]
        return perm2

    def _neighbor(self, perm: np.ndarray):
        # 随机选择一种邻域，以提升探索/开发平衡
        if len(perm) < 2:
            return perm.copy()
        r = self.random.random()
        if r < 1/3:
            return self._swap(perm)
        elif r < 2/3:
            return self._insert(perm)
        else:
            # 有一定概率使用2-opt操作以减少路径交叉
            if self.random.random() < 0.5 and len(perm) > 3:
                i, j = sorted(self.np_random.choice(len(perm), 2, replace=False))
                if i < j:
                    perm2 = perm.copy()
                    perm2[i:j + 1] = perm2[i:j + 1][::-1]
                    return perm2
            return self._two_opt(perm)

    # ---------- 雇佣蜂 ----------
    def send_employed_bees(self):
        i = 0
        while (i < self.FOOD_NUMBER) and (not self.stopping_condition()):
            sol = self._neighbor(self.foods[i])
            ObjValSol = self.calculate_function(sol)[0]
            FitnessSol = self.calculate_fitness(ObjValSol)
            # 贪婪选择
            if FitnessSol > self.fitness[i]:
                self.trial[i] = 0
                self.foods[i] = sol
                self.f[i] = ObjValSol
                self.fitness[i] = FitnessSol
            else:
                self.trial[i] += 1
            i += 1

    # ---------- 概率 ----------
    def calculate_probabilities(self):
        maxfit = float(np.max(self.fitness))
        if maxfit == 0:
            self.prob = [1.0 / self.FOOD_NUMBER] * self.FOOD_NUMBER
        else:
            for i in range(self.FOOD_NUMBER):
                self.prob[i] = 0.9 * (self.fitness[i] / maxfit) + 0.1

    # ---------- 观察蜂 ----------
    def send_onlooker_bees(self):
        i = 0
        t = 0
        while (t < self.FOOD_NUMBER) and (not self.stopping_condition()):
            r = self.random.random()
            if r < self.prob[i]:
                t += 1
                sol = self._neighbor(self.foods[i])
                ObjValSol = self.calculate_function(sol)[0]
                FitnessSol = self.calculate_fitness(ObjValSol)
                if FitnessSol > self.fitness[i]:
                    self.trial[i] = 0
                    self.foods[i] = sol
                    self.f[i] = ObjValSol
                    self.fitness[i] = FitnessSol
                else:
                    self.trial[i] += 1
            i = (i + 1) % self.FOOD_NUMBER

    # ---------- 侦察蜂 ----------
    def send_scout_bees(self):
        if np.max(self.trial) >= self.conf.LIMIT:
            idx = int(np.argmax(self.trial))
            self.init(idx)

    def increase_cycle(self):
        self.globalOpts.append(self.globalOpt)
        self.globalLengths.append(self._global_L)
        self.globalCorners.append(self._global_K)
        self.cycle += 1

    def setExperimentID(self, run, t):
        self.experimentID = f"{t}-{run}"

    # ---------- 可视化 ----------

    def _maybe_visualize(self, force=False):
        if not self.conf.VISUALIZE:
            return
        if (not force) and (self.evalCount - self.last_viz_eval < self.conf.VIS_INTERVAL_EVALS):
            return
        self.last_viz_eval = self.evalCount

        fig = self.conf.make_figure(figsize=(7, 6))
        ax = fig.add_subplot(111, projection='3d')
        pts = self.conf.points
        tour = self.globalParams

        # 绘制点
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='gray', s=30, alpha=0.8)

        # 标注起终点（开环）
        if self.is_open:
            s, e = tour[0], tour[-1]
            ax.scatter(pts[s, 0], pts[s, 1], pts[s, 2], c='green', s=80, label='Start')
            ax.scatter(pts[e, 0], pts[e, 1], pts[e, 2], c='red', s=80, label='End')

        # 绘制路径
        xs, ys, zs = pts[tour, 0], pts[tour, 1], pts[tour, 2]
        ax.plot(xs, ys, zs, c='blue', linewidth=2, label='Best Path')
        if not self.is_open and self.conf.CLOSED_TOUR:
            # 闭环回到起点
            ax.plot([xs[-1], xs[0]], [ys[-1], ys[0]], [zs[-1], zs[0]], c='blue', linewidth=2)

        # 标出当前折点
        corners_idx = []
        if len(tour) >= 3:
            if self.is_open:
                rng = range(1, len(tour) - 1)
            else:
                rng = range(0, len(tour))
            for i in rng:
                i_prev = i - 1 if self.is_open else (i - 1) % len(tour)
                i_next = i + 1 if self.is_open else (i + 1) % len(tour)
                A = pts[tour[i_prev]]
                B = pts[tour[i]]
                C = pts[tour[i_next]]
                ang = angle_between(B - A, C - B)
                if ang > self.conf.angle_threshold_rad:
                    corners_idx.append(i)
            if corners_idx:
                idx = np.array(corners_idx, dtype=int)
                ax.scatter(xs[idx], ys[idx], zs[idx], c='orange', s=70, label='Corners')

        ax.set_title(f"Eval {self.evalCount} | Obj={self.globalOpt:.3f} | L={self._global_L:.3f} | K={self._global_K}")
        ax.legend(loc='best')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        fig.tight_layout()

        out_path = os.path.join(self.conf.VIS_SAVE_DIR, f"frame_eval_{self.evalCount:07d}.png")
        fig.savefig(out_path, dpi=150)

    def _plot_history(self, save_path="history.png"):
        if not self.conf.PLOT_HISTORY:
            return
        fig = self.conf.make_figure(figsize=(7, 4))
        ax1 = fig.add_subplot(111)
        ax1.plot(self.globalOpts, label='Objective', color='tab:blue')
        ax1.set_xlabel('Cycle')
        ax1.set_ylabel('Objective', color='tab:blue')
        ax1.tick_params(axis='y', labelcolor='tab:blue')
        ax2 = ax1.twinx()
        ax2.plot(self.globalLengths, label='Length', color='tab:green', alpha=0.7)
        ax2.plot(self.globalCorners, label='Corners', color='tab:red', alpha=0.7)
        ax2.set_ylabel('Length / Corners', color='tab:red')
        ax2.tick_params(axis='y', labelcolor='tab:red')
        lines, labels = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines + lines2, labels + labels2, loc='best')
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)

    # ---------- 运行主循环 ----------
    def run(self):
        self.initial()
        self.memorize_best_source()
        self._maybe_visualize(force=True)

        while not self.stopping_condition() and len(self.var_indices) >= 2:
            self.send_employed_bees()
            self.calculate_probabilities()
            self.send_onlooker_bees()
            self.memorize_best_source()
            self.send_scout_bees()
            # A scout can improve the incumbent on the last allowed evaluation.
            self.memorize_best_source()
            self.increase_cycle()
            self._maybe_visualize()

        # 结束后再画一次
        self._maybe_visualize(force=True)
        # 输出历史曲线
        if self.conf.PLOT_HISTORY:
            self._plot_history(save_path=os.path.join(self.conf.VIS_SAVE_DIR, "history.png"))

        return {
            "best_objective": self.globalOpt,
            "best_length": self._global_L,
            "best_corners": self._global_K,
            "best_tour": self.globalParams.copy(),  # 完整路径（包含起止点）
            "history_objective": self.globalOpts,
            "history_length": self.globalLengths,
            "history_corners": self.globalCorners
        }
