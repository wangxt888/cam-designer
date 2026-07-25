import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QTextEdit, QFileDialog, QMessageBox, QFrame,
                             QGridLayout, QGroupBox, QSplitter,QSpinBox, QComboBox, QLineEdit )# 添加这行
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from scipy.interpolate import CubicSpline
from scipy.interpolate import make_interp_spline
import scipy.interpolate as spl

import time
import warnings
from math import comb
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtGui import QPixmap, QIcon, QColor, QLinearGradient, QBrush, QPalette, QPainter
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
plt.rcParams['font.sans-serif'] = ['SimHei']  # 显示中文
plt.rcParams['axes.unicode_minus'] = False
# 忽略所有警告
warnings.filterwarnings("ignore")


class BezierCurve1D:
    def __init__(self, control_points_1d):
        """传入单轴（X或Y）的控制点"""
        self.P = control_points_1d
        self.n = len(self.P) - 1  # n次贝塞尔（41个点就是40次）

    def __call__(self, u, nu=0):
        """
        计算曲线上的点或导数
        u: 0到1之间的参数化数组
        nu: 0为坐标，1为一阶导，2为二阶导
        """
        n = self.n
        P = self.P
        result = np.zeros_like(u)

        if nu == 0:
            # 坐标计算公式
            for i in range(n + 1):
                bernstein = comb(n, i) * (u ** i) * ((1 - u) ** (n - i))
                result += bernstein * P[i]
        elif nu == 1:
            # 一阶导数
            for i in range(n):
                bernstein = comb(n - 1, i) * (u ** i) * ((1 - u) ** (n - 1 - i))
                result += n * bernstein * (P[i + 1] - P[i])
        elif nu == 2:
            # 二阶导数
            for i in range(n - 1):
                bernstein = comb(n - 2, i) * (u ** i) * ((1 - u) ** (n - 2 - i))
                result += n * (n - 1) * bernstein * (P[i + 2] - 2 * P[i + 1] + P[i])

        return result
class PathPlannerThread(QThread):
    """路径规划线程，用于在后台运行路径规划算法"""
    update_progress = pyqtSignal(str, float, float)  # 信号：更新进度 (消息, 适应度, 最大曲率变化率)
    update_plot = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                             np.ndarray)  # 信号：更新绘图 (最优控制点, 最优路径, 当前控制点, 当前路径, 打卡点)
    update_curvature = pyqtSignal(np.ndarray, np.ndarray)  # 信号：更新曲率图 (距离, 曲率)
    finished = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)  # 信号：完成规划 (点, 曲率, 曲率半径, 距离, 控制点)
    error_occurred = pyqtSignal(str)  # 信号：发生错误
    def __init__(self, waypoints, max_deviation=80.0,
                 cam_m=30.0, cam_E=20.0, cam_L=107.0, cam_e=73.02, cam_leftorright=1):
        super().__init__()
        self.waypoints = waypoints
        self.max_deviation = max_deviation
        # 凸轮参数（用于适应度中直接评估凸轮光滑度）
        self.cam_m = cam_m
        self.cam_E = cam_E
        self.cam_L = cam_L
        self.cam_e = cam_e
        self.cam_leftorright = cam_leftorright
        self.running = True
    def run(self):
        """运行路径规划算法"""
        try:
            # 创建初始控制点
            control_points = self._create_control_points()
            self.update_progress.emit("初始化控制点完成", 0, 0)
            self.update_plot.emit(control_points, np.array([]), control_points, np.array([]), self.waypoints)
            # 粒子群优化参数
            num_particles = 50
            max_iterations = 2000
            w_start, w_end = 0.9, 0.3  # 惯性权重线性衰减
            c1 = 2.0  # 个体学习因子
            c2 = 2.0  # 社会学习因子
            # 初始化粒子群
            particles = []
            velocities = []
            personal_best = []
            personal_best_fitness = []
            best_fitness = float('inf')
            best_control_points = control_points.copy()
            for i in range(num_particles):
                particle = control_points.copy()
                perturbation = np.random.normal(0, 15, particle.shape)
                particle += perturbation
                velocity = np.random.normal(0, 5, particle.shape)
                particles.append(particle)
                velocities.append(velocity)
                personal_best.append(particle.copy())
                fitness, _, max_curvature_change = self._calculate_fitness(particle)
                personal_best_fitness.append(fitness)
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_control_points = particle.copy()
            self.update_progress.emit("粒子群初始化完成", best_fitness, max_curvature_change)
            # 生成最优路径用于可视化
            spline_x_best, spline_y_best, _ = self._create_spline(best_control_points)
            u_new = np.linspace(0, 1, 500)
            x_best = spline_x_best(u_new)
            y_best = spline_y_best(u_new)
            best_path_points = np.column_stack((x_best, y_best))
            # 生成当前粒子路径用于可视化
            spline_x_current, spline_y_current, _ = self._create_spline(particles[0])
            x_current = spline_x_current(u_new)
            y_current = spline_y_current(u_new)
            current_path_points = np.column_stack((x_current, y_current))
            self.update_plot.emit(best_control_points, best_path_points, particles[0], current_path_points,
                                  self.waypoints)

            # 比如允许控制点最多只能偏离它的初始位置 100mm
            max_search_radius = 1000.0
            # 限制每次飞行的最大速度，防止一步跨太大
            max_velocity = 100.0
            # 停滞检测
            last_best_fitness = best_fitness
            no_improve_count = 0

            # 优化迭代
            for iteration in range(max_iterations):
                if not self.running:
                    break
                # 惯性权重线性衰减：前期大范围探索，后期精细收敛
                w = w_start - (w_start - w_end) * iteration / max_iterations
                for i in range(num_particles):
                    # 更新粒子速度
                    r1 = np.random.random(particles[i].shape)
                    r2 = np.random.random(particles[i].shape)
                    velocities[i] = (w * velocities[i] +
                                     c1 * r1 * (personal_best[i] - particles[i]) +
                                     c2 * r2 * (best_control_points - particles[i]))
                    # 防止速度累积过大导致粒子飞出###########################################
                    velocities[i] = np.clip(velocities[i], -max_velocity, max_velocity)
                    # 更新粒子位置
                    particles[i] += velocities[i]
                    particles[i] = np.clip(
                        particles[i],
                        control_points - max_search_radius,  # 下界：初始点 - 半径
                        control_points + max_search_radius  # 上界：初始点 + 半径
                    )
                    # 计算新适应度
                    fitness, _, max_curvature_change = self._calculate_fitness(particles[i])
                    # 更新个体最优
                    if fitness < personal_best_fitness[i]:
                        personal_best_fitness[i] = fitness
                        personal_best[i] = particles[i].copy()
                    # 更新全局最优
                    if fitness < best_fitness:
                        best_fitness = fitness
                        best_control_points = particles[i].copy()
                # ======== 停滞检测与变异重启 ========
                if best_fitness < last_best_fitness - 0.01:
                    last_best_fitness = best_fitness
                    no_improve_count = 0
                else:
                    no_improve_count += 1

                # 连续 80 代没进步 → 重置最差的一半粒子
                if no_improve_count > 80:
                    self.update_progress.emit(f"迭代{iteration}: 触发变异重启", best_fitness,
                                              max_curvature_change)
                    for j in range(num_particles // 2, num_particles):
                        particles[j] = best_control_points.copy() + np.random.normal(0, 30, particles[j].shape)
                        velocities[j] = np.random.normal(0, 8, particles[j].shape)
                        personal_best[j] = particles[j].copy()
                        pf, _, _ = self._calculate_fitness(particles[j])
                        personal_best_fitness[j] = pf
                    no_improve_count = 0
                # ======================================
                # 每10次迭代更新一次进度
                if iteration % 5 == 0:
                    self.update_progress.emit(f"迭代 {iteration + 1}/{max_iterations}", best_fitness,
                                              max_curvature_change)
                    # 生成最优路径用于可视化
                    spline_x_best, spline_y_best, _ = self._create_spline(best_control_points)
                    u_new = np.linspace(0, 1, 500)
                    x_best = spline_x_best(u_new)
                    y_best = spline_y_best(u_new)
                    best_path_points = np.column_stack((x_best, y_best))
                    # 生成当前粒子路径用于可视化
                    spline_x_current, spline_y_current, _ = self._create_spline(particles[0])
                    x_current = spline_x_current(u_new)
                    y_current = spline_y_current(u_new)
                    current_path_points = np.column_stack((x_current, y_current))
                    self.update_plot.emit(best_control_points, best_path_points, particles[0], current_path_points,
                                          self.waypoints)
                # 每50次迭代更新一次曲率图
                if iteration % 50 == 0:
                    # 计算当前最优曲率
                    spline_x, spline_y, u = self._create_spline(best_control_points)
                    u_new = np.linspace(0, 1, 500)
                    curvature = self._calculate_signed_curvature(spline_x, spline_y, u_new)
                    # 计算累积距离
                    x = spline_x(u_new)
                    y = spline_y(u_new)
                    dx = np.diff(x, prepend=x[0])
                    dy = np.diff(y, prepend=y[0])
                    distances = np.cumsum(np.sqrt(dx ** 2 + dy ** 2))
                    normalized_distances = distances / distances[-1]
                    self.update_curvature.emit(normalized_distances, curvature)
            # 最终路径
            spline_x, spline_y, u = self._create_spline(best_control_points)
            u_new = np.linspace(0, 1, 100000)
            x = spline_x(u_new)
            y = spline_y(u_new)
            points = np.column_stack((x, y))
            # 计算带符号曲率
            curvature = self._calculate_signed_curvature(spline_x, spline_y, u_new)
            # 计算曲率半径
            curvature_radius = self._calculate_curvature_radius(curvature)
            # 计算累积距离
            dx = np.diff(x, prepend=x[0])
            dy = np.diff(y, prepend=y[0])
            distances = np.cumsum(np.sqrt(dx ** 2 + dy ** 2))
            normalized_distances = distances / distances[-1]
            self.finished.emit(points, curvature, curvature_radius, normalized_distances, best_control_points)
            self.update_progress.emit("路径规划完成!", best_fitness, max_curvature_change)
        except Exception as e:
            self.error_occurred.emit(f"路径规划错误: {str(e)}")
    def stop(self):
        """停止路径规划"""
        self.running = False
    def _create_control_points(self):
        """创建控制点序列"""
        control_points = []
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            # 添加起点
            control_points.append(start)
            # 在中间添加2个控制点
            for j in range(1, 3):
                t = j / 3
                point = start + t * (end - start)
                offset = np.random.normal(0, 3, 2)
                control_points.append(point + offset)
        # 添加最后一个点
        control_points.append(self.waypoints[-1])
        return np.array(control_points)
    # def _create_spline(self, control_points):#######3ci
    #     """创建三次样条曲线"""
    #     distances = np.zeros(len(control_points))
    #     for i in range(1, len(control_points)):
    #         dx = control_points[i, 0] - control_points[i - 1, 0]
    #         dy = control_points[i, 1] - control_points[i - 1, 1]
    #         distances[i] = distances[i - 1] + np.sqrt(dx ** 2 + dy ** 2)
    #     u = distances / distances[-1]
    #     spline_x = CubicSpline(u, control_points[:, 0])
    #     spline_y = CubicSpline(u, control_points[:, 1])
    #     return spline_x, spline_y, u
    # def _create_control_points(self):
    #     """创建11个控制点（为了生成40次贝塞尔曲线）"""
    #     n_points = 41  # 40次需要41个点
    #     control_points = np.zeros((n_points, 2))
    #
    #     start = self.waypoints[0]
    #     end = self.waypoints[-1]
    #
    #     # 在起点和终点之间线性插值生成41个初始点
    #     for i in range(n_points):
    #         t = i / (n_points - 1)
    #         control_points[i] = start + t * (end - start)
    #
    #     return control_points
    #
    # def _create_spline(self, control_points):
    #     """使用30次贝塞尔曲线替换原来的样条曲线"""
    #     # u 的范围始终是 0 到 1
    #     # 在贝塞尔曲线中，不需要像样条那样计算累积距离作为节点向量
    #     u = np.linspace(0, 1, 100)
    #
    #     spline_x = BezierCurve1D(control_points[:, 0])
    #     spline_y = BezierCurve1D(control_points[:, 1])
    #
    #     return spline_x, spline_y, u
    def _create_spline(self, control_points):
        """使用带有平滑因子的 B样条逼近，彻底消除曲率尖刺"""
        # 1. 计算控制点的累积距离
        distances = np.zeros(len(control_points))
        for i in range(1, len(control_points)):
            dx = control_points[i, 0] - control_points[i - 1, 0]
            dy = control_points[i, 1] - control_points[i - 1, 1]
            distances[i] = distances[i - 1] + np.sqrt(dx ** 2 + dy ** 2)

        u = distances / distances[-1]

        # SciPy 仅支持 1~5 次样条；点数较少时自动降低阶数。
        spline_degree = min(5, len(control_points) - 1)
        smoothness_factor = 0.5  # 微量平滑（每点~0.02mm²），控制点移动能显著影响曲线

        # splrep 会自动生成平滑的 B样条
        tck_x = spl.splrep(u, control_points[:, 0], k=spline_degree, s=smoothness_factor)
        tck_y = spl.splrep(u, control_points[:, 1], k=spline_degree, s=smoothness_factor)

        # 包装成与原来兼容的调用方式
        def spline_x(u_eval, nu=0):
            return spl.splev(u_eval, tck_x, der=nu)

        def spline_y(u_eval, nu=0):
            return spl.splev(u_eval, tck_y, der=nu)

        return spline_x, spline_y, u

    def _calculate_signed_curvature(self, spline_x, spline_y, u):
        """计算带符号的曲率"""
        dx_du = spline_x(u, 1)
        dy_du = spline_y(u, 1)
        d2x_du2 = spline_x(u, 2)
        d2y_du2 = spline_y(u, 2)
        numerator = dx_du * d2y_du2 - dy_du * d2x_du2
        denominator = (dx_du ** 2 + dy_du ** 2) ** 1.5
        curvature = np.zeros_like(denominator)
        valid_indices = denominator > 1e-8
        curvature[valid_indices] = numerator[valid_indices] / denominator[valid_indices]
        curvature[~valid_indices] = 0.0
        return curvature
    def _calculate_curvature_radius(self, curvature):
        """计算曲率半径"""
        radius = np.full_like(curvature, 1e6, dtype=float)
        valid = np.abs(curvature) > 1e-8
        radius[valid] = 1.0 / curvature[valid]
        return radius
    def _calculate_fitness(self, control_points):
        '''计算适应度：凸轮极径光滑度 + 打卡点偏差'''
        spline_x, spline_y, u = self._create_spline(control_points)
        u_eval = np.linspace(0, 1, 2000)
        x = spline_x(u_eval)
        y = spline_y(u_eval)
        points = np.column_stack((x, y))

        # === 1. 计算曲率 & 曲率半径 ===
        curvature = self._calculate_signed_curvature(spline_x, spline_y, u_eval)
        R = np.full_like(curvature, 1e6)                # 直线 → 极大半径
        valid = np.abs(curvature) > 1e-8
        R[valid] = 1.0 / curvature[valid]

        # === 2. 计算凸轮极径 rou = e - leftorright * E * L / (R - m) ===
        denom = R - self.cam_m                          # 分母 R - m
        n_singularity = np.sum(np.abs(denom) < 10.0)
        if n_singularity > 0:
            return 1e12 * n_singularity, 999.0, 999.0

        rou = self.cam_e - self.cam_leftorright * self.cam_E * self.cam_L / denom
        rou = np.clip(rou, self.cam_e - 100, self.cam_e + 100)

        # === 3. 凸轮光滑度代价 ===
        drou  = np.diff(rou)
        d2rou = np.diff(rou, n=2)
        cost_jerk   = np.mean(d2rou ** 2)
        cost_smooth = np.mean(drou ** 2)

        # === 4. 打卡点偏差 ===
        total_deviation = 0.0
        max_deviation = 0.0
        for waypoint in self.waypoints:
            dists = np.linalg.norm(points - waypoint, axis=1)
            min_dist = np.min(dists)
            total_deviation += min_dist
            if min_dist > max_deviation:
                max_deviation = min_dist
        avg_deviation = total_deviation / len(self.waypoints)

        # === 5. 组装适应度 ===
        fitness = (cost_jerk   * 1e6 * 5.0 +
                   cost_smooth * 1e2 * 2.0 +
                   avg_deviation     * 1.0)

        if max_deviation > self.max_deviation:
            fitness += 1e6 * (max_deviation - self.max_deviation)

        curvature_diff = np.diff(curvature)
        max_curvature_change = np.max(np.abs(curvature_diff))

        return fitness, max_deviation, max_curvature_change
class PathPlotWidget(QWidget):
    """路径绘图控件（包含路径图和曲率图）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        # 创建两个子图：上面是路径图，下面是曲率图
        self.ax_path = self.figure.add_subplot(211)
        self.ax_curv = self.figure.add_subplot(212)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.best_control_points = None
        self.best_path_points = None
        self.current_control_points = None
        self.current_path_points = None
        self.waypoints = None
        self.curvatures = None
        self.distances = None
    def update_plot(self, best_control_points, best_path_points, current_control_points, current_path_points,
                    waypoints):
        """更新路径图"""
        self.best_control_points = best_control_points
        self.best_path_points = best_path_points
        self.current_control_points = current_control_points
        self.current_path_points = current_path_points
        self.waypoints = waypoints
        self.draw_path_plot()
    def draw_path_plot(self):
        """绘制路径图"""
        self.ax_path.clear()
        if self.waypoints is not None:
            self.ax_path.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'ro', markersize=5, label='打卡点')
        if self.best_control_points is not None:
            self.ax_path.plot(self.best_control_points[:, 0], self.best_control_points[:, 1], 'go--', markersize=4,
                              linewidth=1, alpha=0.6, label='最优控制点')
        if self.current_control_points is not None:
            self.ax_path.plot(self.current_control_points[:, 0], self.current_control_points[:, 1], 'bo--',
                              markersize=4, linewidth=1, alpha=0.3, label='当前控制点')
        if self.best_path_points is not None and len(self.best_path_points) > 0:
            self.ax_path.plot(self.best_path_points[:, 0], self.best_path_points[:, 1], 'g-', linewidth=2.0,
                              label='最优路径')
        if self.current_path_points is not None and len(self.current_path_points) > 0:
            self.ax_path.plot(self.current_path_points[:, 0], self.current_path_points[:, 1], 'b-', linewidth=1.0,
                              alpha=0.5, label='当前路径')
        self.ax_path.set_title('AI路径优化')
        self.ax_path.set_xlabel('X (mm)')
        self.ax_path.set_ylabel('Y (mm)')
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.axis('equal')
        self.ax_path.legend()
        self.canvas.draw()
    def update_curvature_plot(self, distances, curvatures):
        """更新曲率图"""
        self.curvatures = curvatures
        self.distances = distances
        self.draw_curvature_plot()
    def draw_curvature_plot(self):
        """绘制曲率图"""
        self.ax_curv.clear()
        if self.distances is not None and self.curvatures is not None:
            self.ax_curv.plot(self.distances, self.curvatures, 'g-', linewidth=1.5)
            self.ax_curv.set_title('带符号曲率')
            self.ax_curv.set_xlabel('归一化距离')
            self.ax_curv.set_ylabel('曲率')
            self.ax_curv.grid(True, alpha=0.3)
            self.ax_curv.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            # 标记正负曲率区域
            self.ax_curv.fill_between(self.distances, self.curvatures, where=self.curvatures > 0, color='green',
                                      alpha=0.2, label='正曲率')
            self.ax_curv.fill_between(self.distances, self.curvatures, where=self.curvatures < 0, color='red',
                                      alpha=0.2, label='负曲率')
            self.ax_curv.legend()
        self.canvas.draw()
class PathDesignWindow(QWidget):
    """路径AI设计窗口"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("路径AI设计")
        self.setGeometry(100, 100, 1200, 800)
        self.waypoints = None
        self.planner_thread = None
        self.path_points = None
        self.curvatures = None
        self.curvature_radii = None
        self.distances = None
        self.control_points = None
        self.init_ui()
    def init_ui(self):
        """初始化UI"""
        main_layout = QHBoxLayout()
        # 左侧面板
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_layout = QVBoxLayout()
        # 导入按钮
        self.import_button = QPushButton("导入打卡点数据")
        self.import_button.clicked.connect(self.import_waypoints)
        left_layout.addWidget(self.import_button)
        # 运行按钮
        self.run_button = QPushButton("运行AI路径规划")
        self.run_button.clicked.connect(self.run_path_planning)
        self.run_button.setEnabled(False)
        left_layout.addWidget(self.run_button)
        # 停止按钮
        self.stop_button = QPushButton("停止规划")
        self.stop_button.clicked.connect(self.stop_path_planning)
        self.stop_button.setEnabled(False)
        left_layout.addWidget(self.stop_button)
        # 保存按钮
        self.save_button = QPushButton("保存路径数据")
        self.save_button.clicked.connect(self.save_path_data)
        self.save_button.setEnabled(False)
        left_layout.addWidget(self.save_button)
        # 信息显示
        info_group = QGroupBox("状态信息")
        info_layout = QVBoxLayout()
        self.info_label = QLabel("状态: 等待导入数据")
        info_layout.addWidget(self.info_label)
        self.fitness_label = QLabel("当前适应度: -")
        info_layout.addWidget(self.fitness_label)
        self.curvature_label = QLabel("最大曲率变化率: -")
        info_layout.addWidget(self.curvature_label)
        info_group.setLayout(info_layout)
        left_layout.addWidget(info_group)
        # 日志文本框
        log_group = QGroupBox("操作日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.append("导入格式说明:")
        self.log_text.append("1. 文件应为文本文件(.txt)")
        self.log_text.append("2. 每行包含一个打卡点的X和Y坐标")
        self.log_text.append("3. 坐标值以空格或逗号分隔")
        self.log_text.append("示例:")
        self.log_text.append("5588 713")
        self.log_text.append("4463 375")
        self.log_text.append("2925 825")
        self.log_text.append("...")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)
        left_panel.setLayout(left_layout)
        # 右侧绘图区域
        self.plot_widget = PathPlotWidget()
        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(self.plot_widget, 2)
        self.setLayout(main_layout)
    def import_waypoints(self):
        """导入打卡点数据"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开打卡点数据", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        if not file_path:
            return
        try:
            # 读取文件
            with open(file_path, 'r') as f:
                lines = f.readlines()
            # 解析数据
            waypoints = []
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                # 尝试空格分隔
                if ' ' in line:
                    parts = line.split()
                # 尝试逗号分隔
                elif ',' in line:
                    parts = line.split(',')
                else:
                    continue
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        waypoints.append([x, y])
                    except ValueError:
                        continue
            if len(waypoints) < 2:
                QMessageBox.warning(self, "数据错误", "至少需要两个打卡点")
                return
            self.waypoints = np.array(waypoints)
            self.log_text.append(f"成功导入 {len(self.waypoints)} 个打卡点")
            self.info_label.setText("状态: 数据导入成功")
            self.run_button.setEnabled(True)
            # 更新绘图
            self.plot_widget.update_plot(None, None, None, None, self.waypoints)
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"导入数据时发生错误: {str(e)}")
    def run_path_planning(self):
        """运行路径规划"""
        if self.waypoints is None:
            QMessageBox.warning(self, "数据错误", "请先导入打卡点数据")
            return
        if self.planner_thread and self.planner_thread.isRunning():
            QMessageBox.information(self, "操作提示", "路径规划已在运行中")
            return
        self.log_text.append("开始路径规划...")
        self.info_label.setText("状态: 路径规划中")
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.save_button.setEnabled(False)
        # 创建并启动规划线程
        self.planner_thread = PathPlannerThread(self.waypoints)
        self.planner_thread.update_progress.connect(self.update_progress)
        self.planner_thread.update_plot.connect(self.plot_widget.update_plot)
        self.planner_thread.update_curvature.connect(self.plot_widget.update_curvature_plot)
        self.planner_thread.finished.connect(self.path_planning_finished)
        self.planner_thread.error_occurred.connect(self.handle_error)
        self.planner_thread.start()
    def stop_path_planning(self):
        """停止路径规划"""
        if self.planner_thread and self.planner_thread.isRunning():
            self.planner_thread.stop()
            self.planner_thread.quit()
            self.planner_thread.wait()
            self.log_text.append("路径规划已停止")
            self.info_label.setText("极: 已停止")
            self.run_button.setEnabled(True)
            self.stop_button.setEnabled(False)
    def save_path_data(self):
        """保存路径数据"""
        if self.path_points is None:
            QMessageBox.warning(self, "数据错误", "没有可保存的路径数据")
            return
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存路径数据", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        if not file_path:
            return
        try:
            # 确保文件扩展名正确
            if not file_path.lower().endswith('.txt'):
                file_path += '.txt'
            # 保存数据
            with open(file_path, 'w') as f:
                f.write("AI优化路径数据\n")
                f.write("Index\tX(mm)\tY(mm)\tCurvature\tCurvature_Radius(mm)\tNormalized_Distance\n")
                # 只保存前10000个点（减少文件大小）
                num_points = min(100000, len(self.path_points))
                for i in range(num_points):
                    x, y = self.path_points[i]
                    curvature = self.curvatures[i]
                    curvature_radius = self.curvature_radii[i]
                    distance = self.distances[i]
                    line = f"{i}\t{x:.4f}\t{y:.4f}\t{curvature:.6f}\t{curvature_radius:.2f}\t{distance:.6f}\n"
                    f.write(line)
            self.log_text.append(f"路径数据已保存至: {file_path} (共{num_points}个点)")
        except Exception as e:
            QMessageBox.critical(self, "保存错误", f"保存数据时发生错误: {str(e)}")
    def update_progress(self, message, fitness, max_curvature_change):
        """更新进度信息"""
        self.log_text.append(message)
        self.fitness_label.setText(f"当前适应度: {fitness:.4f}")
        self.curvature_label.setText(f"最大曲率变化率: {max_curvature_change:.6f}")
    def path_planning_finished(self, points, curvatures, curvature_radii, distances, control_points):
        """路径规划完成"""
        self.path_points = points
        self.curvatures = curvatures
        self.curvature_radii = curvature_radii
        self.distances = distances
        self.control_points = control_points
        self.log_text.append("路径规划完成!")
        self.info_label.setText("状态: 规划完成")
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.save_button.setEnabled(True)
        # 更新绘图
        # 生成最优路径用于可视化
        spline_x, spline_y, _ = self._create_spline(control_points)
        u_new = np.linspace(0, 1, 100)
        x = spline_x(u_new)
        y = spline_y(u_new)
        best_path_points = np.column_stack((x, y))
        self.plot_widget.update_plot(control_points, best_path_points, None, None, self.waypoints)
        # 更新曲率图
        self.plot_widget.update_curvature_plot(distances, curvatures)
    def handle_error(self, error_message):
        """处理错误"""
        QMessageBox.critical(self, "错误", error_message)
        self.log_text.append(f"错误: {error_message}")
        self.info_label.setText("状态: 错误")
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
    def _create_spline(self, control_points):
        """创建三次样条曲线（辅助方法）"""
        distances = np.zeros(len(control_points))
        for i in range(1, len(control_points)):
            dx = control_points[i, 0] - control_points[i - 1, 0]
            dy = control_points[i, 1] - control_points[i - 1, 1]
            distances[i] = distances[i - 1] + np.sqrt(dx ** 2 + dy ** 2)
        u = distances / distances[-1]
        spline_degree = min(5, len(control_points) - 1)
        spline_x = make_interp_spline(u, control_points[:, 0], k=spline_degree)
        spline_y = make_interp_spline(u, control_points[:, 1], k=spline_degree)
        return spline_x, spline_y, u
#####
##下面是凸轮设计部分
#####
class CamDesigner:
    """凸轮设计器，用于根据路径数据设计凸轮"""
    def __init__(self, X, Y, R, waypoints):
        """
        凸轮设计器
        参数:
            X (np.array): X坐标数组
            Y (np.array): Y坐标数组
            R (np.array): 曲率半径数组
            waypoints (np.array): 打卡点坐标数组
        """
        self.X = X
        self.Y = Y
        self.R = R
        self.waypoints = waypoints
        self.cam_data = None
        # 默认参数
        self.M = 150.0  # 车总宽，两侧后轮触地中点距离
        self.m = 30.0  # 车把偏置距离，左偏为正
        self.E = 20.0  # 车把长度，前轮中心与凸轮中心间距
        self.L = 107.0  # 轴距，前后轮轴间距离
        self.e = 73.02  # 凸轮基圆半径
        self.n = 42.0  # 期望传动比
        self.r0 = 65.0  # 后轮半径
        self.d = 2.0  # 推杆直径
        self.d_cam = 3.0  # 凸轮总厚度
        self.Putter_Type = 1  # 推杆类型：0=球头，1=平推
        self.leftorright = 1  # 凸轮转向：1=左拐（凸轮凹下），-1=右拐
    def calculate_cam(self):
        """计算凸轮数据"""
        try:
            # 1. 计算后轮轨迹
            dX = np.diff(self.X, prepend=self.X[0])
            dY = np.diff(self.Y, prepend=self.Y[0])
            # 2. 计算左右后轮轨迹
            X1 = self.X - (self.M / 2) * dY / np.sqrt(dX ** 2 + dY ** 2)
            Y1 = self.Y + (self.M / 2) * dX / np.sqrt(dX ** 2 + dY ** 2)
            X2 = self.X - (-self.M / 2) * dY / np.sqrt(dX ** 2 + dY ** 2)
            Y2 = self.Y + (-self.M / 2) * dX / np.sqrt(dX ** 2 + dY ** 2)
            # 3. 计算轨迹总长
            s = np.zeros_like(self.R)
            ds = np.sqrt(np.diff(self.X) ** 2 + np.diff(self.Y) ** 2)
            s[1:] = np.cumsum(ds)
            total_length = s[-1]
            # 4. 计算凸轮转过总角度
            use_circle = total_length / (self.n * self.r0)
            # 5. 计算凸轮角度分布
            theta = np.zeros_like(s)
            normalized_s = s / total_length
            theta = normalized_s * use_circle
            # 6. 计算凸轮极径
            if self.Putter_Type == 0:  # 球头推杆
                rou = self.e - self.leftorright * self.E * np.sin(np.arctan(self.L / (self.R - self.m)))
            elif self.Putter_Type == 1:  # 平推推杆
                rou = self.e - self.leftorright * self.E * self.L / (self.R - self.m)
            else:
                raise ValueError("推杆类型错误: 必须是0(球头)或1(平推)")
            # 7. 计算实际凸轮廓线（考虑推杆直径）
            theoretical_points = np.column_stack((theta, rou))
            actual_points = self._calculate_actual_profile(theta, rou)
            self.cam_data = {
                'theta': theta,
                'rou': rou,
                'theoretical_points': theoretical_points,
                'actual_points': actual_points,
                'base_circle': self.e - 0.5 * self.d,
                'total_length': total_length,
                'max_rou': np.max(rou),
                'min_rou': np.min(rou),
                'use_circle': use_circle,
                'X1': X1,
                'Y1': Y1,
                'X2': X2,
                'Y2': Y2
            }
            return self.cam_data
        except Exception as e:
            raise ValueError(f"凸轮设计错误: {str(e)}")
    def _calculate_actual_profile(self, theta, rou):
        """计算实际凸轮廓线（考虑推杆直径）"""
        # 计算理论轮廓的直角坐标
        x0 = rou * np.cos(theta)
        y0 = rou * np.sin(theta)
        # 计算法线方向
        dx_dtheta = np.gradient(x0)
        dy_dtheta = np.gradient(y0)
        norm = np.sqrt(dx_dtheta ** 2 + dy_dtheta ** 2)
        nx = -dy_dtheta / norm
        ny = dx_dtheta / norm
        # 计算偏置距离
        if self.Putter_Type == 0:  # 球头推杆
            offset = 0.5 * self.d
        else:  # 平推推杆
            # 计算压力角
            alpha = np.arctan(np.abs(rou - self.e) / self.E)
            offset = 0.5 * self.d / np.cos(alpha)
        # 考虑凸轮转向，确定偏置方向
        offset *= self.leftorright
        # 计算实际轮廓点
        x1 = x0 - offset * nx
        y1 = y0 - offset * ny
        # 转换回极坐标
        actual_theta = np.arctan2(y1, x1)
        actual_rou = np.sqrt(x1 ** 2 + y1 ** 2)
        return np.column_stack((actual_theta, actual_rou))
    def find_waypoint_angles(self):
        """找到打卡点在凸轮上对应的极角"""
        if self.cam_data is None:
            raise ValueError("请先计算凸轮数据")
        # 路径点坐标
        X = self.X
        Y = self.Y
        theta = self.cam_data['theta']
        results = []
        for i, waypoint in enumerate(self.waypoints):
            # 找到路径上最近的点
            dists = np.linalg.norm(np.column_stack((X, Y)) - waypoint, axis=1)
            min_idx = np.argmin(dists)
            # 获取对应的极角
            angle_rad = theta[min_idx]
            angle_deg = np.degrees(angle_rad)
            results.append({
                'waypoint_idx': i,
                'waypoint': waypoint,
                'nearest_point': [X[min_idx], Y[min_idx]],
                'nearest_idx': min_idx,
                'angle_rad': angle_rad,
                'angle_deg': angle_deg,
                'distance': dists[min_idx]
            })
        return results
class CamDesignWindow(QWidget):
    """凸轮设计窗口"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("凸轮设计")
        self.setGeometry(100, 100, 1200, 800)
        self.waypoints = None  # 打卡点数据
        self.path_data = None  # 路径点数据
        self.cam_data = None  # 凸轮设计结果
        self.init_ui()
    def init_ui(self):
        """初始化UI"""
        main_layout = QHBoxLayout()
        # 左侧面板
        left_panel = QFrame()
        left_panel.setFrameShape(QFrame.StyledPanel)
        left_layout = QVBoxLayout()
        # 导入打卡点数据按钮
        self.import_waypoints_button = QPushButton("导入打卡点数据")
        self.import_waypoints_button.clicked.connect(self.import_waypoints)
        left_layout.addWidget(self.import_waypoints_button)
        # 导入路径点数据按钮
        self.import_path_button = QPushButton("导入路径点数据")
        self.import_path_button.clicked.connect(self.import_path_data)
        left_layout.addWidget(self.import_path_button)
        # 路径点数据列选择
        col_group = QGroupBox("路径点数据列选择")
        col_layout = QGridLayout()
        # X坐标列
        col_layout.addWidget(QLabel("X坐标列:"), 0, 0)
        self.x_col_spin = QSpinBox()
        self.x_col_spin.setRange(1, 10)
        self.x_col_spin.setValue(2)
        col_layout.addWidget(self.x_col_spin, 0, 1)
        # Y坐标列
        col_layout.addWidget(QLabel("Y坐标列:"), 1, 0)
        self.y_col_spin = QSpinBox()
        self.y_col_spin.setRange(1, 10)
        self.y_col_spin.setValue(3)
        col_layout.addWidget(self.y_col_spin, 1, 1)
        # 曲率半径列
        col_layout.addWidget(QLabel("曲率半径列:"), 2, 0)
        self.r_col_spin = QSpinBox()
        self.r_col_spin.setRange(1, 10)
        self.r_col_spin.setValue(5)
        col_layout.addWidget(self.r_col_spin, 2, 1)
        col_group.setLayout(col_layout)
        left_layout.addWidget(col_group)
        # 凸轮参数设置
        param_group = QGroupBox("凸轮参数设置")
        param_layout = QGridLayout()
        # 参数标签
        labels = [
            "车总宽M(mm):", "车把偏置距离m(mm):", "车把长度E(mm):", "轴距L(mm):",
            "凸轮基圆半径e(mm):", "期望传动比n:", "后轮半径r0(mm):", "推杆直径d(mm):",
            "凸轮总厚度d_cam(mm):", "推杆类型:", "凸轮转向:"
        ]
        # 参数输入框
        self.param_edits = {}
        for i, label in enumerate(labels):
            row = i // 2
            col = (i % 2) * 2
            param_layout.addWidget(QLabel(label), row, col)
            if label == "推杆类型:":
                self.putter_type_combo = QComboBox()
                self.putter_type_combo.addItem("球头推杆", 0)
                self.putter_type_combo.addItem("平推推杆", 1)
                self.putter_type_combo.setCurrentIndex(1)
                param_layout.addWidget(self.putter_type_combo, row, col + 1)
                self.param_edits["Putter_Type"] = self.putter_type_combo
            elif label == "凸轮转向:":
                self.cam_direction_combo = QComboBox()
                self.cam_direction_combo.addItem("左拐（凸轮凹下）", 1)
                self.cam_direction_combo.addItem("右拐", -1)
                self.cam_direction_combo.setCurrentIndex(0)
                param_layout.addWidget(self.cam_direction_combo, row, col + 1)
                self.param_edits["leftorright"] = self.cam_direction_combo
            else:
                param_name = label.split(":")[0].strip()
                edit = QLineEdit()
                # 设置默认值
                if param_name == "车总宽M(mm)":
                    edit.setText("150.0")
                elif param_name == "车把偏置距离m(mm)":
                    edit.setText("30.0")
                elif param_name == "车把长度E(mm)":
                    edit.setText("20.0")
                elif param_name == "轴距L(mm)":
                    edit.setText("107.0")
                elif param_name == "凸轮基圆半径e(mm)":
                    edit.setText("73.02")
                elif param_name == "期望传动比n":
                    edit.setText("42.0")
                elif param_name == "后轮半径r0(mm)":
                    edit.setText("65.0")
                elif param_name == "推杆直径d(mm)":
                    edit.setText("2.0")
                elif param_name == "凸轮总厚度d_cam(mm)":
                    edit.setText("3.0")
                param_layout.addWidget(edit, row, col + 1)
                self.param_edits[param_name] = edit
        param_group.setLayout(param_layout)
        left_layout.addWidget(param_group)
        # 运行按钮
        self.run_button = QPushButton("运行凸轮设计")
        self.run_button.clicked.connect(self.run_cam_design)
        self.run_button.setEnabled(False)
        left_layout.addWidget(self.run_button)
        # 保存按钮
        self.save_button = QPushButton("保存凸轮数据")
        self.save_button.clicked.connect(self.save_cam_data)
        self.save_button.setEnabled(False)
        left_layout.addWidget(self.save_button)
        # 信息显示
        info_group = QGroupBox("状态信息")
        info_layout = QVBoxLayout()
        self.info_label = QLabel("状态: 等待导入数据")
        info_layout.addWidget(self.info_label)
        self.result_label = QLabel("结果:")
        info_layout.addWidget(self.result_label)
        info_group.setLayout(info_layout)
        left_layout.addWidget(info_group)
        # 日志文本框
        log_group = QGroupBox("操作日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.append("导入格式说明:")
        self.log_text.append("1. 需要导入两个文件:")
        self.log_text.append("   a) 打卡点数据: 包含多个打卡点的X和Y坐标")
        self.log_text.append("   b) 路径点数据: 包含大量路径点的多列数据")
        self.log_text.append("2. 文件应为文本文件(.txt)")
        self.log_text.append("3. 数据列以空格或制表符分隔")
        self.log_text.append("4. 路径点数据列选择说明:")
        self.log_text.append("   - X坐标列: 路径点的X坐标")
        self.log_text.append("   - Y坐标列: 路径点的Y坐标")
        self.log_text.append("   - 曲率半径列: 路径点的曲率半径")
        self.log_text.append("5. 示例文件格式:")
        self.log_text.append("   a) 打卡点数据:")
        self.log_text.append("       5588 713")
        self.log_text.append("       4463 375")
        self.log_text.append("       2925 825")
        self.log_text.append("   b) 路径点数据:")
        self.log_text.append("       Index\tX(mm)\tY(mm)\tCurvature\tCurvature_Radius(mm)\tNormalized_Distance")
        self.log_text.append("       0\t5588.0\t713.0\t0.000123\t1000000.0\t0.000000")
        self.log_text.append("       1\t5570.5\t715.2\t0.000145\t6896.55\t0.000015")
        self.log_text.append("       ...")
        self.log_text.append("\n凸轮参数说明:")
        self.log_text.append("1. 车总宽M(mm): 两侧后轮触地中点距离")
        self.log_text.append("2. 车把偏置距离m(mm): 左偏为正")
        self.log_text.append("3. 车把长度E(mm): 前轮中心与凸轮中心间距")
        self.log_text.append("4. 轴距L(mm): 前后轮轴间距离")
        self.log_text.append("5. 凸轮基圆半径e(mm): 凸轮基圆半径")
        self.log_text.append("6. 期望传动比n: 期望的传动比")
        self.log_text.append("7. 后轮半径r0(mm): 后轮半径")
        self.log_text.append("8. 推杆直径d(mm): 推杆直径")
        self.log_text.append("9. 凸轮总厚度d_cam(mm): 凸轮总厚度")
        self.log_text.append("10. 推杆类型: 球头推杆(0)或平推推杆(1)")
        self.log_text.append("11. 凸轮转向: 左拐(凸轮凹下)或右拐")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)
        left_panel.setLayout(left_layout)
        # 右侧绘图区域
        self.figure = Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        # 创建导航工具栏
        self.toolbar = NavigationToolbar(self.canvas, self)
        # 创建两个子图：上面是路径图，下面是凸轮廓线图
        self.ax_path = self.figure.add_subplot(211)
        self.ax_cam = self.figure.add_subplot(212, projection='polar')
        # 创建布局
        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.toolbar)  # 添加工具栏
        plot_layout.addWidget(self.canvas)  # 添加画布
        plot_widget = QWidget()
        plot_widget.setLayout(plot_layout)
        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(plot_widget, 2)
        self.setLayout(main_layout)
    def import_waypoints(self):
        """导入打卡点数据"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开打卡点数据", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        if not file_path:
            return
        try:
            # 读取文件
            with open(file_path, 'r') as f:
                lines = f.readlines()
            # 解析数据
            waypoints = []
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                # 尝试空格分隔
                if ' ' in line:
                    parts = line.split()
                # 尝试逗号分隔
                elif ',' in line:
                    parts = line.split(',')
                else:
                    continue
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        waypoints.append([x, y])
                    except ValueError:
                        continue
            if len(waypoints) < 1:
                QMessageBox.warning(self, "数据错误", "至少需要一个打卡点")
                return
            self.waypoints = np.array(waypoints)
            self.log_text.append(f"成功导入 {len(self.waypoints)} 个打卡点")
            self.info_label.setText("状态: 打卡点数据导入成功")
            # 检查是否所有数据都已导入
            self.check_data_ready()
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"导入打卡点数据时发生错误: {str(e)}")
    def import_path_data(self):
        """导入路径点数据"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开路径点数据", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        if not file_path:
            return
        try:
            # 读取文件
            with open(file_path, 'r') as f:
                lines = f.readlines()
            # 解析数据
            data = []
            for line in lines:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                # 尝试分割
                if '\t' in line:
                    parts = line.split('\t')
                elif ' ' in line:
                    parts = line.split()
                else:
                    continue
                try:
                    # 转换为浮点数
                    row = [float(part) for part in parts]
                    data.append(row)
                except ValueError:
                    continue
            if len(data) < 2:
                QMessageBox.warning(self, "数据错误", "至少需要两个路径点")
                return
            self.path_data = np.array(data)
            self.log_text.append(f"成功导入 {len(self.path_data)} 个路径点")
            self.info_label.setText("状态: 路径点数据导入成功")
            # 绘制路径图
            self.draw_path_plot()
            # 检查是否所有数据都已导入
            self.check_data_ready()
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"导入路径点数据时发生错误: {str(e)}")
    def check_data_ready(self):
        """检查是否所有数据都已导入"""
        if self.waypoints is not None and self.path_data is not None:
            self.run_button.setEnabled(True)
            self.info_label.setText("状态: 数据导入完成，可运行凸轮设计")
        else:
            self.run_button.setEnabled(False)
    def draw_path_plot(self):
        """绘制路径图"""
        if self.path_data is None:
            return
        # 获取选择的列索引
        x_col = self.x_col_spin.value() - 1
        y_col = self.y_col_spin.value() - 1
        # 确保列索引有效
        if x_col >= self.path_data.shape[1] or y_col >= self.path_data.shape[1]:
            QMessageBox.warning(self, "列选择错误", "选择的列索引超出范围")
            return
        # 提取坐标
        X = self.path_data[:, x_col]
        Y = self.path_data[:, y_col]
        # 计算连续两点之间的距离差
        dX = np.diff(X, prepend=X[0])
        dY = np.diff(Y, prepend=Y[0])
        dist = np.sqrt(dX ** 2 + dY ** 2)
        # 避免除以零
        dist[dist == 0] = 1e-8
        # 单位切向量
        tx = dX / dist
        ty = dY / dist
        # 法向量（旋转90度）
        nx = -ty
        ny = tx
        # 左右后轮轨迹（距离后轴中点M/2）
        # 获取参数值
        params = {}
        for name, widget in self.param_edits.items():
            if isinstance(widget, QLineEdit):
                params[name] = float(widget.text())
            elif isinstance(widget, QComboBox):
                params[name] = widget.currentData()
        X1 = X + (params.get("车总宽M(mm)", 150.0)/ 2) * nx
        Y1 = Y + (params.get("车总宽M(mm)", 150.0) / 2) * ny
        X2 = X - (params.get("车总宽M(mm)", 150.0)/ 2) * nx
        Y2 = Y - (params.get("车总宽M(mm)", 150.0) / 2) * ny
        # 绘制路径
        self.ax_path.clear()
        # 绘制中点轨迹
        self.ax_path.plot(X, Y, 'b-', linewidth=1.5, label='中点轨迹')
       # 绘制左右后轮轨迹
        self.ax_path.plot(X1, Y1, 'g--', linewidth=1.0, label='左后轮轨迹')
        self.ax_path.plot(X2, Y2, 'r--', linewidth=1.0, label='右后轮轨迹')
        # 绘制打卡点
        if self.waypoints is not None:
            self.ax_path.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'ro', markersize=5, label='打卡点')
        self.ax_path.set_title('路径轨迹')
        self.ax_path.set_xlabel('X (mm)')
        self.ax_path.set_ylabel('Y (mm)')
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.axis('equal')
        self.ax_path.legend()
        self.canvas.draw()
    def run_cam_design(self):
        """运行凸轮设计"""
        if self.waypoints is None or self.path_data is None:
            QMessageBox.warning(self, "数据错误", "请先导入打卡点和路径点数据")
            return
        try:
            # 获取选择的列索引
            x_col = self.x_col_spin.value() - 1
            y_col = self.y_col_spin.value() - 1
            r_col = self.r_col_spin.value() - 1
            # 确保列索引有效
            if (x_col >= self.path_data.shape[1] or
                    y_col >= self.path_data.shape[1] or
                    r_col >= self.path_data.shape[1]):
                QMessageBox.warning(self, "列选择错误", "选择的列索引超出范围")
                return
            # 提取数据
            X = self.path_data[:, x_col]
            Y = self.path_data[:, y_col]
            R = self.path_data[:, r_col]
            # 获取参数值
            params = {}
            for name, widget in self.param_edits.items():
                if isinstance(widget, QLineEdit):
                    params[name] = float(widget.text())
                elif isinstance(widget, QComboBox):
                    params[name] = widget.currentData()
            # 创建凸轮设计器
            self.cam_designer = CamDesigner(X, Y, R, self.waypoints)
            # 设置参数
            self.cam_designer.M = params.get("车总宽M(mm)", 150.0)
            self.cam_designer.m = params.get("车把偏置距离m(mm)", 30.0)
            self.cam_designer.E = params.get("车把长度E(mm)", 20.0)
            self.cam_designer.L = params.get("轴距L(mm)", 107.0)
            self.cam_designer.e = params.get("凸轮基圆半径e(mm)", 73.02)
            self.cam_designer.n = params.get("期望传动比n", 42.0)
            self.cam_designer.r0 = params.get("后轮半径r0(mm)", 65.0)
            self.cam_designer.d = params.get("推杆直径d(mm)", 2.0)
            self.cam_designer.d_cam = params.get("凸轮总厚度d_cam(mm)", 3.0)
            self.cam_designer.Putter_Type = params.get("Putter_Type", 1)
            self.cam_designer.leftorright = params.get("leftorright", 1)
            # 计算凸轮数据
            self.cam_data = self.cam_designer.calculate_cam()
            # 显示结果
            self.log_text.append("凸轮设计完成!")
            self.log_text.append(f"极径最大值: {self.cam_data['max_rou']:.2f} mm")
            self.log_text.append(f"极径最小值: {self.cam_data['min_rou']:.2f} mm")
            self.log_text.append(f"路径总长: {self.cam_data['total_length']:.2f} mm")
            self.log_text.append(f"凸轮转过总角度: {np.degrees(self.cam_data['use_circle']):.2f} 度")
            # 更新结果标签
            self.result_label.setText(
                f"结果: 极径范围 [{self.cam_data['min_rou']:.2f}, {self.cam_data['max_rou']:.2f}] mm\n"
                f"路径总长: {self.cam_data['total_length']:.2f} mm\n"
                f"凸轮转角: {np.degrees(self.cam_data['use_circle']):.2f} 度"
            )
            # 启用保存按钮
            self.save_button.setEnabled(True)
            # 绘制凸轮廓线
            self.draw_cam_profile()
            # 显示打卡点对应极角
            self.log_waypoint_angles()
        except Exception as e:
            QMessageBox.critical(self, "凸轮设计错误", f"设计凸轮时发生错误: {str(e)}")
    def log_waypoint_angles(self):
        """记录打卡点对应极角"""
        if self.cam_designer is None or self.cam_data is None:
            return
        waypoint_angles = self.cam_designer.find_waypoint_angles()
        self.log_text.append("\n打卡点对应极角:")
        for result in waypoint_angles:
            self.log_text.append(
                f"打卡点 {result['waypoint_idx'] + 1}: "
                f"位置({result['waypoint'][0]:.2f}, {result['waypoint'][1]:.2f}) "
                f"对应极角: {result['angle_deg']:.2f}°"
            )
    def draw_cam_profile(self):
        """绘制凸轮廓线"""
        if self.cam_data is None:
            return
        # 获取数据
        theta = self.cam_data['theta']
        rou = self.cam_data['rou']
        actual_points = self.cam_data['actual_points']
        base_circle = self.cam_data['base_circle']
        # 绘制凸轮廓线
        self.ax_cam.clear()
        # 理论廓线
        self.ax_cam.plot(theta, rou, 'g-', label='理论廓线')
        # 实际廓线
        self.ax_cam.plot(actual_points[:, 0], actual_points[:, 1], 'r-', label='实际廓线')
        # 基圆
        base_theta = np.linspace(0, 2 * np.pi, 100)
        base_rou = np.full_like(base_theta, base_circle)
        self.ax_cam.plot(base_theta, base_rou, 'b--', label='基圆')
        # 设置标题
        self.ax_cam.set_title('凸轮廓线')
        self.ax_cam.legend()
        # 标记打卡点位置
        if self.cam_designer:
            waypoint_angles = self.cam_designer.find_waypoint_angles()
            for result in waypoint_angles:
                self.ax_cam.plot(
                    result['angle_rad'],
                    rou[result['nearest_idx']],
                    'ro', markersize=5
                )
                self.ax_cam.text(
                    result['angle_rad'],
                    rou[result['nearest_idx']] + 5,
                    f"WP{result['waypoint_idx'] + 1}",
                    fontsize=8
                )
        self.canvas.draw()
    def save_cam_data(self):
        """保存凸轮数据"""
        if self.cam_data is None:
            QMessageBox.warning(self, "数据错误", "没有可保存的凸轮数据")
            return
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存凸轮数据", "", "文本文件 (*.txt);;所有文件 (*)"
        )
        if not file_path:
            return
        try:
            # 确保文件扩展名正确
            if not file_path.lower().endswith('.txt'):
                file_path += '.txt'
            # 获取数据
            theoretical_points = self.cam_data['theoretical_points']
            actual_points = self.cam_data['actual_points']
            # 保存数据
            with open(file_path, 'w') as f:
                f.write("# 凸轮设计数据\n")
                f.write("# 参数:\n")
                f.write(f"# 车总宽M(mm): {self.cam_designer.M}\n")
                f.write(f"# 车把偏置距离m(mm): {self.cam_designer.m}\n")
                f.write(f"# 车把长度E(mm): {self.cam_designer.E}\n")
                f.write(f"# 轴距L(mm): {self.cam_designer.L}\n")
                f.write(f"# 凸轮基圆半径e(mm): {self.cam_designer.e}\n")
                f.write(f"# 期望传动比n: {self.cam_designer.n}\n")
                f.write(f"# 后轮半径r0(mm): {self.cam_designer.r0}\n")
                f.write(f"# 推杆直径d(mm): {self.cam_designer.d}\n")
                f.write(f"# 凸轮总厚度d_cam(mm): {self.cam_designer.d_cam}\n")
                f.write(f"# 推杆类型: {'球头' if self.cam_designer.Putter_Type == 0 else '平推'}\n")
                f.write(f"# 凸轮转向: {'左拐' if self.cam_designer.leftorright == 1 else '右拐'}\n")
                f.write("#\n")
                f.write("# 计算结果:\n")
                f.write(f"# 极径最大值(mm): {self.cam_data['max_rou']:.2f}\n")
                f.write(f"# 极径最小值(mm): {self.cam_data['min_rou']:.2f}\n")
                f.write(f"# 路径总长(mm): {self.cam_data['total_length']:.2f}\n")
                f.write(f"# 凸轮转过总角度(度): {self.cam_data['use_circle']:.2f}\n")
                f.write("#\n")
                f.write("# 理论廓线数据 (X[mm], Y[mm], Z[mm]):\n")
                # 均匀采样 1000 个点
                n_pts = len(theoretical_points)
                step = max(1, n_pts // 1000)
                for i in range(0, n_pts, step):
                    th, r = theoretical_points[i, 0], theoretical_points[i, 1]
                    f.write(f"{r * np.cos(th):.6f}\t{r * np.sin(th):.6f}\t0.000\n")
                # f.write("\n# 实际廓线数据 (X[mm], Y[mm], Z[mm]):\n")
                # n_pts = len(actual_points)
                # step = max(1, n_pts // 1000)
                # for i in range(0, n_pts, step):
                #     x, y = actual_points[i]
                #     f.write(f"{x:.6f}\t{y:.6f}\t0.000\n")
            self.log_text.append(f"凸轮数据已保存至: {file_path}")
        except Exception as e:
            QMessageBox.critical(self, "保存错误", f"保存数据时发生错误: {str(e)}")
class MainWindow(QMainWindow):
    """主窗口"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("凸轮设计软件")
        self.setGeometry(100, 100, 800, 600)
        self.set_gradient_background()
        self.init_ui()
    def set_gradient_background(self):
        """设置渐变背景"""
        palette = self.palette()
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(236, 240, 241))  # 浅灰
        gradient.setColorAt(1, QColor(189, 195, 199))  # 中灰
        palette.setBrush(QPalette.Window, QBrush(gradient))
        self.setPalette(palette)
    def init_ui(self):
        """初始化UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        # 标题
        title_label = QLabel("凸轮设计软件")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
                    font-size: 32px;
                    font-weight: bold;
                    color: #2c3e50;
                    margin: 20px;
                    padding: 10px;
                    border-bottom: 2px solid #3498db;
                """)
        main_layout.addWidget(title_label)
        # 添加描述
        description_label = QLabel("主要用于共创赛工程基础赛道凸轮设计")
        description_label.setAlignment(Qt.AlignCenter)
        description_label.setStyleSheet("""
            font-size: 18px;
            color: #000000;
            margin-bottom: 30px;
        """)
        main_layout.addWidget(description_label)
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.setSpacing(30)
        button_layout.setContentsMargins(50, 0, 50, 0)
        # 创建按钮样式
        button_style = """
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                color: white;
                background-color: #3498db;
                border-radius: 10px;
                padding: 15px;
                min-width: 200px;
                min-height: 70px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1c6ea4;
            }
        """
        # 路径AI设计按钮
        path_button = QPushButton("路径AI设计")
  #      path_button.setFixedSize(200, 60)
        path_button.setStyleSheet(button_style)
        path_button.clicked.connect(self.open_path_design)
        button_layout.addWidget(path_button)
        # 凸轮设计按钮
        cam_button = QPushButton("凸轮设计")
        cam_button.setStyleSheet(button_style)
        cam_button.clicked.connect(self.open_cam_design)
        button_layout.addWidget(cam_button)
        # 说明按钮
        help_button = QPushButton("说明")
     #   help_button.setFixedSize(200, 60)
        help_button.setStyleSheet(button_style)
        help_button.clicked.connect(self.show_help)
        button_layout.addWidget(help_button)
        main_layout.addLayout(button_layout)
        # 背景图
        # 添加中间留白
        main_layout.addSpacing(40)
        # 背景区域
        background_widget = QWidget()
        background_widget.setStyleSheet("""
            background-color: #f8f9fa;
            border-radius: 15px;
            border: 1px solid #e0e0e0;
        """)
        # 添加阴影效果
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(15)
        shadow.setXOffset(0)
        shadow.setYOffset(5)
        shadow.setColor(QColor(0, 0, 0, 100))
        background_widget.setGraphicsEffect(shadow)
        background_layout = QVBoxLayout(background_widget)
        background_layout.setContentsMargins(20, 20, 20, 20)
        # 检查背景图片
        background_path = "background.png"
        if os.path.exists(background_path):
            try:
                # 创建图片标签
                image_label = QLabel()
                pixmap = QPixmap(background_path)
                # 缩放图片以适应窗口
                pixmap = pixmap.scaled(
                    800, 400,
                    Qt.KeepAspectRatioByExpanding,
                    Qt.SmoothTransformation
                )
                image_label.setPixmap(pixmap)
                image_label.setAlignment(Qt.AlignCenter)
                image_label.setStyleSheet("border-radius: 10px;")
                background_layout.addWidget(image_label)
            except Exception as e:
                print(f"加载背景图片错误: {str(e)}")
                # 使用默认文本背景
                self.create_text_background(background_layout)
        else:
            # 使用默认文本背景
            self.create_text_background(background_layout)
        main_layout.addWidget(background_widget, 1)
        # 底部信息栏
        footer = QLabel("© 2025 凸轮设计软件 by:wxt | 版本 1.0")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("""
            font-size: 30px;
            color: #000000;
            margin-top: 20px;
            padding: 10px;
            border-top: 1px solid #ecf0f1;
        """)
        main_layout.addWidget(footer)
    def create_text_background(self, layout):
        """创建文本背景"""
        # 创建容器
        text_container = QWidget()
        text_container.setStyleSheet("""
            background-color: #ffffff;
            border-radius: 10px;
            padding: 20px;
        """)
        text_layout = QVBoxLayout(text_container)
        text_layout.setAlignment(Qt.AlignCenter)
    def open_path_design(self):
        """打开路径设计窗口"""
        self.path_window = PathDesignWindow()
        self.path_window.show()
    def open_cam_design(self):
        """打开凸轮设计窗口"""
        try:
            # 创建窗口实例
            self.cam_window = CamDesignWindow()
            # 显示窗口
            self.cam_window.show()
        except Exception as e:
            # 捕获并显示异常
            QMessageBox.critical(self, "错误", f"打开凸轮设计窗口时发生错误: {str(e)}")
    def show_help(self):
        """显示帮助信息"""
        help_text = """
        <h2>凸轮设计软件使用说明</h2>
        <p><b>本软件为wxt团队使用，严禁侵权，联系方式：1990029866@qq.com</b></p>
        <p><b>路径AI设计：</b>使用人工智能算法优化凸轮路径，生成平滑的凸轮轮廓。但目前效果还不是很理想，建议路径使用UG样条曲线生成</p>
        <p><b>凸轮设计：</b>基于优化后的路径设计凸轮几何形状。请在使用时认真查看凸轮设计内的左侧说明以及参数定义</p>
        <p><b>使用流程：</b></p>
        <ol>
            <li>点击"路径AI设计"按钮打开路径设计窗口（可选）</li>
            <li>导入包含打卡点坐标的文本文件</li>
            <li>点击"运行AI路径规划"开始优化</li>
            <li>优化完成后保存路径数据</li>
            <p><b>------------------------------------------</b></p>
            <li>打开凸轮设计窗口</li>
            <li>导入打卡点数据和路径数据</li>
            <li>设置凸轮参数</li>
            <li>点击"运行凸轮设计"</li>
            <li>保存凸轮数据</li>
        </ol>
        """
        QMessageBox.information(self, "软件说明", help_text)
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
