from dataclasses import replace

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

from cam_calc import (
    FULL_TURN,                              # 常量 2π
    CamParameters,                          # 凸轮机构参数数据类
    calculate_flat_cam,                     # 根据路径曲率生成完整凸轮轮廓
    calculate_signed_curvature,             # 从样条曲线计算带符号曲率
    create_parametric_spline,               # 控制点 → B 样条参数曲线 x(t)/y(t)
    flat_follower_radius_from_curvature,   # 曲率 → 平推/摆杆凸轮工作极径
    signed_curvature_radius,               # 带符号曲率 → 带符号曲率半径
)
from path_tasks import evaluate_constraints  # 路径约束条件校验（评价函数）


def cam_smoothness_cost(angles, radii, reference_radius):
    """按凸轮角计算极径的一、二阶导数，并同时评价均方值和峰值。"""
    radius_speed = np.gradient(radii, angles, edge_order=2)
    radius_acceleration = np.gradient(radius_speed, angles, edge_order=2)
    normalized_speed = radius_speed / reference_radius
    normalized_acceleration = radius_acceleration / reference_radius
    rms_cost = np.mean(normalized_speed**2) + np.mean(normalized_acceleration**2)
    peak_cost = 0.01 * (
        np.max(np.abs(normalized_speed))**2
        + np.max(np.abs(normalized_acceleration))**2
    )
    return rms_cost + peak_cost, radius_speed, radius_acceleration


class PathPlannerThread(QThread):
    """在后台运行粒子群路径规划。"""

    update_progress = pyqtSignal(str, float, float)
    update_plot = pyqtSignal(
        np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
    )
    update_curvature = pyqtSignal(np.ndarray, np.ndarray)
    update_cam = pyqtSignal(object)
    result_ready = pyqtSignal(object)
    error_occurred = pyqtSignal(str)

    def __init__(
        self,
        waypoints,
        max_deviation=80.0,
        spline_degree=3,
        cam_parameters=None,
        constraints=None,
        waypoint_weights=None,
        start_angle_deg=180.0,
        straight_length=200.0,
    ):
        super().__init__()
        self.waypoints = np.asarray(waypoints, dtype=float)
        self.max_deviation = max_deviation
        self.spline_degree = spline_degree
        self.actual_spline_degree = None
        self.cam_parameters = cam_parameters or CamParameters()
        self.constraints = list(constraints or [])
        self.start_angle_deg = start_angle_deg
        self.straight_length = straight_length
        self.spline_knots = None
        self.running = True

        if waypoint_weights is None:
            self.waypoint_weights = np.ones(len(waypoints), dtype=float)
        else:
            self.waypoint_weights = np.asarray(waypoint_weights, dtype=float)
            invalid = (
                self.waypoint_weights.shape != (len(waypoints),)
                or np.any(self.waypoint_weights <= 0)
            )
            if invalid:
                raise ValueError("打卡点权重必须为正数，并且数量与打卡点一致")

    def run(self):
        """运行路径规划算法。"""
        try:
            initial_points = self._create_control_points()
            initial_spline = create_parametric_spline(
                initial_points, degree=self.spline_degree
            )
            self.spline_knots = initial_spline.t.copy()
            self.actual_spline_degree = initial_spline.k
            if self.actual_spline_degree == self.spline_degree:
                degree_message = f"实际使用{self.actual_spline_degree}次 B 样条"
            else:
                degree_message = (
                    f"请求{self.spline_degree}次，因控制点数量不足，"
                    f"实际使用{self.actual_spline_degree}次 B 样条"
                )
            self.update_progress.emit(f"初始化控制点完成；{degree_message}", 0, 0)
            self.update_plot.emit(
                initial_points, np.array([]), initial_points, np.array([]),
                self.waypoints
            )
########超参数#######################
            num_particles = 50
            max_iterations = 2000
            w_start, w_end = 0.9, 0.4
            c1, c2 = 2.0, 2.0
            particles = []
            velocities = []
            personal_best = []
            personal_best_fitness = []
            best_fitness = float("inf")
            best_control_points = initial_points.copy()

            for _ in range(num_particles):
                particle = initial_points + np.random.normal(0, 15, initial_points.shape)
                velocity = np.random.normal(0, 5, initial_points.shape)
                self._lock_boundary(particle, velocity, initial_points)
                fitness, _, max_curvature_change = self._calculate_fitness(particle)
                particles.append(particle)
                velocities.append(velocity)
                personal_best.append(particle.copy())
                personal_best_fitness.append(fitness)
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_control_points = particle.copy()

            self.update_progress.emit(
                "粒子群初始化完成", best_fitness, max_curvature_change
            )
            self._emit_paths(best_control_points, particles[0])

            max_search_radius = 1500.0
            max_velocity = 100.0
            last_best_fitness = best_fitness
            no_improve_count = 0

            for iteration in range(max_iterations):
                if not self.running:
                    break
                w = w_start - (w_start - w_end) * iteration / max_iterations

                for i in range(num_particles):
                    r1 = np.random.random(particles[i].shape)
                    r2 = np.random.random(particles[i].shape)
                    velocities[i] = (
                        w * velocities[i]
                        + c1 * r1 * (personal_best[i] - particles[i])
                        + c2 * r2 * (best_control_points - particles[i])
                    )
                    velocities[i] = np.clip(
                        velocities[i], -max_velocity, max_velocity
                    )
                    particles[i] += velocities[i]
                    particles[i] = np.clip(
                        particles[i],
                        initial_points - max_search_radius,
                        initial_points + max_search_radius,
                    )
                    self._lock_boundary(particles[i], velocities[i], initial_points)

                    fitness, _, max_curvature_change = self._calculate_fitness(
                        particles[i]
                    )
                    if fitness < personal_best_fitness[i]:
                        personal_best_fitness[i] = fitness
                        personal_best[i] = particles[i].copy()
                    if fitness < best_fitness:
                        best_fitness = fitness
                        best_control_points = particles[i].copy()

                if best_fitness < last_best_fitness - 0.01:
                    last_best_fitness = best_fitness
                    no_improve_count = 0
                else:
                    no_improve_count += 1

                if no_improve_count > 80:
                    self.update_progress.emit(
                        f"迭代{iteration}: 触发变异重启",
                        best_fitness,
                        max_curvature_change,
                    )
                    for j in range(num_particles // 2, num_particles):
                        particles[j] = best_control_points + np.random.normal(
                            0, 30, particles[j].shape
                        )
                        velocities[j] = np.random.normal(0, 8, particles[j].shape)
                        self._lock_boundary(
                            particles[j], velocities[j], initial_points
                        )
                        personal_best[j] = particles[j].copy()
                        personal_best_fitness[j] = self._calculate_fitness(
                            particles[j]
                        )[0]
                    no_improve_count = 0

                if iteration % 5 == 0:
                    self.update_progress.emit(
                        f"迭代 {iteration + 1}/{max_iterations}",
                        best_fitness,
                        max_curvature_change,
                    )
                    self._emit_paths(best_control_points, particles[0])

                if iteration % 50 == 0:
                    self._emit_cam_preview(best_control_points)

            stopped_early = not self.running
            points, curvature = self._sample_path(best_control_points, 100000)
            distances = self._normalized_distances(points)
            curvature_radius = signed_curvature_radius(curvature)
            cam_data = calculate_flat_cam(
                points[:, 0], points[:, 1], curvature, self.cam_parameters
            )
            self.update_cam.emit(cam_data)
            self.result_ready.emit({
                "points": points,
                "curvature": curvature,
                "curvature_radius": curvature_radius,
                "distances": distances,
                "control_points": best_control_points,
                "cam_data": cam_data,
                "stopped_early": stopped_early,
            })
            self.update_progress.emit(
                "已停止并生成当前最优结果" if stopped_early else "路径规划完成!",
                best_fitness,
                max_curvature_change,
            )
        except Exception as error:
            self.error_occurred.emit(f"路径规划错误: {error}")

    def stop(self):
        self.running = False

    def _start_direction(self):
        """方向角以 X 轴正方向为 0°，逆时针为正。"""
        angle = np.radians(self.start_angle_deg)
        return np.array([np.cos(angle), np.sin(angle)])

    def _create_control_points(self):
        """创建真实 B 样条控制点，并固定起点方向。"""
        direction = self._start_direction()
        control_points = []
        for start, end in zip(self.waypoints[:-1], self.waypoints[1:]):
            control_points.append(start)
            for j in range(1, 3):
                point = start + j / 3 * (end - start)
                control_points.append(point)
        control_points.append(self.waypoints[-1])
        control_points = np.asarray(control_points)

        # 第一段仍保留原有两个中间控制点，只调整方向，不改变控制点数量。
        handle = np.linalg.norm(self.waypoints[1] - self.waypoints[0]) / 3
        control_points[0] = self.waypoints[0]
        control_points[1] = self.waypoints[0] + handle * direction
        control_points[2] = self.waypoints[0] + 2 * handle * direction
        return control_points

    @staticmethod
    def _lock_boundary(points, velocity, initial_points):
        """固定起点位置、起点方向控制点和路径终点。"""
        points[:3] = initial_points[:3]
        points[-1] = initial_points[-1]
        velocity[:3] = 0.0
        velocity[-1] = 0.0

    def _sample_path(self, control_points, sample_count):
        """拼接精确直线段和二维 B 样条段。"""
        spline = create_parametric_spline(
            control_points,
            degree=self.actual_spline_degree or self.spline_degree,
            knots=self.spline_knots,
        )
        estimate_t = np.linspace(0.0, 1.0, min(sample_count, 1000))
        estimate_points = spline(estimate_t)
        main_length = np.sum(np.linalg.norm(np.diff(estimate_points, axis=0), axis=1))

        if self.straight_length > 0:
            line_ratio = self.straight_length / (self.straight_length + main_length)
            line_count = int(round(sample_count * line_ratio))
            line_count = min(max(line_count, 2), sample_count - 20)
        else:
            line_count = 0
        main_count = sample_count - line_count

        t = np.linspace(0.0, 1.0, main_count)
        main_points = spline(t)
        main_curvature = calculate_signed_curvature(spline, t)
        if line_count == 0:
            return main_points, main_curvature

        line_start = self.waypoints[0] - self.straight_length * self._start_direction()
        line_points = np.linspace(line_start, control_points[0], line_count, endpoint=False)
        points = np.vstack((line_points, main_points))
        curvature = np.concatenate((np.zeros(line_count), main_curvature))
        return points, curvature

    @staticmethod
    def _normalized_distances(points):
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        return distances / distances[-1]

    @staticmethod
    def _resample_by_distance(points, values):
        """把路径坐标和对应数值改为按弧长均匀采样。"""
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        uniform_distances = np.linspace(0.0, distances[-1], len(points))
        uniform_points = np.column_stack((
            np.interp(uniform_distances, distances, points[:, 0]),
            np.interp(uniform_distances, distances, points[:, 1]),
        ))
        uniform_values = np.interp(uniform_distances, distances, values)
        return uniform_points, uniform_values, uniform_distances

    @staticmethod
    def _waypoint_distances(points, waypoints):
        """计算各打卡点到离散路径折线的最短距离。"""
        starts = points[:-1]
        vectors = np.diff(points, axis=0)
        length_squared = np.sum(vectors**2, axis=1)
        distances = []
        for waypoint in waypoints:
            ratios = np.sum((waypoint - starts) * vectors, axis=1) / length_squared
            ratios = np.clip(ratios, 0.0, 1.0)
            projections = starts + ratios[:, None] * vectors
            distances.append(np.min(np.linalg.norm(projections - waypoint, axis=1)))
        return np.asarray(distances)

    def _emit_paths(self, best_control_points, current_control_points):
        best_path, _ = self._sample_path(best_control_points, 500)
        current_path, _ = self._sample_path(current_control_points, 500)
        self.update_plot.emit(
            best_control_points,
            best_path,
            current_control_points,
            current_path,
            self.waypoints,
        )

    def _emit_cam_preview(self, control_points):
        points, curvature = self._sample_path(control_points, 2000)
        distances = self._normalized_distances(points)
        self.update_curvature.emit(distances, curvature)
        try:
            preview_parameters = replace(self.cam_parameters, closure_mode="none")
            cam_data = calculate_flat_cam(
                points[:, 0], points[:, 1], curvature, preview_parameters
            )
            self.update_cam.emit(cam_data)
        except ValueError:
            # 优化中的临时候选路径可能暂时无法生成有效凸轮。
            pass

    def _calculate_fitness(self, control_points):
        """计算凸轮光滑度、打卡点偏差和路径任务罚分。"""
        points, curvature = self._sample_path(control_points, 2000)
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(segment_lengths <= 1e-10):
            return 1e12, 999.0, 999.0
        total_length = np.sum(segment_lengths)
        if total_length > FULL_TURN * self.cam_parameters.n * self.cam_parameters.r0:
            return 1e12, 999.0, 999.0

        points, curvature, distances = self._resample_by_distance(points, curvature)
        normalized_distances = distances / distances[-1]
        cam_angles = distances / (self.cam_parameters.n * self.cam_parameters.r0)

        try:
            cam_radius = flat_follower_radius_from_curvature(
                curvature, self.cam_parameters
            )
        except ValueError:
            return 1e12, 999.0, 999.0

        cam_cost, _, _ = cam_smoothness_cost(
            cam_angles, cam_radius, self.cam_parameters.e
        )

        waypoint_distances = self._waypoint_distances(points, self.waypoints)
        max_deviation = float(np.max(waypoint_distances))
        free_distances = waypoint_distances[1:-1]
        free_weights = self.waypoint_weights[1:-1]
        if len(free_distances):
            normalized_error = free_distances / self.max_deviation
            waypoint_cost = np.average(normalized_error**2, weights=free_weights)
            excess = np.max(np.maximum(normalized_error - 1.0, 0.0))
        else:
            waypoint_cost = 0.0
            excess = 0.0

        constraint_penalty, _ = evaluate_constraints(
            self.constraints, points, normalized_distances
        )
        fitness = (
            cam_cost
            + 10.0 * waypoint_cost
            + 1e4 * excess**2
            + constraint_penalty
        )

        curvature_change = np.gradient(curvature, distances, edge_order=2)
        max_curvature_change = np.max(np.abs(curvature_change))
        return fitness, max_deviation, max_curvature_change
