from dataclasses import dataclass
from typing import Tuple

import numpy as np
import scipy.interpolate as spl


EPSILON = 1e-8
CURVATURE_EPSILON = 1e-12
FULL_TURN = 2.0 * np.pi


@dataclass(frozen=True)
class CamParameters:
    M: float = 150.0
    m: float = 30.0
    E: float = 20.0
    L: float = 107.0
    e: float = 73.02
    n: float = 42.0
    r0: float = 65.0
    d: float = 2.0
    d_cam: float = 3.0
    direction: int = 1
    closure_mode: str = "linear"


def create_clamped_knots(control_points: np.ndarray, degree: int) -> np.ndarray:
    """按控制多边形弦长创建定义域为 [0,1] 的夹持节点向量。"""
    segment_lengths = np.linalg.norm(np.diff(control_points, axis=0), axis=1)
    if np.sum(segment_lengths) <= EPSILON:
        parameters = np.linspace(0.0, 1.0, len(control_points))
    else:
        parameters = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        parameters /= parameters[-1]

    internal_count = len(control_points) - degree - 1
    internal_knots = np.array([
        np.mean(parameters[index:index + degree])
        for index in range(1, internal_count + 1)
    ])
    return np.concatenate((
        np.zeros(degree + 1),
        internal_knots,
        np.ones(degree + 1),
    ))


def create_parametric_spline(
    control_points: np.ndarray,
    degree: int = 3,
    knots: np.ndarray = None,
) -> spl.BSpline:
    """用真实控制点和共同节点向量构造二维夹持 B 样条。"""
    points = np.asarray(control_points, dtype=float)
    degree = min(int(degree), 5, len(points) - 1)
    if degree < 1:
        raise ValueError("至少需要两个控制点")
    if not np.isfinite(points).all():
        raise ValueError("控制点中包含 NaN 或无穷值")
    if knots is None:
        knots = create_clamped_knots(points, degree)
    else:
        knots = np.asarray(knots, dtype=float)
    return spl.BSpline(knots, points, degree, axis=0)


def calculate_signed_curvature(spline, t: np.ndarray) -> np.ndarray:
    """计算带符号曲率；在标准 XY 坐标系中，正值表示左转。"""
    first_derivative = np.asarray(spline(t, nu=1), dtype=float)
    second_derivative = np.asarray(spline(t, nu=2), dtype=float)
    dx, dy = first_derivative[..., 0], first_derivative[..., 1]
    ddx, ddy = second_derivative[..., 0], second_derivative[..., 1]
    numerator = dx * ddy - dy * ddx
    denominator = (dx**2 + dy**2) ** 1.5
    curvature = np.zeros_like(denominator, dtype=float)
    np.divide(
        numerator,
        denominator,
        out=curvature,
        where=denominator > EPSILON,
    )
    return curvature


def signed_curvature_radius(
    curvature: np.ndarray,
) -> np.ndarray:
    """把曲率转换为带符号半径；零曲率明确使用正无穷。"""
    values = np.asarray(curvature, dtype=float)
    radius = np.full(values.shape, np.inf, dtype=float)
    nonzero = np.abs(values) > CURVATURE_EPSILON
    np.divide(1.0, values, out=radius, where=nonzero)
    return radius


def curvature_from_radius(curvature_radius: np.ndarray) -> np.ndarray:
    """导入旧路径文件时，把曲率半径转换为带符号曲率。"""
    radius = np.asarray(curvature_radius, dtype=float)
    if np.isnan(radius).any():
        raise ValueError("曲率半径中包含空值")
    curvature = np.zeros_like(radius)
    valid = np.isfinite(radius) & (np.abs(radius) > EPSILON)
    curvature[valid] = 1.0 / radius[valid]
    return curvature


def flat_follower_radius_from_curvature(
    curvature: np.ndarray,
    params: CamParameters,
) -> np.ndarray:
    """根据路径带符号曲率计算凸轮的理论极径。"""
    values = np.asarray(curvature, dtype=float)
    denominator = 1.0 - params.m * values
    if np.any(np.abs(denominator) <= EPSILON):
        indices = np.flatnonzero(np.abs(denominator) <= EPSILON)[:5]
        raise ValueError(f"凸轮转向公式在采样点 {indices.tolist()} 附近出现奇点")
###e=e-EL/m-R(E左正)
    steering_tangent = params.L * values / denominator
    radius = params.e - params.direction * params.E * steering_tangent
    invalid = (~np.isfinite(radius)) | (radius <= 0)
    if np.any(invalid):
        indices = np.flatnonzero(invalid)[:5]
        raise ValueError(f"凸轮极径在采样点 {indices.tolist()} 处无效")
    return radius


def complete_cam_profile(
    work_angles: np.ndarray,
    work_radii: np.ndarray,
    mode: str = "linear",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """把开环工作轮廓补全到一整圈，并单独返回补全段。"""
    work_angles = np.asarray(work_angles, dtype=float)
    work_radii = np.asarray(work_radii, dtype=float)
    if mode == "none" or work_angles[-1] >= FULL_TURN - EPSILON:
        return work_angles, work_radii, np.array([]), np.array([])

    work_angle = max(work_angles[-1], EPSILON)
    remaining_angle = FULL_TURN - work_angles[-1]
    samples_per_radian = max(64.0, len(work_angles) / work_angle)
    closure_count = min(
        100000, max(2, int(np.ceil(remaining_angle * samples_per_radian)))
    )
    closure_angles = np.linspace(work_angles[-1], FULL_TURN, closure_count + 1)[1:]

    if mode == "linear":
        closure_radii = np.linspace(
            work_radii[-1], work_radii[0], closure_count + 1
        )[1:]
    else:
        closure_radii = np.full(closure_count, work_radii[-1], dtype=float)
        # 保持末端半径时，最后增加一条可见的径向线连接起点。
        closure_angles = np.append(closure_angles, FULL_TURN)
        closure_radii = np.append(closure_radii, work_radii[0])

    cam_angles = np.concatenate((work_angles, closure_angles))
    cam_radii = np.concatenate((work_radii, closure_radii))
    return cam_angles, cam_radii, closure_angles, closure_radii


def virtual_path_from_cam(
    path_x: np.ndarray,
    path_y: np.ndarray,
    work_angle_end: float,
    work_radius_end: float,
    closure_angles: np.ndarray,
    closure_radii: np.ndarray,
    params: CamParameters,
) -> Tuple[np.ndarray, np.ndarray]:
    """从补全段凸轮极径反算车辆在真实路径之后的虚拟轨迹。"""
    if len(closure_angles) == 0:
        return np.empty((0, 2)), np.array([])

    angles = np.concatenate(([work_angle_end], closure_angles))
    radii = np.concatenate(([work_radius_end], closure_radii))
    steering_tangent = (params.e - radii) / (params.direction * params.E)
    curvature = steering_tangent / (params.L + params.m * steering_tangent)

    dx = path_x[-1] - path_x[-2]
    dy = path_y[-1] - path_y[-2]
    heading = np.arctan2(dy, dx)
    distance = params.n * params.r0 * np.diff(angles)
    middle_curvature = 0.5 * (curvature[:-1] + curvature[1:])
    heading_change = middle_curvature * distance
    heading_at_start = heading + np.concatenate((
        [0.0], np.cumsum(heading_change[:-1])
    ))
    middle_heading = heading_at_start + 0.5 * heading_change
    increments = distance[:, None] * np.column_stack((
        np.cos(middle_heading), np.sin(middle_heading)
    ))
    virtual_points = np.array([path_x[-1], path_y[-1]]) + np.cumsum(
        increments, axis=0
    )
    return virtual_points, curvature[1:]


def _prepare_path(
    x: np.ndarray,
    y: np.ndarray,
    curvature: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    if not (len(x) == len(y) == len(curvature)) or len(x) < 2:
        raise ValueError("X、Y 和曲率数组必须等长且至少包含两个点")
    if not np.isfinite(np.column_stack((x, y, curvature))).all():
        raise ValueError("路径数据中包含 NaN 或无穷值")

    segment_lengths = np.hypot(np.diff(x), np.diff(y))
    keep = np.concatenate(([True], segment_lengths > EPSILON))
    x, y, curvature = x[keep], y[keep], curvature[keep]
    if len(x) < 2:
        raise ValueError("路径总长度必须大于零")
    segment_lengths = np.hypot(np.diff(x), np.diff(y))
    #相对误差：≈ (Δθ)² / 24
    distance = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total_length = float(distance[-1])
    return x, y, curvature, distance, total_length


def calculate_flat_cam(
    x: np.ndarray,
    y: np.ndarray,
    curvature: np.ndarray,
    params: CamParameters,
) -> dict:
    """计算平推/顶摆杆凸轮的工作段，并生成一整圈闭环轮廓。"""
    x, y, curvature, distance, total_length = _prepare_path(
        x, y, curvature
    )
    total_work_angle = total_length / (params.n * params.r0)
    if total_work_angle > FULL_TURN + EPSILON:
        raise ValueError(
            "凸轮工作段超过 360°，请增大传动比 n 或后轮半径 r0"
        )

    work_angles = distance / total_length * total_work_angle
    work_radii = flat_follower_radius_from_curvature(curvature, params)
    cam_angles, cam_radii, closure_angles, closure_radii = complete_cam_profile(
        work_angles, work_radii, params.closure_mode
    )
    virtual_path, closure_curvature = virtual_path_from_cam(
        x, y, work_angles[-1], work_radii[-1],
        closure_angles, closure_radii, params
    )

    # 当前平推/顶摆杆模式输出理论接触轮廓。
    # 按杆半径生成实际加工包络的功能留给后续从动件模型实现。
    profile_polar = np.column_stack((cam_angles, cam_radii))

    return {
        "cam_angles": cam_angles,
        "cam_radii": cam_radii,
        "work_angles": work_angles,
        "work_radii": work_radii,
        "closure_angles": closure_angles,
        "closure_radii": closure_radii,
        "profile_polar": profile_polar,
        "base_radius": params.e,
        "total_length": total_length,
        "max_cam_radius": float(np.max(cam_radii)),
        "min_cam_radius": float(np.min(cam_radii)),
        "total_work_angle": total_work_angle,
        "path_x": x,
        "path_y": y,
        "path_distances": distance,
        "curvature": curvature,
        "virtual_path": virtual_path,
        "closure_curvature": closure_curvature,
    }


def map_waypoints_to_cam_angles(x, y, waypoints, work_angles):
    """找到各打卡点在路径上的最近位置及对应凸轮角度。"""
    path_points = np.column_stack((x, y))
    results = []
    for index, waypoint in enumerate(waypoints):
        distances = np.linalg.norm(path_points - waypoint, axis=1)
        nearest = int(np.argmin(distances))
        angle = work_angles[nearest]
        results.append({
            "waypoint_idx": index,
            "waypoint": waypoint,
            "nearest_point": path_points[nearest],
            "nearest_idx": nearest,
            "angle_rad": angle,
            "angle_deg": np.degrees(angle),
            "distance": distances[nearest],
        })
    return results
