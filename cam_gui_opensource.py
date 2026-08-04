import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QTextEdit, QFileDialog, QMessageBox, QFrame,
                             QGridLayout, QGroupBox, QSpinBox, QDoubleSpinBox,
                             QComboBox)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtGui import QPixmap, QColor, QLinearGradient, QBrush, QPalette
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
from cam_calc import (CamParameters, calculate_flat_cam, curvature_from_radius,
                      map_waypoints_to_cam_angles, signed_curvature_radius)
plt.rcParams['font.sans-serif'] = ['SimHei']  # 显示中文
plt.rcParams['axes.unicode_minus'] = False


def save_path_table(file_path, title, points, curvature):
    """按统一六列格式保存路径离散点。"""
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    distances /= distances[-1]
    data = np.column_stack((
        np.arange(len(points)), points, curvature,
        signed_curvature_radius(curvature), distances
    ))
    with open(file_path, "w") as file:
        file.write(title + "\n")
        file.write(
            "Index\tX(mm)\tY(mm)\tCurvature\t"
            "Curvature_Radius(mm)\tNormalized_Distance\n"
        )
        np.savetxt(
            file,
            data,
            delimiter="\t",
            fmt=["%d", "%.4f", "%.4f", "%.9f", "%.6f", "%.9f"],
        )
    return len(points)


#####
## 凸轮设计部分
#####
class CamDesignWindow(QWidget):
    """凸轮设计窗口"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("凸轮设计")
        self.setGeometry(100, 100, 1200, 800)
        self.waypoints = None  # 打卡点数据
        self.path_data = None  # 路径点数据
        self.cam_data = None  # 凸轮设计结果
        self.cam_params = None
        self.waypoint_angles = []
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
        param_specs = [
            ("M", "车总宽M(mm):", 150.0, 0.001, 10000.0),
            ("m", "车把偏置距离m(mm):", 30.0, -1000.0, 1000.0),
            ("E", "车把长度E(mm):", 20.0, 0.001, 1000.0),
            ("L", "轴距L(mm):", 107.0, 0.001, 10000.0),
            ("e", "凸轮基圆半径e(mm):", 73.02, 0.001, 10000.0),
            ("n", "期望传动比n:", 42.0, 0.001, 10000.0),
            ("r0", "后轮半径r0(mm):", 65.0, 0.001, 10000.0),
            ("d", "推杆直径d(mm):", 2.0, 0.001, 1000.0),
            ("d_cam", "凸轮总厚度d_cam(mm):", 3.0, 0.001, 1000.0),
        ]
        self.param_spins = {}
        controls = []
        for name, label, value, minimum, maximum in param_specs:
            spin = QDoubleSpinBox()
            spin.setRange(minimum, maximum)
            spin.setDecimals(3)
            spin.setValue(value)
            self.param_spins[name] = spin
            controls.append((label, spin))

        self.putter_type_combo = QComboBox()
        self.putter_type_combo.addItem("平推/顶摆杆（其他类型预留）", "flat")
        controls.append(("推杆类型:", self.putter_type_combo))

        self.cam_direction_combo = QComboBox()
        self.cam_direction_combo.addItem("左拐（凸轮凹下）", 1)
        self.cam_direction_combo.addItem("右拐", -1)
        controls.append(("凸轮转向:", self.cam_direction_combo))

        self.closure_combo = QComboBox()
        self.closure_combo.addItem("线性回到起始半径", "linear")
        self.closure_combo.addItem("保持末端后径向闭合", "hold")
        self.closure_combo.addItem("不补全（仅工作段）", "none")
        controls.append(("闭环补全:", self.closure_combo))

        for i, (label, widget) in enumerate(controls):
            row = i // 2
            col = (i % 2) * 2
            param_layout.addWidget(QLabel(label), row, col)
            param_layout.addWidget(widget, row, col + 1)
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
        self.log_text.append("       0\t5588.0\t713.0\t0.000000\tinf\t0.000000")
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
        self.log_text.append("8. 推杆直径d(mm): 平推模式仅记录，实际包络偏置预留")
        self.log_text.append("9. 凸轮总厚度d_cam(mm): 凸轮总厚度")
        self.log_text.append("10. 推杆类型: 当前仅实现平推/顶摆杆")
        self.log_text.append("11. 凸轮转向: 左拐(凸轮凹下)或右拐")
        self.log_text.append("12. 闭环补全: 线性、保持末端或不补全")
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
            self.cam_data = None
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

    def _selected_path_columns(self, include_radius=False):
        """读取当前列号；列号在界面中从 1 开始。"""
        columns = [self.x_col_spin.value() - 1, self.y_col_spin.value() - 1]
        if include_radius:
            columns.append(self.r_col_spin.value() - 1)
        if max(columns) >= self.path_data.shape[1]:
            QMessageBox.warning(self, "列选择错误", "选择的列索引超出范围")
            return None
        return columns

    def _current_cam_parameters(self):
        values = {name: spin.value() for name, spin in self.param_spins.items()}
        return CamParameters(
            **values,
            direction=self.cam_direction_combo.currentData(),
            closure_mode=self.closure_combo.currentData(),
        )

    def draw_path_plot(self):
        """绘制路径图"""
        if self.path_data is None:
            return
        columns = self._selected_path_columns()
        if columns is None:
            return
        # 提取坐标
        X = self.path_data[:, columns[0]]
        Y = self.path_data[:, columns[1]]
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
        half_width = self.param_spins["M"].value() / 2
        X1, Y1 = X + half_width * nx, Y + half_width * ny
        X2, Y2 = X - half_width * nx, Y - half_width * ny
        # 绘制路径
        self.ax_path.clear()
        # 绘制中点轨迹
        self.ax_path.plot(X, Y, 'b-', linewidth=1.5, label='中点轨迹')
       # 绘制左右后轮轨迹
        self.ax_path.plot(X1, Y1, 'g--', linewidth=1.0, label='左后轮轨迹')
        self.ax_path.plot(X2, Y2, 'r--', linewidth=1.0, label='右后轮轨迹')
        if self.cam_data is not None and len(self.cam_data["virtual_path"]):
            virtual_path = np.vstack(([[X[-1], Y[-1]]], self.cam_data["virtual_path"]))
            self.ax_path.plot(
                virtual_path[:, 0], virtual_path[:, 1], 'k--', linewidth=1.4,
                label='凸轮补全对应虚拟轨迹'
            )
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
            columns = self._selected_path_columns(include_radius=True)
            if columns is None:
                return

            path_x = self.path_data[:, columns[0]]
            path_y = self.path_data[:, columns[1]]
            curvature_radius = self.path_data[:, columns[2]]
            curvature = curvature_from_radius(curvature_radius)
            self.cam_params = self._current_cam_parameters()
            self.cam_data = calculate_flat_cam(
                path_x, path_y, curvature, self.cam_params
            )
            self.waypoint_angles = map_waypoints_to_cam_angles(
                self.cam_data["path_x"], self.cam_data["path_y"],
                self.waypoints, self.cam_data["work_angles"]
            )
            # 显示结果
            self.log_text.append("凸轮设计完成!")
            self.log_text.append(f"极径最大值: {self.cam_data['max_cam_radius']:.2f} mm")
            self.log_text.append(f"极径最小值: {self.cam_data['min_cam_radius']:.2f} mm")
            self.log_text.append(f"路径总长: {self.cam_data['total_length']:.2f} mm")
            self.log_text.append(
                f"凸轮转过总角度: {np.degrees(self.cam_data['total_work_angle']):.2f} 度"
            )
            # 更新结果标签
            self.result_label.setText(
                f"结果: 极径范围 [{self.cam_data['min_cam_radius']:.2f}, "
                f"{self.cam_data['max_cam_radius']:.2f}] mm\n"
                f"路径总长: {self.cam_data['total_length']:.2f} mm\n"
                f"凸轮转角: {np.degrees(self.cam_data['total_work_angle']):.2f} 度"
            )
            # 启用保存按钮
            self.save_button.setEnabled(True)
            # 绘制凸轮廓线
            self.draw_cam_profile()
            self.draw_path_plot()
            # 显示打卡点对应极角
            self.log_waypoint_angles()
        except Exception as e:
            QMessageBox.critical(self, "凸轮设计错误", f"设计凸轮时发生错误: {str(e)}")
    def log_waypoint_angles(self):
        """记录打卡点对应极角"""
        if not self.waypoint_angles:
            return
        self.log_text.append("\n打卡点对应极角:")
        for result in self.waypoint_angles:
            self.log_text.append(
                f"打卡点 {result['waypoint_idx'] + 1}: "
                f"位置({result['waypoint'][0]:.2f}, {result['waypoint'][1]:.2f}) "
                f"对应极角: {result['angle_deg']:.2f}°"
            )
    def draw_cam_profile(self):
        """绘制凸轮廓线"""
        if self.cam_data is None:
            return
        cam_radii = self.cam_data['cam_radii']
        work_angles = self.cam_data['work_angles']
        work_radii = self.cam_data['work_radii']
        base_radius = self.cam_data['base_radius']
        self.ax_cam.clear()
        self.ax_cam.plot(work_angles, work_radii, color='#16a34a', linewidth=1.8, label='工作段')
        if len(self.cam_data['closure_angles']):
            self.ax_cam.plot(
                self.cam_data['closure_angles'], self.cam_data['closure_radii'],
                color='#6b7280', linestyle='--', linewidth=1.3, label='补全段'
            )
        base_theta = np.linspace(0, 2 * np.pi, 100)
        base_rou = np.full_like(base_theta, base_radius)
        self.ax_cam.plot(base_theta, base_rou, 'b--', label='基圆')
        self.ax_cam.plot(work_angles[0], work_radii[0], marker='*', color='#111827',
                         markersize=10, label='起点')
        self.ax_cam.plot(work_angles[-1], work_radii[-1], marker='s', color='#f97316',
                         markersize=6, label='工作终点')
        self.ax_cam.plot([0, 0], [0, max(cam_radii)], color='#9ca3af', linestyle=':', linewidth=0.8)
        self.ax_cam.set_title('平推凸轮轮廓')
        # 标记打卡点位置
        if self.waypoint_angles:
            for result in self.waypoint_angles:
                self.ax_cam.plot(
                    result['angle_rad'],
                    work_radii[result['nearest_idx']],
                    'ro', markersize=5
                )
                self.ax_cam.text(
                    result['angle_rad'],
                    work_radii[result['nearest_idx']] + 5,
                    f"WP{result['waypoint_idx'] + 1}",
                    fontsize=8
                )
        self.ax_cam.legend(loc='upper right', fontsize=8)
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
            if not file_path.lower().endswith('.txt'):
                file_path += '.txt'
            angles = self.cam_data["cam_angles"]
            radii = self.cam_data["cam_radii"]
            xyz = np.column_stack((
                radii * np.cos(angles),
                radii * np.sin(angles),
                np.zeros(len(angles)),
            ))
            np.savetxt(file_path, xyz, delimiter="\t", fmt="%.6f")
            self.log_text.append(
                f"凸轮XYZ数据已保存至: {file_path} (共{len(xyz)}个点)"
            )
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
        """路径AI设计——暂未开放，敬请关注后续更新。"""
        msg = (
            "<h3>路径AI设计功能暂未开放</h3>"
            "<p>后续更新敬请期待！</p>"
            "<hr>"
            "<p><b>GitHub:</b> <a href='https://github.com/wangxt888'>github.com/wangxt888</a></p>"
            "<p><b>Gitee:</b> <a href='https://gitee.com/wangxt888'>gitee.com/wangxt888</a></p>"
            "<hr>"
            "<p>欢迎 Star 和关注，第一时间获取更新通知。</p>"
        )
        QMessageBox.information(self, "路径AI设计", msg)
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
        <p><b>路径AI设计：</b>使用人工智能算法优化凸轮路径，生成平滑的凸轮轮廓。此功能暂未开放，后续更新敬请期待。</p>
        <p><b>凸轮设计：</b>基于路径数据设计凸轮几何形状。请在使用时认真查看凸轮设计内的左侧说明以及参数定义。</p>
        <p><b>使用流程：</b></p>
        <ol>
            <li>在 UG 等软件中生成路径样条曲线并导出路径点数据</li>
            <li>打开凸轮设计窗口</li>
            <li>导入打卡点数据和路径数据</li>
            <li>设置凸轮参数</li>
            <li>点击"运行凸轮设计"</li>
            <li>保存凸轮XYZ数据</li>
        </ol>
        <hr>
        <p>GitHub: <a href='https://github.com/wangxt888'>github.com/wangxt888</a></p>
        <p>Gitee: <a href='https://gitee.com/wangxt888'>gitee.com/wangxt888</a></p>
        """
        QMessageBox.information(self, "软件说明", help_text)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
