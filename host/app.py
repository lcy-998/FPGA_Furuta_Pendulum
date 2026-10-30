from __future__ import annotations

import csv
import math
import sys
import time
from collections import deque
from datetime import datetime

import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PySide6 import QtCore, QtGui, QtWidgets

from protocol import FIELDS, GAIN_DEFAULTS, Parser, TYPE_ACK, TYPE_TELEMETRY, deadband_command, gain_command, telemetry_values


class SerialWorker(QtCore.QThread):
    packet = QtCore.Signal(int, int, bytes)
    link_status = QtCore.Signal(str)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port_name = port
        self.baud = baud
        self.stop_requested = False
        self.write_queue: list[bytes] = []
        self.queue_lock = QtCore.QMutex()

    def send(self, data: bytes) -> None:
        self.queue_lock.lock()
        self.write_queue.append(data)
        self.queue_lock.unlock()

    def run(self) -> None:
        parser = Parser()
        try:
            with serial.Serial(self.port_name, self.baud, timeout=0.03, write_timeout=0.5) as port:
                self.link_status.emit(f"已连接 {self.port_name} · {self.baud} baud")
                while not self.stop_requested:
                    incoming = port.read(128)
                    for packet in parser.feed(incoming):
                        self.packet.emit(packet.frame_type, packet.sequence, packet.payload)
                    self.queue_lock.lock()
                    queued, self.write_queue = self.write_queue, []
                    self.queue_lock.unlock()
                    for item in queued:
                        port.write(item)
        except (serial.SerialException, OSError) as exc:
            self.link_status.emit(f"串口错误：{exc}")
        finally:
            self.link_status.emit("已断开")

    def stop(self) -> None:
        self.stop_requested = True
        self.wait(1000)


class PendulumView(QtWidgets.QGraphicsView):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)
        self.scene.setSceneRect(-180, -180, 360, 320)
        arm_pen = QtGui.QPen(QtGui.QColor("#51c8c1"), 9); arm_pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        pendulum_pen = QtGui.QPen(QtGui.QColor("#ffc857"), 7); pendulum_pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        self.arm = self.scene.addLine(0, 0, 90, 0, arm_pen)
        self.pendulum = self.scene.addLine(90, 0, 90, -95, pendulum_pen)
        self.scene.addEllipse(-10, -10, 20, 20, QtGui.QPen(QtCore.Qt.GlobalColor.white), QtGui.QBrush(QtGui.QColor("#e76f51")))
        self.scene.addLine(-145, 0, 145, 0, QtGui.QPen(QtGui.QColor("#516175"), 2))
        self.scene.addText("转臂与摆杆示意（非比例）", QtGui.QFont("Microsoft YaHei", 9)).setPos(-160, 112)
        self.setBackgroundBrush(QtGui.QColor("#101820"))

    def set_angles(self, alpha: float, theta: float) -> None:
        arm_length, pend_length = 90.0, 95.0
        ax, ay = arm_length * math.cos(alpha), -arm_length * math.sin(alpha)
        self.arm.setLine(0, 0, ax, ay)
        # theta=0 is upright; positive theta leans toward screen right.
        px, py = ax + pend_length * math.sin(theta), ay - pend_length * math.cos(theta)
        self.pendulum.setLine(ax, ay, px, py)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Furuta 摆 FPGA 实时监控")
        self.resize(1220, 820)
        self.worker: SerialWorker | None = None
        self.sequence = 0
        self.pending_gain_seq: int | None = None
        self.pending_deadband_seq: int | None = None
        self.paused = False
        self.rows: deque[dict[str, float | str]] = deque(maxlen=60000)
        self.start_time = time.monotonic()
        self.history: dict[str, deque[tuple[float, float]]] = {
            key: deque(maxlen=2000) for key in FIELDS
        }
        self.latest_values: dict[str, float] | None = None
        self.latest_time = 0.0
        self._build_ui()
        self.refresh_ports()
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setInterval(50)  # 20 Hz display refresh; UART remains at 100 Hz.
        self.plot_timer.timeout.connect(self.refresh_visuals)
        self.plot_timer.start()

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        bar = QtWidgets.QHBoxLayout()
        self.port_box = QtWidgets.QComboBox()
        refresh = QtWidgets.QPushButton("刷新")
        self.connect_button = QtWidgets.QPushButton("连接")
        self.status = QtWidgets.QLabel("未连接")
        bar.addWidget(QtWidgets.QLabel("串口")); bar.addWidget(self.port_box, 1)
        bar.addWidget(refresh); bar.addWidget(self.connect_button); bar.addWidget(self.status, 2)
        layout.addLayout(bar)
        refresh.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)

        body = QtWidgets.QHBoxLayout()
        layout.addLayout(body, 1)
        left = QtWidgets.QVBoxLayout(); body.addLayout(left, 3)
        self.angle_plot = pg.PlotWidget(title="角度 · 实际值与目标值 (rad)")
        self.speed_plot = pg.PlotWidget(title="角速度 · 实际值与目标值 (rad/s)")
        for plot in (self.angle_plot, self.speed_plot):
            plot.setBackground("#101820"); plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend(); plot.setLabel("bottom", "时间", units="s")
        self.angle_curves = {
            "theta": self.angle_plot.plot(pen=pg.mkPen("#ffc857", width=2), name="摆杆实际"),
            "theta_ref": self.angle_plot.plot(pen=pg.mkPen("#ffc857", style=QtCore.Qt.PenStyle.DashLine), name="摆杆目标"),
            "alpha": self.angle_plot.plot(pen=pg.mkPen("#51c8c1", width=2), name="转臂实际"),
            "alpha_ref": self.angle_plot.plot(pen=pg.mkPen("#51c8c1", style=QtCore.Qt.PenStyle.DashLine), name="转臂目标"),
        }
        self.speed_curves = {
            "dtheta": self.speed_plot.plot(pen=pg.mkPen("#ffc857", width=2), name="摆杆角速度"),
            "dtheta_ref": self.speed_plot.plot(pen=pg.mkPen("#ffc857", style=QtCore.Qt.PenStyle.DashLine), name="摆杆角速度目标"),
            "dalpha": self.speed_plot.plot(pen=pg.mkPen("#51c8c1", width=2), name="转臂角速度"),
            "dalpha_ref": self.speed_plot.plot(pen=pg.mkPen("#51c8c1", style=QtCore.Qt.PenStyle.DashLine), name="转臂角速度目标"),
        }
        left.addWidget(self.angle_plot, 1); left.addWidget(self.speed_plot, 1)
        controls = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton("暂停曲线")
        save_button = QtWidgets.QPushButton("保存 CSV")
        controls.addWidget(self.pause_button); controls.addWidget(save_button); controls.addStretch()
        left.addLayout(controls)
        self.pause_button.clicked.connect(self.toggle_pause)
        save_button.clicked.connect(self.save_csv)

        right = QtWidgets.QVBoxLayout(); body.addLayout(right, 2)
        self.animation = PendulumView(); right.addWidget(self.animation)
        self.live_values = QtWidgets.QLabel("等待遥测…")
        self.live_values.setWordWrap(True)
        self.live_values.setStyleSheet("font-size: 14px; padding: 8px; color: #e7edf3")
        right.addWidget(self.live_values)
        gain_box = QtWidgets.QGroupBox("在线反馈增益（Q10，输入实际数值）")
        gain_layout = QtWidgets.QGridLayout(gain_box)
        self.gain_inputs = []
        for idx, value in enumerate(GAIN_DEFAULTS):
            field = QtWidgets.QDoubleSpinBox(); field.setDecimals(5); field.setRange(-128.0, 127.999); field.setSingleStep(0.1); field.setValue(value)
            gain_layout.addWidget(QtWidgets.QLabel(f"K{idx + 1}"), idx // 2, (idx % 2) * 2)
            gain_layout.addWidget(field, idx // 2, (idx % 2) * 2 + 1)
            self.gain_inputs.append(field)
        self.deadband_input = QtWidgets.QSpinBox(); self.deadband_input.setRange(0, 1000); self.deadband_input.setValue(20)
        self.send_deadband = QtWidgets.QPushButton("Apply deadband")
        self.send_gains = QtWidgets.QPushButton("发送并应用")
        self.gain_result = QtWidgets.QLabel("等待参数发送")
        gain_layout.addWidget(self.send_gains, 3, 0, 1, 2); gain_layout.addWidget(self.gain_result, 3, 2, 1, 2)
        gain_layout.addWidget(QtWidgets.QLabel("Deadband"), 4, 0); gain_layout.addWidget(self.deadband_input, 4, 1); gain_layout.addWidget(self.send_deadband, 4, 2, 1, 2)
        right.addWidget(gain_box)
        self.send_gains.clicked.connect(self.send_gain_values)
        self.send_deadband.clicked.connect(self.send_deadband_value)
        self.send_gains.setEnabled(False); self.send_deadband.setEnabled(False)

    def refresh_ports(self) -> None:
        current = self.port_box.currentData()
        self.port_box.clear()
        for item in list_ports.comports():
            self.port_box.addItem(f"{item.device} · {item.description}", item.device)
        if current:
            idx = self.port_box.findData(current)
            if idx >= 0:
                self.port_box.setCurrentIndex(idx)

    def toggle_connection(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop(); self.worker = None
            self.connect_button.setText("连接")
            self.send_gains.setEnabled(False); self.send_deadband.setEnabled(False)
            self.pending_gain_seq = None; self.pending_deadband_seq = None
            return
        port = self.port_box.currentData()
        if not port:
            self.status.setText("没有可用串口"); return
        self.worker = SerialWorker(port)
        self.worker.packet.connect(self.on_packet)
        self.worker.link_status.connect(self.status.setText)
        self.worker.start()
        self.connect_button.setText("断开")
        self.send_gains.setEnabled(True); self.send_deadband.setEnabled(True)

    @QtCore.Slot(int, int, bytes)
    def on_packet(self, kind: int, sequence: int, payload: bytes) -> None:
        if kind == TYPE_TELEMETRY:
            try:
                values = telemetry_values(payload)
            except ValueError as exc:
                self.status.setText(str(exc)); return
            now = time.monotonic() - self.start_time
            values["time"] = now
            values["wall_time"] = datetime.now().isoformat(timespec="milliseconds")
            self.rows.append(values.copy())
            if self.paused:
                return
            for key in FIELDS:
                self.history[key].append((now, values[key]))
            self.latest_values = values
            self.latest_time = now
        elif kind == TYPE_ACK and len(payload) == 1:
            if (self.pending_gain_seq is None or sequence != self.pending_gain_seq) and (self.pending_deadband_seq is None or sequence != self.pending_deadband_seq):
                return
            if payload[0] == 0:
                self.gain_result.setText(f"序号 {sequence}：已应用")
                self.gain_result.setStyleSheet("color: #51c8c1")
            else:
                self.gain_result.setText(f"序号 {sequence}：FPGA 拒绝，范围无效")
                self.gain_result.setStyleSheet("color: #ff6b6b")
            self.pending_gain_seq = None; self.pending_deadband_seq = None
            self.send_gains.setEnabled(True); self.send_deadband.setEnabled(True)

    @QtCore.Slot()
    def refresh_visuals(self) -> None:
        values = self.latest_values
        if self.paused or values is None:
            return
        for key, curve in self.angle_curves.items():
            points = self.history[key]
            curve.setData([p[0] for p in points], [p[1] for p in points])
        for key, curve in self.speed_curves.items():
            points = self.history[key]
            curve.setData([p[0] for p in points], [p[1] for p in points])
        self.angle_plot.setXRange(max(0.0, self.latest_time - 20.0), max(20.0, self.latest_time), padding=0)
        self.speed_plot.setXRange(max(0.0, self.latest_time - 20.0), max(20.0, self.latest_time), padding=0)
        self.animation.set_angles(values["alpha"], values["theta"])
        self.live_values.setText(
            f"摆杆 θ {values['theta']:+.3f} / 目标 {values['theta_ref']:+.3f} rad\n"
            f"摆杆速度 {values['dtheta']:+.3f} / 目标 {values['dtheta_ref']:+.3f} rad/s\n"
            f"转臂 α {values['alpha']:+.3f} / 目标 {values['alpha_ref']:+.3f} rad\n"
            f"转臂速度 {values['dalpha']:+.3f} / 目标 {values['dalpha_ref']:+.3f} rad/s"
        )

    def send_gain_values(self) -> None:
        if not self.worker or not self.worker.isRunning():
            self.gain_result.setText("请先连接串口"); return
        try:
            packet = gain_command(self.sequence, [field.value() for field in self.gain_inputs])
        except ValueError as exc:
            self.gain_result.setText(str(exc)); return
        self.pending_gain_seq = self.sequence
        self.worker.send(packet)
        self.send_gains.setEnabled(False); self.send_deadband.setEnabled(False)
        self.gain_result.setText(f"已发送序号 {self.sequence}，等待 FPGA 应答…")
        self.sequence = (self.sequence + 1) & 0xFF

    def send_deadband_value(self) -> None:
        if not self.worker or not self.worker.isRunning(): return
        self.pending_deadband_seq = self.sequence
        self.worker.send(deadband_command(self.sequence, self.deadband_input.value()))
        self.sequence = (self.sequence + 1) & 0xFF

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_button.setText("继续曲线" if self.paused else "暂停曲线")

    def save_csv(self) -> None:
        if not self.rows:
            self.status.setText("暂无数据可保存"); return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存遥测数据", "furuta_telemetry.csv", "CSV 文件 (*.csv)")
        if not filename:
            return
        columns = ["wall_time", "time", *FIELDS, "pwm_duty"]
        with open(filename, "w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader(); writer.writerows(self.rows)
        self.status.setText(f"已保存 {len(self.rows)} 条记录")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop()
        event.accept()


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(); window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
