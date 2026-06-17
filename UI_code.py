import sys
import math
import numpy as np
import paho.mqtt.client as mqtt
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QGraphicsScene, QGraphicsView,
    QGraphicsRectItem, QGraphicsTextItem,
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QTextBrowser, QListWidget,
    QPushButton, QFileDialog, QMessageBox
)

from PyQt6.QtGui import (
    QBrush, QPen, QColor, QFont,
    QPainter, QPixmap, QPolygonF
)

from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal,
    QTimer, QPointF
)

# ---------------- CONFIG ----------------
SCALE = 4
ROBOT_SIZE = 40
SENSOR_OFFSET_CM = 6.0

# Odometry calibration. The firmware streams ODOM,left,right as raw stepper
# step counts; this converts steps -> cm for pose integration.
# Per the 5EID0 manual (sec 4.3): ONE FULL WHEEL ROTATION = 1600 steps.
# So CM_PER_STEP = 2*pi*WHEEL_RADIUS / 1600. The old handler used /512 (wrong:
# 1600/512 = 3.125x too large -> 0.046 cm/step), which was the odometry error.
# With the manual's 1600 and a ~3.1 cm wheel radius this is ~0.012 cm/step.
# Measure the wheel radius on hardware and set it here.
WHEEL_RADIUS_CM = 3.13
STEPS_PER_REV   = 1600                                  # manual sec 4.3
CM_PER_STEP     = 2 * math.pi * WHEEL_RADIUS_CM / STEPS_PER_REV   # ~0.0123
WHEEL_BASE_CM   = 12.5     # center-to-center; used for heading integration

# --- NEON COLOR PALETTE ---
CYAN = "#00FFFF"
MAGENTA = "#FF00FF"
AMBER = "#FFAB00"
RED_ALERT = "#FF2A2A"
GRID_BLUE = "#101D35"
PANEL_BG = "#11111A"
APP_BG = "#050509"

# ---------------- STYLESHEET (FUTURISTIC MINIMALIST) ----------------
STYLESHEET = f"""
QMainWindow {{ background-color: {APP_BG}; }}

QGroupBox {{
    background-color: {PANEL_BG};
    border: none;
    border-radius: 10px;
    margin-top: 25px;
    font-weight: bold;
    color: #FFFFFF;
    font-size: 14px;
    font-family: "Segoe UI", "Helvetica Neue", sans-serif;
}}

QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0px 10px;
    color: #4A5568;
    letter-spacing: 2px;
}}

QLabel {{
    color: #A0AEC0;
    background-color: transparent;
    font-family: "Consolas", monospace;
    font-size: 13px;
}}

QTextBrowser, QListWidget {{
    background-color: #0A0A0F;
    border: 1px solid #1A1A24;
    border-radius: 6px;
    color: #E2E8F0;
    font-family: "Consolas", monospace;
    font-size: 12px;
    padding: 8px;
}}

/* Custom Scrollbar for a sleek look */
QScrollBar:vertical {{
    border: none;
    background: #0A0A0F;
    width: 8px;
    border-radius: 4px;
}}
QScrollBar::handle:vertical {{
    background: #2D3748;
    min-height: 20px;
    border-radius: 4px;
}}

QPushButton {{
    background-color: #1A202C;
    color: {CYAN};
    border-radius: 6px;
    border: 1px solid {CYAN};
    padding: 8px;
    font-family: "Segoe UI", sans-serif;
    font-weight: bold;
    letter-spacing: 1px;
}}

QPushButton:hover {{
    background-color: {CYAN};
    color: #000000;
}}
"""

# ---------------- ROCK ----------------
class RockSample(QGraphicsRectItem):
    def __init__(self, x, y, size_cm, color_str, temp):
        display_size = size_cm * SCALE
        super().__init__(0, 0, display_size, display_size)
        self.setPos(x * SCALE - display_size / 2, y * SCALE - display_size / 2)
        
        # Give rocks a slightly glowing border
        self.setBrush(QBrush(QColor(color_str)))
        self.setPen(QPen(QColor("#FFFFFF"), 1))
        self.setToolTip(f"Rock\nColor: {color_str}\nSize: {size_cm}cm\nTemp: {temp}°C")

# ---------------- MQTT ----------------
class MQTTWorker(QThread):
    message_received = pyqtSignal(str, str)
    def __init__(self, robot_name, username, password, topic):
        super().__init__()
        self.robot_name = robot_name
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.username_pw_set(username, password)
        self.topic = topic

    def run(self):
        BROKER = "mqtt.ics.ele.tue.nl"
        def on_message(client, userdata, msg):
            payload = msg.payload.decode("utf-8")
            self.message_received.emit(self.robot_name, payload)
        self.client.on_message = on_message
        self.client.connect(BROKER, 1883, 60)
        self.client.subscribe(self.topic)
        self.client.loop_forever()

    def publish_message(self, topic, payload):
        self.client.publish(topic, payload)

# ---------------- DASHBOARD ----------------
class VenusDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Venus Ground Station // SYSTEM SECURE")
        self.resize(1300, 1000)
        self.setStyleSheet(STYLESHEET)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20) # Add breathing room to the edges
        main_layout.setSpacing(20)

        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0,0,0,0)

        right_col = QWidget()
        right_layout = QVBoxLayout(right_col)
        right_layout.setContentsMargins(0,0,0,0)

        main_layout.addWidget(left_col, stretch=3)
        main_layout.addWidget(right_col, stretch=1)

        self.setup_map_panel(left_layout)
        self.setup_log_panel(left_layout)

        self.setup_telemetry_panel(right_layout)
        self.setup_findings_panel(right_layout)
        self.setup_controls_panel(right_layout)

        # Timers
        self.timer41 = QTimer()
        self.timer80 = QTimer()
        self.timer41.timeout.connect(lambda: self.set_offline("Robot41"))
        self.timer80.timeout.connect(lambda: self.set_offline("Robot80"))

        # MQTT
        self.workerA = MQTTWorker("Robot41", "robot_41_1", "t7gIhbJF", "/pynqbridge/41/send")
        self.workerB = MQTTWorker("Robot80", "robot_80_1", "OQfY8Km2", "/pynqbridge/80/send")
        self.workerA.message_received.connect(self.handle_message)
        self.workerB.message_received.connect(self.handle_message)
        self.workerA.start()
        self.workerB.start()

    # ---------------- UI ----------------
    def setup_map_panel(self, parent_layout):
        group = QGroupBox("TACTICAL MAP")
        layout = QVBoxLayout(group)
        self.scene = QGraphicsScene(-150 * SCALE, -150 * SCALE, 300 * SCALE, 300 * SCALE)
        self.view = QGraphicsView(self.scene)
        
        # Pure black map background for high contrast
        self.view.setBackgroundBrush(QBrush(QColor("#000000")))
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setStyleSheet("border: none; border-radius: 6px;")
        
        layout.addWidget(self.view)
        parent_layout.addWidget(group, stretch=2)
        self.draw_guides()
        self.initialize_robots()

    def setup_log_panel(self, parent_layout):
        group = QGroupBox("DATA STREAM")
        layout = QVBoxLayout(group)
        self.log_widget = QTextBrowser()
        layout.addWidget(self.log_widget)
        parent_layout.addWidget(group, stretch=1)

    def setup_telemetry_panel(self, parent_layout):
        group = QGroupBox("TELEMETRY")
        layout = QVBoxLayout(group)

        self.status_41_label = QLabel("OFFLINE")
        self.pos_41_label = QLabel("X: --- | Y: ---")
        layout.addWidget(QLabel(f"<span style='color:{CYAN};'>■</span> UNIT_41 [ALPHA]"))
        layout.addWidget(self.status_41_label)
        layout.addWidget(self.pos_41_label)

        self.status_80_label = QLabel("OFFLINE")
        self.pos_80_label = QLabel("X: --- | Y: ---")
        layout.addWidget(QLabel(f"<br><span style='color:{MAGENTA};'>■</span> UNIT_80 [BRAVO]"))
        layout.addWidget(self.status_80_label)
        layout.addWidget(self.pos_80_label)

        parent_layout.addWidget(group)

    def setup_findings_panel(self, parent_layout):
        group = QGroupBox("ANOMALY LOG")
        layout = QVBoxLayout(group)
        self.findings_list = QListWidget()
        layout.addWidget(self.findings_list)
        parent_layout.addWidget(group)

    def setup_controls_panel(self, parent_layout):
        group = QGroupBox("SYS_COMMS")
        layout = QVBoxLayout(group)
        
        btn_fit = QPushButton("CALIBRATE VIEW")
        btn_fit.clicked.connect(self.zoom_to_fit)
        
        btn_clear = QPushButton("PURGE DATA")
        btn_clear.clicked.connect(self.clear_all)
        
        layout.addWidget(btn_fit)
        layout.addWidget(btn_clear)
        parent_layout.addWidget(group)

    # ---------------- MAP ----------------
    def draw_guides(self):
        # Subtle radar-style blue grid instead of gray
        grid_pen = QPen(QColor(GRID_BLUE), 1)
        for i in range(-150, 151, 10):
            self.scene.addLine(i * SCALE, -150 * SCALE, i * SCALE, 150 * SCALE, grid_pen)
            self.scene.addLine(-150 * SCALE, i * SCALE, 150 * SCALE, i * SCALE, grid_pen)
            
        # Draw a bright origin crosshair
        origin_pen = QPen(QColor("#3A506B"), 2)
        self.scene.addLine(-10*SCALE, 0, 10*SCALE, 0, origin_pen)
        self.scene.addLine(0, -10*SCALE, 0, 10*SCALE, origin_pen)

    def initialize_robots(self):
        # Robot 41: Cyan
        poly41 = QPolygonF([QPointF(0, -ROBOT_SIZE/2), QPointF(ROBOT_SIZE/2.5, ROBOT_SIZE/2), QPointF(0, ROBOT_SIZE/4), QPointF(-ROBOT_SIZE/2.5, ROBOT_SIZE/2)])
        self.robot41_marker = self.scene.addPolygon(poly41, QPen(QColor(CYAN), 2), QBrush(QColor("#002222")))
        self.robot41_marker.setTransformOriginPoint(0, 0)

        # Robot 80: Magenta
        poly80 = QPolygonF([QPointF(0, -ROBOT_SIZE/2), QPointF(ROBOT_SIZE/2.5, ROBOT_SIZE/2), QPointF(0, ROBOT_SIZE/4), QPointF(-ROBOT_SIZE/2.5, ROBOT_SIZE/2)])
        self.robot80_marker = self.scene.addPolygon(poly80, QPen(QColor(MAGENTA), 2), QBrush(QColor("#220022")))
        self.robot80_marker.setTransformOriginPoint(0, 0)

        self.robot41_marker.setPos(0, 0)
        self.robot80_marker.setPos(0, 0)
        self.robot41_angle = 90
        self.robot80_angle = 90
        self.robot41_marker.setRotation(90)
        self.robot80_marker.setRotation(90)
        
        self.tape_hits = []

        # --- boundary trace + cliff map state (BMAP) ---
        # All points stored in scene pixels (scenePos); fitting is scale-free.
        self.boundary_segments = [[]]   # list of segments; each = [(x,y), ...]
        self.corner_angles = []         # measured corner angles (deg) from SEARCH
        self.cliffs = []                # hazard points (x,y)
        self.pending_pt = None          # provisional crossing awaiting confirm
        self.sweep_items = []           # QGraphics items of the latest SWEEPQ (cleared each sweep)

    # ---------------- BMAP DRAW HELPERS ----------------
    def _draw_dot(self, x, y, color, faint=False):
        d = 1.5 * SCALE
        fill = QColor(color)
        fill.setAlpha(80 if faint else 200)
        self.scene.addEllipse(x - d/2, y - d/2, d, d, QPen(QColor(color), 1), QBrush(fill))

    def _draw_cliff(self, x, y):
        s = 2.0 * SCALE
        pen = QPen(QColor(RED_ALERT), 2)
        self.scene.addLine(x - s/2, y - s/2, x + s/2, y + s/2, pen)   # red X = hazard
        self.scene.addLine(x - s/2, y + s/2, x + s/2, y - s/2, pen)
        self.findings_list.addItem(f"[HAZARD] CLIFF @ ({round(x/SCALE,1)}, {round(y/SCALE,1)})cm")

    # ---------------- EXPLORE DRAW HELPERS ----------------
    def _draw_explored_sector(self, cx, cy, heading_deg, radius_cm, span=180.0, steps=24):
        # Filled semicircle (QPolygonF arc) centered on the robot heading. Uses the
        # same (sin, -cos) convention as the SCAN handler so it matches the robot's
        # real forward direction in this scene frame.
        r = radius_cm * SCALE
        pts = [QPointF(cx, cy)]
        start = heading_deg - span / 2.0
        for i in range(steps + 1):
            a = math.radians(start + span * i / steps)
            pts.append(QPointF(cx + r * math.sin(a), cy - r * math.cos(a)))
        fill = QColor("#00FF66"); fill.setAlpha(26)
        self.scene.addPolygon(QPolygonF(pts), QPen(QColor("#00FF66"), 1), QBrush(fill))

    def _draw_nogo_box(self, cx, cy, size_cm=10):
        h = (size_cm / 2.0) * SCALE
        fill = QColor(RED_ALERT); fill.setAlpha(40)
        self.scene.addRect(cx - h, cy - h, 2*h, 2*h,
                           QPen(QColor(RED_ALERT), 1, Qt.PenStyle.DashLine), QBrush(fill))
        self.findings_list.addItem(f"[NO-GO] cliff box @ ({round(cx/SCALE,1)}, {round(cy/SCALE,1)})cm")

    @staticmethod
    def _temp_str(temp):
        # firmware sends -100 (TEMP_INVALID) or -1 (legacy placeholder) for "no reading"
        return "n/a" if temp is None or temp <= -99 or temp == -1 else f"{round(temp,1)}°C"

    def _draw_rock_temp_label(self, x_cm, y_cm, sz_cm, temp):
        label = self.scene.addText(self._temp_str(temp))
        label.setDefaultTextColor(QColor("#FF6B6B"))
        f = QFont(); f.setPointSizeF(7.0); label.setFont(f)
        label.setPos(x_cm * SCALE + (sz_cm * SCALE) / 2 + 2, y_cm * SCALE - (sz_cm * SCALE) / 2)

    def _draw_mountain(self, cx, cy, size_cm):
        r = (size_cm / 2.0) * SCALE
        self.scene.addEllipse(cx - r, cy - r, 2*r, 2*r,
                              QPen(QColor(AMBER), 2, Qt.PenStyle.DashLine),
                              QBrush(QColor(255, 171, 0, 40)))
        self.findings_list.addItem(f"[MOUNTAIN] ~{int(size_cm)}cm @ ({round(cx/SCALE,1)}, {round(cy/SCALE,1)})cm")

    # ---------------- SWEEPQ OBJECT MARKERS ----------------
    def _clear_sweep(self):
        for it in self.sweep_items:
            try:
                self.scene.removeItem(it)
            except Exception:
                pass
        self.sweep_items = []

    def _draw_sweep_object(self, cx, cy, robot_center, dist_cm, angle_deg):
        OBJ_GREEN = "#39FF14"
        # faint line from the robot to the object (direction)
        line = self.scene.addLine(robot_center.x(), robot_center.y(), cx, cy,
                                   QPen(QColor(57, 255, 20, 90), 1))
        self.sweep_items.append(line)
        # object marker
        d = 2.2 * SCALE
        dot = self.scene.addEllipse(cx - d/2, cy - d/2, d, d,
                                    QPen(QColor("#FFFFFF"), 1), QBrush(QColor(OBJ_GREEN)))
        dot.setToolTip(f"Object\nDist: {round(dist_cm,1)} cm\nBearing: {round(angle_deg,1)}°")
        self.sweep_items.append(dot)
        # distance label
        label = self.scene.addText(f"{round(dist_cm,1)} cm")
        label.setDefaultTextColor(QColor(OBJ_GREEN))
        f = QFont(); f.setPointSizeF(7.0); label.setFont(f)
        label.setPos(cx + d/2, cy - d)
        self.sweep_items.append(label)
        self.findings_list.addItem(
            f"[SWEEP] obj @ {round(angle_deg,1)}°, {round(dist_cm,1)}cm")

    def _fit_segment_tls(self, pts):
        # Total least squares via SVD: principal axis of the centroid-subtracted
        # points. Minimizes PERPENDICULAR distance, so it's correct for any
        # orientation including near-vertical (where ordinary least squares fails).
        P = np.asarray(pts, dtype=float)
        c = P.mean(axis=0)
        _, _, vt = np.linalg.svd(P - c)
        d = vt[0]
        return c, d / np.linalg.norm(d)

    def _intersect(self, c0, d0, c1, d1):
        # solve c0 + t*d0 == c1 + u*d1  for the intersection point
        A = np.array([[d0[0], -d1[0]], [d0[1], -d1[1]]], dtype=float)
        if abs(np.linalg.det(A)) < 1e-9:
            return None   # parallel
        t = np.linalg.solve(A, np.array([c1[0]-c0[0], c1[1]-c0[1]], dtype=float))[0]
        return (c0[0] + t*d0[0], c0[1] + t*d0[1])

    def fit_and_draw_boundary(self):
        # Offline geometric fit, replacing the hardcoded-150cm draw_arena_boundary.
        try:
            segs = [s for s in self.boundary_segments if len(s) >= 2]
            if len(segs) < 2:
                self.log_widget.append(
                    f"<span style='color:{RED_ALERT};'>[MAP] Not enough segments to fit ({len(segs)}).</span>")
                return

            lines = [self._fit_segment_tls(s) for s in segs]   # [(centroid, unit_dir)]
            n = len(lines)

            # Intersect consecutive fitted lines -> corner vertices.
            # Wrap last->first to close the loop (closed boundary assumption).
            verts = []
            for i in range(n):
                c0, d0 = lines[i]
                c1, d1 = lines[(i + 1) % n]
                v = self._intersect(c0, d0, c1, d1)
                if v is not None:
                    verts.append(v)

            if len(verts) < 2:
                self.log_widget.append(
                    f"<span style='color:{RED_ALERT};'>[MAP] Boundary fit failed (parallel segments).</span>")
                return

            poly = QPolygonF([QPointF(float(x), float(y)) for x, y in verts])
            self.scene.addPolygon(
                poly, QPen(QColor(RED_ALERT), 3, Qt.PenStyle.DashLine),
                QBrush(QColor(255, 42, 42, 10)))

            # Cross-check: measured SEARCH corner angle vs angle between fitted dirs.
            for i in range(min(len(self.corner_angles), n)):
                _, d0 = lines[i]
                _, d1 = lines[(i + 1) % n]
                fit_ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(d0, d1))))))
                meas = self.corner_angles[i]
                if min(abs(meas - fit_ang), abs(meas - (180.0 - fit_ang))) > 25.0:
                    self.log_widget.append(
                        f"<span style='color:{AMBER};'>[MAP] corner {i}: measured {meas:.0f}° "
                        f"vs fitted {fit_ang:.0f}° — possible odometry drift.</span>")

            self.log_widget.append(
                f"<span style='color:{RED_ALERT};'>[SYS] BOUNDARY_MAPPED: "
                f"{len(segs)} segments, {len(self.cliffs)} cliffs.</span>")
        except Exception as e:
            self.log_widget.append(
                f"<span style='color:#FF0000;'>[ERR] Boundary fit failed: {e}</span>")

    def draw_arena_boundary(self):
        try:
            p1, p2, p3 = self.tape_hits[0], self.tape_hits[1], self.tape_hits[2]
            KNOWN_SIZE = 150 * SCALE

            dx, dy = p3[0] - p1[0], p3[1] - p1[1]
            dist = math.hypot(dx, dy)
            if dist == 0: return

            cos_val = min(1.0, KNOWN_SIZE / dist) 
            theta = math.atan2(dy, dx) - math.acos(cos_val)

            u_x, u_y = math.cos(theta), math.sin(theta)
            v_x, v_y = -math.sin(theta), math.cos(theta)

            dp = (p1[0] - p2[0]) * v_x + (p1[1] - p2[1]) * v_y
            c1_x, c1_y = p2[0] + dp * v_x, p2[1] + dp * v_y

            mid_x, mid_y = (p1[0] + p3[0]) / 2, (p1[1] + p3[1]) / 2
            best_dist, best_poly = float('inf'), None

            for sign_u in [1, -1]:
                for sign_v in [1, -1]:
                    cx = c1_x + (sign_u * KNOWN_SIZE * u_x / 2) + (sign_v * KNOWN_SIZE * v_x / 2)
                    cy = c1_y + (sign_u * KNOWN_SIZE * u_y / 2) + (sign_v * KNOWN_SIZE * v_y / 2)
                    if math.hypot(cx - mid_x, cy - mid_y) < best_dist:
                        best_dist = math.hypot(cx - mid_x, cy - mid_y)
                        best_poly = QPolygonF([
                            QPointF(c1_x, c1_y),
                            QPointF(c1_x + sign_u * KNOWN_SIZE * u_x, c1_y + sign_u * KNOWN_SIZE * u_y),
                            QPointF(c1_x + sign_u * KNOWN_SIZE * u_x + sign_v * KNOWN_SIZE * v_x, c1_y + sign_u * KNOWN_SIZE * u_y + sign_v * KNOWN_SIZE * v_y),
                            QPointF(c1_x + sign_v * KNOWN_SIZE * v_x, c1_y + sign_v * KNOWN_SIZE * v_y)
                        ])

            # Draw glowing red boundary
            self.scene.addPolygon(best_poly, QPen(QColor(RED_ALERT), 3, Qt.PenStyle.DashLine), QBrush(QColor(255, 42, 42, 10)))
            self.log_widget.append(f"<span style='color: {RED_ALERT};'>[SYS] BOUNDARY_LOCKED: 150x150cm secure zone established.</span>")
        except Exception as e:
            self.log_widget.append(f"<span style='color: #FF0000;'>[ERR] Boundary calc failed: {e}</span>")

    def rotate_robot(self, robot_name, delta):
        if robot_name == "Robot41":
            self.robot41_angle += delta
            self.robot41_marker.setRotation(self.robot41_angle)
        else:
            self.robot80_angle += delta
            self.robot80_marker.setRotation(self.robot80_angle)

    # ---------------- MESSAGE HANDLER ----------------
    def handle_message(self, robot_name, text):
        time_str = datetime.now().strftime("%H:%M:%S")
        is_41 = robot_name == "Robot41"
        
        # Color code the incoming logs
        bot_color = CYAN if is_41 else MAGENTA
        self.log_widget.append(f"<span style='color: #4A5568;'>[{time_str}]</span> <span style='color: {bot_color}; font-weight: bold;'>[{robot_name}]</span> <span style='color: #E2E8F0;'>{text}</span>")

        # Routing (Silent in the UI to prevent clutter, just do the math)
        if robot_name == "Robot41": self.workerB.publish_message("/pynqbridge/80/recv", text)
        elif robot_name == "Robot80": self.workerA.publish_message("/pynqbridge/41/recv", text)

        try:
            target = self.robot41_marker if is_41 else self.robot80_marker
            parts = text.strip().split(",")
            cmd = parts[0]

            # --- MOUNTAIN SCANNING ---
            if cmd == "SCAN":
                dist_mm = float(parts[1])
                if dist_mm > 0 and dist_mm < 800:
                    dist_cm = dist_mm / 10.0
                    total_dist_cm = dist_cm + SENSOR_OFFSET_CM

                    old_c = target.sceneBoundingRect().center()
                    angle_deg = self.robot41_angle if is_41 else self.robot80_angle
                    angle_rad = math.radians(angle_deg)

                    mountain_x = old_c.x() + (total_dist_cm * SCALE * math.sin(angle_rad))
                    mountain_y = old_c.y() - (total_dist_cm * SCALE * math.cos(angle_rad))

                    dot_size = 2 * SCALE 
                    dot = self.scene.addEllipse(
                        mountain_x - dot_size / 2, mountain_y - dot_size / 2,
                        dot_size, dot_size,
                        QPen(Qt.GlobalColor.transparent), QBrush(QColor(AMBER)) 
                    )
                    dot.setToolTip(f"Point Cloud\nDist: {dist_cm}cm\nAngle: {round(angle_deg, 1)}°")

            # --- BOUNDARY TAPE ---
            elif cmd == "STOPBLACK":
                current_pos = target.scenePos()
                tx, ty = current_pos.x(), current_pos.y()
                dot_size = 2 * SCALE 
                self.scene.addEllipse(tx - dot_size/2, ty - dot_size/2, dot_size, dot_size, 
                                      QPen(QColor("#FFFFFF"), 1), QBrush(QColor(255, 255, 255, 200)))
                
                real_x, real_y = tx / SCALE, ty / SCALE
                self.tape_hits.append((real_x * SCALE, real_y * SCALE))
                if len(self.tape_hits) == 3:
                    self.draw_arena_boundary()

            # --- BMAP: provisional crossing (awaiting confirm) ---
            elif cmd == "PROV":
                cur = target.scenePos()
                self.pending_pt = (cur.x(), cur.y())
                self._draw_dot(cur.x(), cur.y(), AMBER, faint=True)

            # --- BMAP: confirmed boundary crossing(s) ---
            elif cmd == "BPT":
                cur = target.scenePos()
                seg = self.boundary_segments[-1]
                if self.pending_pt is not None:                  # the original crossing
                    seg.append(self.pending_pt)
                    self._draw_dot(self.pending_pt[0], self.pending_pt[1], "#FFFFFF")
                    self.pending_pt = None
                seg.append((cur.x(), cur.y()))                   # the re-crossing
                self._draw_dot(cur.x(), cur.y(), "#FFFFFF")

            # --- BMAP: corner -> start a new segment ---
            elif cmd == "CORNER":
                angle = float(parts[1]) if len(parts) > 1 else 0.0
                self.corner_angles.append(angle)
                if self.boundary_segments[-1]:                   # only split if non-empty
                    self.boundary_segments.append([])
                self.log_widget.append(
                    f"<span style='color:{AMBER};'>[MAP] CORNER ~{angle:.1f}°</span>")

            # --- BMAP: isolated cliff (hazard) ---
            elif cmd == "CLIFF":
                if self.pending_pt is not None:
                    pt = self.pending_pt
                    self.pending_pt = None
                else:
                    cur = target.scenePos()
                    pt = (cur.x(), cur.y())
                self.cliffs.append(pt)
                self._draw_cliff(pt[0], pt[1])

            # --- BMAP: line lost ---
            elif cmd == "LINE_LOST":
                self.log_widget.append(
                    f"<span style='color:{RED_ALERT};'>[MAP] LINE_LOST — trace ended without closure.</span>")

            # --- BMAP: trace done -> offline fit ---
            elif cmd == "BMAP_DONE":
                self.fit_and_draw_boundary()

            # --- TRUE ODOMETRY ---
            elif cmd == "ODOM":
                steps_l, steps_r = float(parts[1]), float(parts[2])

                dist_l, dist_r = steps_l * CM_PER_STEP, steps_r * CM_PER_STEP
                d_center = (dist_l + dist_r) / 2.0

                d_theta_rad = (dist_l - dist_r) / WHEEL_BASE_CM
                d_theta_deg = math.degrees(d_theta_rad)

                old_pos = target.scenePos()
                angle_deg = self.robot41_angle if is_41 else self.robot80_angle
                new_angle_deg = (angle_deg + d_theta_deg) % 360
                
                if is_41: self.robot41_angle = new_angle_deg
                else: self.robot80_angle = new_angle_deg

                avg_angle_rad = math.radians(angle_deg + (d_theta_deg / 2))
                dx = d_center * SCALE * math.sin(avg_angle_rad)
                dy = -d_center * SCALE * math.cos(avg_angle_rad)

                nx, ny = old_pos.x() + dx, old_pos.y() + dy

                trail_color = CYAN if is_41 else MAGENTA
                self.scene.addLine(old_pos.x(), old_pos.y(), nx, ny, QPen(QColor(trail_color), 2, Qt.PenStyle.SolidLine))
                
                target.setRotation(new_angle_deg)
                target.setPos(nx, ny)

                label = self.pos_41_label if is_41 else self.pos_80_label
                label.setText(f"X: {round(nx/SCALE,1)} | Y: {round(ny/SCALE,1)}")

            # --- FINE ROTATION ---
            elif cmd == "ROT":
                self.rotate_robot(robot_name, float(parts[1]))

            # =========================================
            # ORIENTATION (Absolute Compass/IMU Snap)
            # =========================================

            elif cmd == "ORT":
                ort_zone = int(parts[1])
                
                # Check if the message has the new theta angle, otherwise default to 0
                theta = float(parts[2]) if len(parts) > 2 else 0.0

                # Determine the base angle of the quadrant
                base_angle = {
                    1: 90,     # right / East
                    2: 180,    # down / South
                    3: 270,    # left / West
                    4: 0       # up / North
                }.get(ort_zone, 0)

                # Calculate absolute angle. 
                # NOTE: If your robot turns right and the UI turns left, 
                # just change (base_angle + theta) to (base_angle - theta)
                absolute_angle = (base_angle + theta) % 360

                if is_41:
                    self.robot41_angle = absolute_angle
                else:
                    self.robot80_angle = absolute_angle

                # Physically snap the UI robot to the new corrected heading
                target.setRotation(absolute_angle)
                #self.update_direction_marker(robot_name)

            # =========================================
            # STEPS (Legacy Straight-Line Driving)
            # =========================================
            elif cmd == "STEPS":
                steps = float(parts[1])

                # Get the true center position
                old_pos = target.scenePos()

                if is_41:
                    angle_deg = self.robot41_angle
                else:
                    angle_deg = self.robot80_angle

                angle = math.radians(angle_deg)

                # Calculate movement vector
                dx = steps * SCALE * math.sin(angle)
                dy = -steps * SCALE * math.cos(angle)

                nx = old_pos.x() + dx
                ny = old_pos.y() + dy

                # Set position directly to the center (no clunky offsets!)
                target.setPos(nx, ny)

                trail_color = CYAN if is_41 else MAGENTA
                self.scene.addLine(old_pos.x(), old_pos.y(), nx, ny, QPen(QColor(trail_color), 2, Qt.PenStyle.SolidLine))

                label = self.pos_41_label if is_41 else self.pos_80_label
                label.setText(f"X: {round(nx/SCALE,1)} | Y: {round(ny/SCALE,1)}")

            # --- ROCK/SAMPLE ---
            elif cmd == "ROCK":
                x, y, sz, col, temp = float(parts[1]), float(parts[2]), int(parts[3]), parts[4], float(parts[5])
                self.scene.addItem(RockSample(x, y, sz, col, temp))
                self._draw_rock_temp_label(x, y, sz, temp)
                self.findings_list.addItem(f"[{robot_name}] {col.upper()} ANOMALY | {sz}cm | {self._temp_str(temp)}")

            # --- AUTO-LOCATING ROCK ---
            elif cmd == "FOUND_ROCK":
                # Expected format: FOUND_ROCK,size,color,temperature (e.g., FOUND_ROCK,5,red,45.0)
                sz = int(parts[1])
                col = parts[2]
                temp = float(parts[3])

                # Grab the robot's exact current coordinate from the physics engine!
                current_pos = target.scenePos()
                
                # Convert UI pixels back to real-world centimeters
                real_x = current_pos.x() / SCALE
                real_y = current_pos.y() / SCALE

                # Drop the rock exactly under the robot
                self.scene.addItem(RockSample(real_x, real_y, sz, col, temp))
                self._draw_rock_temp_label(real_x, real_y, sz, temp)
                self.findings_list.addItem(f"[{robot_name}] {col.upper()} ANOMALY | {sz}cm | {self._temp_str(temp)}")

            # --- EXPLORE: explored semicircle region ---
            elif cmd == "REGION":
                radius_cm = float(parts[1]) if len(parts) > 1 else 40.0
                c = target.scenePos()
                angle_deg = self.robot41_angle if is_41 else self.robot80_angle
                self._draw_explored_sector(c.x(), c.y(), angle_deg, radius_cm)

            # --- EXPLORE: mountain obstacle (placed ahead of the robot) ---
            elif cmd == "MOUNTAIN":
                size_cm = float(parts[1]) if len(parts) > 1 else 30.0
                c = target.scenePos()
                angle_deg = self.robot41_angle if is_41 else self.robot80_angle
                a = math.radians(angle_deg)
                fwd = 15.0 * SCALE
                self._draw_mountain(c.x() + fwd * math.sin(a),
                                    c.y() - fwd * math.cos(a), size_cm)

            # --- EXPLORE: small-cliff no-go box at the robot ---
            elif cmd == "NOGO":
                c = target.scenePos()
                self._draw_nogo_box(c.x(), c.y(), 10)

            # --- EXPLORE: run finished ---
            elif cmd == "EXPLORE_DONE":
                self.log_widget.append(
                    f"<span style='color:{CYAN};'>[EXPLORE] mapping run complete.</span>")

            # --- SWEEPQ: start of a new object list -> clear the previous sweep markers ---
            elif cmd == "OBJN":
                self._clear_sweep()
                n = int(parts[1]) if len(parts) > 1 else 0
                self.log_widget.append(
                    f"<span style='color:#39FF14;'>[SWEEP] {n} object(s)</span>")

            # --- SWEEPQ: one detected object at (rel bearing, distance) ---
            elif cmd == "OBJ":
                rel = float(parts[1])
                dist_mm = float(parts[2])
                c = target.sceneBoundingRect().center()
                angle_deg = (self.robot41_angle if is_41 else self.robot80_angle) + rel
                angle_rad = math.radians(angle_deg)
                total_cm = dist_mm / 10.0 + SENSOR_OFFSET_CM
                ox = c.x() + total_cm * SCALE * math.sin(angle_rad)
                oy = c.y() - total_cm * SCALE * math.cos(angle_rad)
                self._draw_sweep_object(ox, oy, c, total_cm, angle_deg)

            # --- standalone temperature reading ---
            elif cmd == "TEMP":
                temp = float(parts[1]) if len(parts) > 1 else -100.0
                self.log_widget.append(
                    f"<span style='color:#FF6B6B;'>[TEMP] {self._temp_str(temp)}</span>")

            # --- WIRELESS LOG line (already shown raw above; just don't parse as geometry) ---
            elif cmd == "LOG":
                pass

            # --- STATUS UPDATE ---
            if is_41:
                self.status_41_label.setText(f"<span style='color: {CYAN};'>● ONLINE</span>")
                self.timer41.start(3000)
            else:
                self.status_80_label.setText(f"<span style='color: {MAGENTA};'>● ONLINE</span>")
                self.timer80.start(3000)

        except Exception as e:
            self.log_widget.append(f"<span style='color: {RED_ALERT};'>[SYS_ERR] {e}</span>")

    def set_offline(self, robot_name):
        if robot_name == "Robot41":
            self.status_41_label.setText("<span style='color: #4A5568;'>○ OFFLINE</span>")
        else:
            self.status_80_label.setText("<span style='color: #4A5568;'>○ OFFLINE</span>")

    def zoom_to_fit(self):
        rect = self.scene.itemsBoundingRect()
        if not rect.isNull():
            rect.adjust(-50, -50, 50, 50)
            self.view.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)

    def clear_all(self):
        self.scene.clear()
        self.draw_guides()
        self.initialize_robots()
        self.log_widget.clear()
        self.findings_list.clear()

# ---------------- MAIN ----------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VenusDashboard()
    window.show()
    sys.exit(app.exec())
