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
# Firmware STEPS messages carry a MOVE_UNIT *chunk count* (advance_monitored /
# move_batch_until count their hops), NOT centimetres. One chunk = MOVE_UNIT
# steps, so the physical distance per chunk is MOVE_UNIT * CM_PER_STEP. The STEPS
# handler MUST scale by this (earlier it treated the chunk count as cm directly,
# so the marker advanced ~6x too little for the real distance driven).
MOVE_UNIT       = 500                                   # firmware MOVE_UNIT (main_header.h)
CM_PER_UNIT     = MOVE_UNIT * CM_PER_STEP               # ~6.15 cm physical per STEPS chunk
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

    # Robot config: name → (color_str, fill_hex)
    ROBOT_CONFIG = {
        "Robot41": (CYAN,    "#002222"),
        "Robot80": (MAGENTA, "#220022"),
    }

    def _make_robot_marker(self, color_str, fill_hex):
        """Create and return a triangular polygon marker for one robot."""
        poly = QPolygonF([
            QPointF(0, -ROBOT_SIZE/2),
            QPointF(ROBOT_SIZE/2.5, ROBOT_SIZE/2),
            QPointF(0, ROBOT_SIZE/4),
            QPointF(-ROBOT_SIZE/2.5, ROBOT_SIZE/2),
        ])
        marker = self.scene.addPolygon(poly, QPen(QColor(color_str), 2), QBrush(QColor(fill_hex)))
        marker.setTransformOriginPoint(0, 0)
        return marker

    def _bot_state(self, robot_name):
        """Return (creating if absent) the per-robot state dict for robot_name.

        Key schema — FROZEN for F4/F5 compatibility (IP-4):
          marker            : QGraphicsPolygonItem on the scene
          angle             : float, current heading in scene degrees (0=up/N, 90=right/E, …)
          pending_pt        : (x,y) scene-px or None — provisional boundary crossing
          sweep_items       : list of QGraphicsItem — cleared each SWEEPQ
          boundary_segments : list of segments; each segment = [(x,y), …]
          corner_angles     : list of float — measured corners (deg) from SEARCH
          tape_hits         : list of (x,y) — legacy tape-boundary hits (STOPBLACK)
          black_contacts    : list of dict — F5 BLACKPT contacts, per-robot odometry frame
                             (F5 appends {"pos": (x_cm, y_cm), "heading": deg, "run": int})
        """
        if robot_name not in self.bots:
            color_str, fill_hex = self.ROBOT_CONFIG.get(
                robot_name, (CYAN, "#002222"))
            marker = self._make_robot_marker(color_str, fill_hex)
            marker.setPos(0, 0)
            marker.setRotation(90)
            self.bots[robot_name] = {
                "marker":            marker,
                "angle":             90,
                "pending_pt":        None,
                "sweep_items":       [],
                "boundary_segments": [[]],
                "corner_angles":     [],
                "tape_hits":         [],
                "black_contacts":    [],
            }
        return self.bots[robot_name]

    def initialize_robots(self):
        """(Re-)create per-robot state and the shared map layers."""
        # Per-robot state — one dict per known robot name.
        self.bots = {}

        # Eagerly create the two competition robots so they appear at startup.
        self._bot_state("Robot41")
        self._bot_state("Robot80")

        # Shared combined hazard layer (NOT per-robot — both robots contribute).
        self.cliffs = []

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
    def _clear_sweep(self, st):
        """Remove sweep markers for the given robot only."""
        for it in st["sweep_items"]:
            try:
                self.scene.removeItem(it)
            except Exception:
                pass
        st["sweep_items"] = []

    def _draw_sweep_object(self, cx, cy, robot_center, dist_cm, angle_deg, st):
        OBJ_GREEN = "#39FF14"
        # faint line from the robot to the object (direction)
        line = self.scene.addLine(robot_center.x(), robot_center.y(), cx, cy,
                                   QPen(QColor(57, 255, 20, 90), 1))
        st["sweep_items"].append(line)
        # object marker
        d = 2.2 * SCALE
        dot = self.scene.addEllipse(cx - d/2, cy - d/2, d, d,
                                    QPen(QColor("#FFFFFF"), 1), QBrush(QColor(OBJ_GREEN)))
        dot.setToolTip(f"Object\nDist: {round(dist_cm,1)} cm\nBearing: {round(angle_deg,1)}°")
        st["sweep_items"].append(dot)
        # distance label
        label = self.scene.addText(f"{round(dist_cm,1)} cm")
        label.setDefaultTextColor(QColor(OBJ_GREEN))
        f = QFont(); f.setPointSizeF(7.0); label.setFont(f)
        label.setPos(cx + d/2, cy - d)
        st["sweep_items"].append(label)
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

    def fit_and_draw_boundary(self, st):
        """Offline geometric fit for one robot's accumulated boundary segments."""
        try:
            segs = [s for s in st["boundary_segments"] if len(s) >= 2]
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
            for i in range(min(len(st["corner_angles"]), n)):
                _, d0 = lines[i]
                _, d1 = lines[(i + 1) % n]
                fit_ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(d0, d1))))))
                meas = st["corner_angles"][i]
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

    # ============================================================
    # F5 — black-contact classification (boundary loop vs interior cliff)
    # All methods take st = self.bots[robot_name]; they CLUSTER per-robot
    # (odometry drift differs per robot, so contacts must be grouped within
    # one robot) and draw onto the SHARED self.cliffs hazard layer. Boundary
    # segments feed the per-robot fit_and_draw_boundary(st) helper.
    # ============================================================
    BLACK_CLUSTER_GAP_CM   = 25.0   # contacts farther apart than this start a new cluster
    BLACK_BOUNDARY_MIN_PTS = 4      # a cluster with >= this many points is a boundary run, not a patch
    BLACK_BOUNDARY_MIN_RUN = 3      # ... OR any contact whose reported run reached this length

    def _cluster_contacts(self, st):
        # Greedy spatial clustering of this robot's black contacts (scene px).
        # Returns list of clusters; each cluster = list of contact dicts.
        # Contacts arrive in time order along the robot's path, so a simple
        # "gap from the previous accepted point" split tracks a continuous trace
        # and isolates lone patches. Per-robot only (st-scoped).
        contacts = st.get("black_contacts", [])
        if not contacts:
            return []
        gap_px = self.BLACK_CLUSTER_GAP_CM * SCALE
        clusters = [[contacts[0]]]
        prev = contacts[0]
        for pt in contacts[1:]:
            d = math.hypot(pt["x"] - prev["x"], pt["y"] - prev["y"])
            # A run reset (prev run > 1 dropping back to 1) means the firmware started a
            # fresh EXPLORE pass — always force a new cluster regardless of current size.
            run_reset = (prev.get("run", 1) > 1 and pt.get("run", 1) == 1)
            if d > gap_px or run_reset:
                clusters.append([pt])
            else:
                clusters[-1].append(pt)
            prev = pt
        return clusters

    def _segment_boundary_loop(self, st, cluster):
        # A continuous-run cluster = the robot followed the boundary tape. Append its
        # points (as (x,y) tuples) to the per-robot boundary_segments as one new
        # segment, then re-fit + redraw the boundary polygon via the existing TLS
        # fitter so the boundary layer stays in sync.
        seg = [(c["x"], c["y"]) for c in cluster]
        if len(seg) < 2:
            return False
        if st["boundary_segments"] and not st["boundary_segments"][-1]:
            st["boundary_segments"][-1] = seg          # fill the empty trailing segment
        else:
            st["boundary_segments"].append(seg)
        st["boundary_segments"].append([])             # open a fresh trailing segment
        return True

    def classify_black_contacts(self, st):
        # Run at EXPLORE_DONE for one robot. Cluster its contacts, then per cluster:
        #   continuous run  -> boundary segment (polygon, per-robot fit)
        #   lone patch      -> interior cliff (red X, shared self.cliffs)
        try:
            clusters = self._cluster_contacts(st)
            if not clusters:
                return
            n_bound, n_cliff = 0, 0
            for cl in clusters:
                max_run = max((c.get("run", 1) for c in cl), default=1)
                is_boundary = (len(cl) >= self.BLACK_BOUNDARY_MIN_PTS
                               or max_run >= self.BLACK_BOUNDARY_MIN_RUN)
                if is_boundary and self._segment_boundary_loop(st, cl):
                    n_bound += 1
                else:
                    # lone patch -> interior cliff at the cluster centroid (shared layer)
                    cx = sum(c["x"] for c in cl) / len(cl)
                    cy = sum(c["y"] for c in cl) / len(cl)
                    self.cliffs.append((cx, cy))
                    self._draw_cliff(cx, cy)
                    n_cliff += 1
            if n_bound:
                self.fit_and_draw_boundary(st)          # per-robot polygon re-fit
            self.log_widget.append(
                f"<span style='color:{AMBER};'>[MAP] black: "
                f"{len(clusters)} cluster(s) -> {n_bound} boundary, {n_cliff} cliff.</span>")
        except Exception as e:
            self.log_widget.append(
                f"<span style='color:{RED_ALERT};'>[ERR] classify_black_contacts: {e}</span>")

    def draw_arena_boundary(self, st):
        """Draw the legacy 3-hit boundary for the given robot's tape_hits."""
        try:
            p1, p2, p3 = st["tape_hits"][0], st["tape_hits"][1], st["tape_hits"][2]
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
        st = self._bot_state(robot_name)
        st["angle"] = (st["angle"] + delta) % 360
        st["marker"].setRotation(st["angle"])

    # ---------------- MESSAGE HANDLER ----------------
    def handle_message(self, robot_name, text):
        time_str = datetime.now().strftime("%H:%M:%S")
        is_41 = robot_name == "Robot41"
        
        # Color code the incoming logs
        bot_color = CYAN if is_41 else MAGENTA
        self.log_widget.append(f"<span style='color: #4A5568;'>[{time_str}]</span> <span style='color: {bot_color}; font-weight: bold;'>[{robot_name}]</span> <span style='color: #E2E8F0;'>{text}</span>")

        try:
            st = self._bot_state(robot_name)   # per-robot state dict
            target = st["marker"]
            parts = text.strip().split(",")
            cmd = parts[0]

            # --- MOUNTAIN SCANNING ---
            if cmd == "SCAN":
                dist_mm = float(parts[1])
                if dist_mm > 0 and dist_mm < 800:
                    dist_cm = dist_mm / 10.0
                    total_dist_cm = dist_cm + SENSOR_OFFSET_CM

                    old_c = target.sceneBoundingRect().center()
                    angle_deg = st["angle"]
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
                st["tape_hits"].append((real_x * SCALE, real_y * SCALE))
                if len(st["tape_hits"]) == 3:
                    self.draw_arena_boundary(st)

            # --- BMAP: provisional crossing (awaiting confirm) ---
            elif cmd == "PROV":
                cur = target.scenePos()
                st["pending_pt"] = (cur.x(), cur.y())
                self._draw_dot(cur.x(), cur.y(), AMBER, faint=True)

            # --- BMAP: confirmed boundary crossing(s) ---
            elif cmd == "BPT":
                cur = target.scenePos()
                seg = st["boundary_segments"][-1]
                if st["pending_pt"] is not None:                  # the original crossing
                    seg.append(st["pending_pt"])
                    self._draw_dot(st["pending_pt"][0], st["pending_pt"][1], "#FFFFFF")
                    st["pending_pt"] = None
                seg.append((cur.x(), cur.y()))                    # the re-crossing
                self._draw_dot(cur.x(), cur.y(), "#FFFFFF")

            # --- BMAP: corner -> start a new segment ---
            elif cmd == "CORNER":
                angle = float(parts[1]) if len(parts) > 1 else 0.0
                st["corner_angles"].append(angle)
                if st["boundary_segments"][-1]:                  # only split if non-empty
                    st["boundary_segments"].append([])
                self.log_widget.append(
                    f"<span style='color:{AMBER};'>[MAP] CORNER ~{angle:.1f}°</span>")

            # --- BMAP: isolated cliff (hazard) ---
            elif cmd == "CLIFF":
                if st["pending_pt"] is not None:
                    pt = st["pending_pt"]
                    st["pending_pt"] = None
                else:
                    cur = target.scenePos()
                    pt = (cur.x(), cur.y())
                self.cliffs.append(pt)      # shared combined cliff layer
                self._draw_cliff(pt[0], pt[1])

            # --- BMAP: line lost ---
            elif cmd == "LINE_LOST":
                self.log_widget.append(
                    f"<span style='color:{RED_ALERT};'>[MAP] LINE_LOST — trace ended without closure.</span>")

            # --- BMAP: trace done -> offline fit ---
            elif cmd == "BMAP_DONE":
                self.fit_and_draw_boundary(st)

            # --- TRUE ODOMETRY ---
            elif cmd == "ODOM":
                steps_l, steps_r = float(parts[1]), float(parts[2])

                dist_l, dist_r = steps_l * CM_PER_STEP, steps_r * CM_PER_STEP
                d_center = (dist_l + dist_r) / 2.0

                d_theta_rad = (dist_l - dist_r) / WHEEL_BASE_CM
                d_theta_deg = math.degrees(d_theta_rad)

                old_pos = target.scenePos()
                angle_deg = st["angle"]
                new_angle_deg = (angle_deg + d_theta_deg) % 360
                st["angle"] = new_angle_deg

                avg_angle_rad = math.radians(angle_deg + (d_theta_deg / 2))
                dx = d_center * SCALE * math.sin(avg_angle_rad)
                dy = -d_center * SCALE * math.cos(avg_angle_rad)

                nx, ny = old_pos.x() + dx, old_pos.y() + dy

                trail_color = CYAN if is_41 else MAGENTA
                self.scene.addLine(old_pos.x(), old_pos.y(), nx, ny, QPen(QColor(trail_color), 2, Qt.PenStyle.SolidLine))
                
                target.setRotation(new_angle_deg)
                target.setPos(nx, ny)

                label = {"Robot41": self.pos_41_label, "Robot80": self.pos_80_label}.get(robot_name)
                if label is not None:
                    label.setText(f"X: {round(nx/SCALE,1)} | Y: {round(ny/SCALE,1)}")

            # --- FINE ROTATION ---
            elif cmd == "ROT":
                self.rotate_robot(robot_name, float(parts[1]))

            # =========================================
            # ORIENTATION (Absolute Compass/IMU Snap)
            # =========================================

            elif cmd == "ORT":
                ort_zone = int(parts[1])

                # theta = fine angle within the quadrant (0–90°, clockwise from the
                # quadrant's base direction). Default 0 for older firmware.
                theta = float(parts[2]) if len(parts) > 2 else 0.0

                # BUG-1 FIX: firmware defines ort 1=NORTH, 2=EAST, 3=SOUTH, 4=WEST.
                # Map directly to scene degrees (0=up=North, 90=right=East, …).
                # Old (wrong) map was {1:90, 2:180, 3:270, 4:0} (shifted by one quadrant).
                base_angle = {
                    1: 0,      # NORTH — up in scene
                    2: 90,     # EAST  — right in scene
                    3: 180,    # SOUTH — down in scene
                    4: 270,    # WEST  — left in scene
                }.get(ort_zone, 0)

                absolute_angle = (base_angle + theta) % 360
                st["angle"] = absolute_angle

                # Snap the UI robot to the corrected heading
                target.setRotation(absolute_angle)

            # =========================================
            # STEPS (Legacy Straight-Line Driving)
            # =========================================
            elif cmd == "STEPS":
                units = float(parts[1])           # MOVE_UNIT chunk count (signed), NOT cm
                dist_cm = units * CM_PER_UNIT      # chunks -> physical centimetres

                old_pos = target.scenePos()
                angle_deg = st["angle"]

                angle = math.radians(angle_deg)

                # Calculate movement vector (cm -> scene pixels via SCALE)
                dx = dist_cm * SCALE * math.sin(angle)
                dy = -dist_cm * SCALE * math.cos(angle)

                nx = old_pos.x() + dx
                ny = old_pos.y() + dy

                # Set position directly to the center (no clunky offsets!)
                target.setPos(nx, ny)

                trail_color = CYAN if is_41 else MAGENTA
                self.scene.addLine(old_pos.x(), old_pos.y(), nx, ny, QPen(QColor(trail_color), 2, Qt.PenStyle.SolidLine))

                label = {"Robot41": self.pos_41_label, "Robot80": self.pos_80_label}.get(robot_name)
                if label is not None:
                    label.setText(f"X: {round(nx/SCALE,1)} | Y: {round(ny/SCALE,1)}")

            # --- POSFIX: translation fix from re-sweep bearing/distance geometry (F4) ---
            elif cmd == "POSFIX":
                try:
                    dx_mm = float(parts[1])   # rightward in robot-local frame (mm)
                    dy_mm = float(parts[2])   # forward  in robot-local frame (mm)

                    # F8's per-robot dict (applied before F4/F5) — IP-4
                    st = self.bots[robot_name]
                    angle_rad = math.radians(st["angle"])

                    # Rotate robot-local (dx_right, dy_fwd) into scene coords.
                    # Scene: x = right, y = DOWN; robot angle=0 means facing UP (North).
                    # BUG-1 fixed: {1:0, 2:90, 3:180, 4:270} so angle=0 = North = up.
                    # Forward (dy) in scene = (sin θ, −cos θ) (same as STEPS handler).
                    # Right   (dx) in scene = (cos θ, +sin θ) (90° CW from forward).
                    scene_dx = (dy_mm / 10.0 * SCALE) * math.sin(angle_rad) \
                             + (dx_mm / 10.0 * SCALE) * math.cos(angle_rad)
                    scene_dy = -(dy_mm / 10.0 * SCALE) * math.cos(angle_rad) \
                             +  (dx_mm / 10.0 * SCALE) * math.sin(angle_rad)

                    old_pos = st["marker"].scenePos()
                    new_x   = old_pos.x() + scene_dx
                    new_y   = old_pos.y() + scene_dy
                    st["marker"].setPos(new_x, new_y)

                    self.log_widget.append(
                        f"<span style='color:{AMBER};'>"
                        f"[POSFIX] {robot_name}: dx={int(dx_mm):+d} mm right, "
                        f"dy={int(dy_mm):+d} mm fwd → "
                        f"scene ({old_pos.x():.0f},{old_pos.y():.0f}) → "
                        f"({new_x:.0f},{new_y:.0f})"
                        f"</span>"
                    )
                except Exception as e:
                    self.log_widget.append(
                        f"<span style='color:{RED_ALERT};'>[ERR] POSFIX parse: {e}</span>")

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
                self._draw_explored_sector(c.x(), c.y(), st["angle"], radius_cm)

            # --- EXPLORE: mountain obstacle (placed ahead of the robot) ---
            elif cmd == "MOUNTAIN":
                size_cm = float(parts[1]) if len(parts) > 1 else 30.0
                c = target.scenePos()
                a = math.radians(st["angle"])
                fwd = 15.0 * SCALE
                self._draw_mountain(c.x() + fwd * math.sin(a),
                                    c.y() - fwd * math.cos(a), size_cm)

            # --- EXPLORE: small-cliff no-go box at the robot ---
            elif cmd == "NOGO":
                c = target.scenePos()
                self._draw_nogo_box(c.x(), c.y(), 10)

            # --- F5: black contact report (paired with NOGO). Format: BLACKPT,<heading>,<run> ---
            elif cmd == "BLACKPT":
                heading = float(parts[1]) if len(parts) > 1 else st["angle"]
                run     = int(parts[2])   if len(parts) > 2 else 1
                c = target.scenePos()
                st.setdefault("black_contacts", []).append(
                    {"x": c.x(), "y": c.y(), "heading": heading, "run": run})
                # provisional faint amber dot so the operator sees live contacts; the final
                # boundary/cliff decision is drawn at EXPLORE_DONE.
                self._draw_dot(c.x(), c.y(), AMBER, faint=True)

            # --- EXPLORE: mission started ---
            elif cmd == "EXPLORE":
                self.log_widget.append(
                    f"<span style='color:{CYAN};'>[EXPLORE] mission started.</span>")

            # --- EXPLORE: run finished ---
            elif cmd == "EXPLORE_DONE":
                self.classify_black_contacts(st)   # F5: cluster + draw
                st["black_contacts"] = []           # reset so next EXPLORE run starts clean
                self.log_widget.append(
                    f"<span style='color:{CYAN};'>[EXPLORE] mapping run complete.</span>")

            # --- EXPLORE: trap escape attempt ---
            elif cmd == "TRAP":
                self.log_widget.append(
                    f"<span style='color:{AMBER};'>[TRAP] escaping — scanning for opening.</span>")

            # --- EXPLORE: trap escape succeeded ---
            elif cmd == "TRAP_OK":
                self.log_widget.append(
                    f"<span style='color:{CYAN};'>[TRAP_OK] escaped successfully.</span>")

            # --- EXPLORE: trap escape failed, mission stopped ---
            elif cmd == "TRAP_FAIL":
                self.log_widget.append(
                    f"<span style='color:{RED_ALERT};'>[TRAP_FAIL] escape failed — mission stopped.</span>")

            # --- EXPLORE: heading drift corrected by re-sweep ---
            elif cmd == "DRIFT":
                try:
                    drift_deg = float(parts[1]) if len(parts) > 1 else 0.0
                    n_refs    = int(parts[2])   if len(parts) > 2 else 0
                    self.log_widget.append(
                        f"<span style='color:{AMBER};'>"
                        f"[DRIFT] heading corrected {drift_deg:+.1f}° from {n_refs} refs"
                        f"</span>")
                except Exception:
                    self.log_widget.append(
                        f"<span style='color:{AMBER};'>[DRIFT] heading corrected</span>")

            # --- SWEEPQ: start of a new object list -> clear the previous sweep markers ---
            elif cmd == "OBJN":
                self._clear_sweep(st)
                n = int(parts[1]) if len(parts) > 1 else 0
                self.log_widget.append(
                    f"<span style='color:#39FF14;'>[SWEEP] {n} object(s)</span>")

            # --- SWEEPQ: one detected object at (rel bearing, distance) ---
            elif cmd == "OBJ":
                rel = float(parts[1])
                dist_mm = float(parts[2])
                c = target.sceneBoundingRect().center()
                angle_deg = st["angle"] + rel
                angle_rad = math.radians(angle_deg)
                total_cm = dist_mm / 10.0 + SENSOR_OFFSET_CM
                ox = c.x() + total_cm * SCALE * math.sin(angle_rad)
                oy = c.y() - total_cm * SCALE * math.cos(angle_rad)
                self._draw_sweep_object(ox, oy, c, total_cm, angle_deg, st)

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
