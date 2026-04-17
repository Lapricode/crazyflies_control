#!/usr/bin/env python3
"""
Crazyflie 3D State Visualizer
pygame + PyOpenGL to get a hardware-accelerated interactive 3D

Controls:
    Left-drag  : rotate view
    Right-drag : pan / translate camera target
    Scroll     : zoom in / out
    Q / Escape : quit

Usage:
    python crazyflie_visualizer.py                    # demo mode (offline)
    python crazyflie_visualizer.py --connect          # connect to drone
    python crazyflie_visualizer.py --yaml my_lh.yaml  # custom lighthouse file

Dependencies
------------
    pip3 install pygame PyOpenGL PyOpenGL_accelerate numpy pyyaml
    pip3 install cflib   # optional - only needed with --connect
"""


import sys
import os
import math
import time
import threading
import logging
import numpy as np
import pygame
from pygame.locals import (
    DOUBLEBUF, OPENGL, QUIT, KEYDOWN,
    K_ESCAPE, K_q,
    MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION,
)
from OpenGL.GL import *
from OpenGL.GLU import *
import yaml
try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.log import LogConfig
    CFLIB_AVAILABLE = True
except ImportError:
    CFLIB_AVAILABLE = False


# ==============================================================================
#  CONFIGURATION
# ==============================================================================

TARGET_FPS = 60

# Ground grid
GROUND_N_CELLS = 4    # half-extent: total grid spans 2*N cells each side
GROUND_CELL_M = 1.0  # metres per cell

# Crazyflie geometry
# l = distance from body centre to each motor (Crazyflie 2.x ≈ 46 mm)
DRONE_ARM_L    = 0.046   # metres
DRONE_BODY_YAW = math.pi / 4   # initial body yaw for arm geometry

# Files / connection
LIGHTHOUSE_YAML = "lighthouse_config.yaml"
CF_URI          = "radio://0/80/2M/E7E7E7E701"
LOG_PERIOD_MS   = 20


# ==============================================================================
#  CAMERA
# ==============================================================================

class Camera:
    """Arcball-style camera: azimuth/elevation rotation, pan, zoom."""

    def __init__(self):
        self.az   = 45.0                          # azimuth  (degrees, around Z)
        self.el   = 30.0                          # elevation (degrees, up/down)
        self.dist = 6.0                           # distance from target (m)
        self.tgt  = np.array([0., 0., 1.])        # world look-at point

    # ── apply to OpenGL modelview ──────────────────────────────────────────

    def apply(self):
        glTranslatef(0, 0, -self.dist)
        glRotatef(-self.el, 1, 0, 0)
        glRotatef( self.az, 0, 0, 1)
        glTranslatef(-self.tgt[0], -self.tgt[1], -self.tgt[2])

    # ── mouse interactions ─────────────────────────────────────────────────

    def rotate(self, dx, dy):
        self.az  += dx * 0.35
        self.el   = max(-89., min(89., self.el - dy * 0.35))

    def pan(self, dx, dy):
        """Translate target in camera-right / camera-up plane."""
        az = math.radians(self.az)
        el = math.radians(self.el)
        right = np.array([ math.cos(az), -math.sin(az), 0.])
        up    = np.array([ math.sin(az) * math.cos(el),
                           math.cos(az) * math.cos(el),
                           math.sin(el)])
        speed = 0.0015 * self.dist
        self.tgt -= right * dx * speed
        self.tgt += up    * dy * speed

    def zoom(self, sign):
        factor = 0.90 if sign > 0 else 1.11
        self.dist = max(0.3, min(100., self.dist * factor))


# ==============================================================================
#  THREAD-SAFE DRONE STATE
# ==============================================================================

class DroneState:
    def __init__(self):
        self._lock     = threading.Lock()
        self.position  = np.zeros(3)
        self.rotation  = np.eye(3)    # body → world, columns = body axes in world
        self.connected = False

    def update(self, pos, rot):
        with self._lock:
            self.position = np.asarray(pos, float)
            self.rotation = np.asarray(rot, float)

    def get(self):
        with self._lock:
            return self.position.copy(), self.rotation.copy()


# ==============================================================================
#  LOW-LEVEL OPENGL PRIMITIVES
# ==============================================================================

def _push_transform(origin, R3):
    """Push modelview matrix: world-space rigid body transform."""
    m = np.eye(4, dtype=np.float32)
    m[:3, :3] = R3
    m[:3,  3] = origin
    glPushMatrix()
    glMultMatrixf(m.T.flatten())   # OpenGL stores matrices column-major


def _z_to_dir(d):
    """
    Compute (axis, angle_deg) rotation that maps +Z to direction d.
    Used to orient GLU cylinders/cones.
    """
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    z = np.array([0., 0., 1.])
    axis  = np.cross(z, d)
    s     = np.linalg.norm(axis)
    c     = float(np.dot(z, d))
    if s < 1e-9:
        # parallel: no rotation; anti-parallel: 180° around X
        return (np.array([1., 0., 0.]), 0.) if c > 0 else (np.array([1., 0., 0.]), 180.)
    return (axis / s, math.degrees(math.atan2(s, c)))


def _sphere(radius, slices=18, stacks=14):
    q = gluNewQuadric()
    gluQuadricNormals(q, GLU_SMOOTH)
    gluSphere(q, radius, slices, stacks)
    gluDeleteQuadric(q)


def _arrow(direction, length, shaft_r, tip_r, tip_len, slices=14):
    """
    Draw an arrow from the current origin toward `direction`.
    Shaft = cylinder, tip = cone.
    """
    shaft_len = max(0., length - tip_len)
    ax, ang   = _z_to_dir(direction)
    glPushMatrix()
    glRotatef(ang, *ax)
    q = gluNewQuadric()
    gluQuadricNormals(q, GLU_SMOOTH)
    # shaft
    gluCylinder(q, shaft_r, shaft_r, shaft_len, slices, 1)
    # tip
    glPushMatrix()
    glTranslatef(0, 0, shaft_len)
    gluCylinder(q, tip_r, 0., tip_len, slices, 1)
    glPopMatrix()
    gluDeleteQuadric(q)
    glPopMatrix()


# ==============================================================================
#  HIGH-LEVEL SCENE ELEMENTS
# ==============================================================================

_AXIS_DIRS   = [(1,0,0), (0,1,0), (0,0,1)]
_AXIS_COLORS = [(1,0,0), (0,1,0), (0,0,1)]   # X=red, Y=green, Z=blue


def draw_frame(origin, R, axis_len, sphere_r):
    """
    Coordinate frame: black origin sphere + red/green/blue axis arrows.

    Parameters
    ----------
    origin    : (3,) world position
    R         : (3,3) rotation matrix (columns = frame axes in world)
    axis_len  : arrow length in metres
    sphere_r  : origin sphere radius in metres
    """
    shaft_r = axis_len * 0.018
    tip_r   = axis_len * 0.048
    tip_len = axis_len * 0.22

    _push_transform(origin, R)

    # origin sphere (near-black)
    glColor3f(0.06, 0.06, 0.06)
    _sphere(sphere_r)

    for color, direction in zip(_AXIS_COLORS, _AXIS_DIRS):
        glColor3f(*color)
        _arrow(direction, axis_len, shaft_r, tip_r, tip_len)

    glPopMatrix()


def draw_ground(n_cells, cell_m):
    """
    Grey grid at z = 0.

    Parameters
    ----------
    n_cells : half-extent in cells (grid goes from -n*cell_m to +n*cell_m)
    cell_m  : cell side in metres
    """
    glDisable(GL_LIGHTING)
    glColor3f(0.32, 0.36, 0.32)
    glLineWidth(1.0)
    size = n_cells * cell_m
    glBegin(GL_LINES)
    for i in range(-n_cells, n_cells + 1):
        c = i * cell_m
        glVertex3f(c,    -size, 0.);  glVertex3f(c,    size, 0.)
        glVertex3f(-size, c,   0.);  glVertex3f(size,  c,   0.)
    glEnd()
    glEnable(GL_LIGHTING)


def draw_quadrotor(position, R, arm_l, body_yaw0=math.pi / 4):
    """
    Draw the quadrotor body at the given world pose.

    Geometry logic is preserved verbatim from the reference
    quadrotor_visualize() function; only the rendering backend changes
    (OpenGL instead of matplotlib).

    Parameters
    ----------
    position  : (3,) world position
    R         : (3,3) body→world rotation matrix
    arm_l     : arm half-length in metres (motor distance from centre)
    body_yaw0 : initial body yaw offset (radians)
    """
    l    = arm_l
    by0  = body_yaw0

    # ── body geometry (verbatim from reference) ────────────────────────────
    body_orient0 = np.array([
        [math.cos(by0), -math.sin(by0), 0.],
        [math.sin(by0),  math.cos(by0), 0.],
        [0.,             0.,            1.]
    ])

    p_body    = np.array([0., 0., 0.])
    p_antenna = l / 4 * np.array([-1. / math.sqrt(2), 1. / math.sqrt(2), 0.])
    p_motor1  = np.array([0.,  l, 0.])
    p_motor2  = np.array([ l, 0., 0.])
    p_motor3  = np.array([0., -l, 0.])
    p_motor4  = np.array([-l, 0., 0.])

    local_pts = np.vstack(
        [p_body, p_antenna, p_motor1, p_motor2, p_motor3, p_motor4]
    ).T                                       # (3, 6)
    body_pts  = body_orient0 @ local_pts      # (3, 6) – apply body yaw
    world_pts = (R @ body_pts).T + position   # (6, 3) – world transform
    pB, pA, pM1, pM2, pM3, pM4 = world_pts
    pMs = [pM1, pM2, pM3, pM4]

    # ── render ─────────────────────────────────────────────────────────────
    motor_r = l * 0.18
    body_r  = l * 0.12

    glDisable(GL_LIGHTING)

    # arms: centre → motors (red)
    glColor3f(0.85, 0.15, 0.15)
    glLineWidth(2.5)
    glBegin(GL_LINES)
    for pM in pMs:
        glVertex3f(*pB); glVertex3f(*pM)
    glEnd()

    # antenna wire (dark)
    glColor3f(0.18, 0.18, 0.18)
    glLineWidth(1.5)
    glBegin(GL_LINES)
    glVertex3f(*pB); glVertex3f(*pA)
    glEnd()

    glEnable(GL_LIGHTING)

    # centre sphere (dark body)
    glColor3f(0.12, 0.12, 0.16)
    glPushMatrix(); glTranslatef(*pB); _sphere(body_r);  glPopMatrix()

    # motor spheres (green)
    glColor3f(0.10, 0.78, 0.10)
    for pM in pMs:
        glPushMatrix(); glTranslatef(*pM); _sphere(motor_r); glPopMatrix()

    # antenna tip
    glColor3f(0.14, 0.14, 0.14)
    glPushMatrix(); glTranslatef(*pA); _sphere(l * 0.04); glPopMatrix()


# ==============================================================================
#  LIGHTHOUSE YAML LOADER
# ==============================================================================

def load_lighthouses(yaml_path):
    """
    Parse a Crazyflie lighthouse_system_configuration YAML.
    Returns list of (label, origin_vec3, rotation_3x3) sorted by index.
    Index 0 → label "1", index 1 → label "2", etc.
    """
    if not os.path.exists(yaml_path):
        print(f"[warn] Lighthouse YAML not found: {yaml_path!r} – no stations shown.")
        return []
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    stations = []
    for idx, geo in sorted(data.get("geos", {}).items()):
        origin = np.array(geo["origin"], dtype=float)
        rot    = np.array(geo["rotation"], dtype=float)   # 3×3 row-major
        stations.append((str(idx + 1), origin, rot))
    print(f"[info] Loaded {len(stations)} lighthouse station(s).")
    return stations


# ==============================================================================
#  CRAZYFLIE CONNECTION THREAD
# ==============================================================================

def _euler_zyx_to_R(roll_deg, pitch_deg, yaw_deg):
    """ZYX Euler → body-to-world rotation matrix (matches Crazyflie convention)."""
    r, p, y = map(math.radians, [roll_deg, pitch_deg, yaw_deg])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,            0           ],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    return Rz @ Ry @ Rx


class CrazyflieThread(threading.Thread):
    """
    Background thread that connects to a Crazyflie and streams state estimates
    into a shared DroneState object.

    Logs: stateEstimate.{x, y, z, roll, pitch, yaw}
    """

    def __init__(self, uri, state: DroneState):
        super().__init__(daemon=True, name="cf-thread")
        self.uri    = uri
        self.state  = state
        self._stop  = threading.Event()

    def run(self):
        if not CFLIB_AVAILABLE:
            print("[cf] cflib not installed – running without drone.")
            return
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers()
        try:
            with SyncCrazyflie(self.uri,
                               cf=Crazyflie(rw_cache="./cache")) as scf:
                self.state.connected = True
                print(f"[cf] Connected: {self.uri}")

                lg = LogConfig(name="StateEst", period_in_ms=LOG_PERIOD_MS)
                lg.add_variable("stateEstimate.x",     "float")
                lg.add_variable("stateEstimate.y",     "float")
                lg.add_variable("stateEstimate.z",     "float")
                lg.add_variable("stateEstimate.roll",  "float")
                lg.add_variable("stateEstimate.pitch", "float")
                lg.add_variable("stateEstimate.yaw",   "float")

                def _cb(timestamp, data, logconf):
                    pos = [data["stateEstimate.x"],
                           data["stateEstimate.y"],
                           data["stateEstimate.z"]]
                    R   = _euler_zyx_to_R(data["stateEstimate.roll"],
                                          data["stateEstimate.pitch"],
                                          data["stateEstimate.yaw"])
                    self.state.update(pos, R)

                scf.cf.log.add_config(lg)
                lg.data_received_cb.add_callback(_cb)
                lg.start()
                while not self._stop.is_set():
                    time.sleep(0.05)
                lg.stop()

        except Exception as exc:
            print(f"[cf] Connection error: {exc}")
            self.state.connected = False

    def stop(self):
        self._stop.set()


# ==============================================================================
#  2-D HUD OVERLAY
# ==============================================================================

def _upload_text_texture(font, lines):
    """
    Render a list of (str, rgb_float_tuple) lines to an RGBA OpenGL texture.
    Returns (tex_id, pixel_width, pixel_height).
    """
    surfaces = [font.render(text, True,
                            tuple(int(c * 255) for c in col))
                for text, col in lines]
    if not surfaces:
        return None, 0, 0
    tw   = max(s.get_width()  for s in surfaces)
    lh   = surfaces[0].get_height() + 4
    th   = lh * len(surfaces) + 4
    canvas = pygame.Surface((tw, th), pygame.SRCALPHA)
    canvas.fill((0, 0, 0, 0))
    for i, s in enumerate(surfaces):
        canvas.blit(s, (0, i * lh + 2))
    raw = pygame.image.tostring(canvas, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    return tex, tw, th


def draw_hud(lines, font, win_w, win_h):
    """Blit text lines onto the upper-left corner of the screen."""
    if not lines:
        return
    tex, tw, th = _upload_text_texture(font, lines)
    if tex is None:
        return

    glDisable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glMatrixMode(GL_PROJECTION)
    glPushMatrix(); glLoadIdentity()
    glOrtho(0, win_w, win_h, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix(); glLoadIdentity()

    x, y = 10, 10
    glColor4f(1, 1, 1, 1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 1); glVertex2f(x,      y)
    glTexCoord2f(1, 1); glVertex2f(x + tw, y)
    glTexCoord2f(1, 0); glVertex2f(x + tw, y + th)
    glTexCoord2f(0, 0); glVertex2f(x,      y + th)
    glEnd()
    glDeleteTextures(1, [tex])

    glDisable(GL_TEXTURE_2D)
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW);  glPopMatrix()
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)


# ==============================================================================
#  ENTRY POINT
# ==============================================================================

def main():
    # ── parse CLI ──────────────────────────────────────────────────────────
    connect_cf = "--connect" in sys.argv
    yaml_path  = LIGHTHOUSE_YAML
    for i, a in enumerate(sys.argv):
        if a == "--yaml" and i + 1 < len(sys.argv):
            yaml_path = sys.argv[i + 1]

    # ── shared state ───────────────────────────────────────────────────────
    drone_state = DroneState()
    lighthouses = load_lighthouses(yaml_path)

    # ── optional drone thread ──────────────────────────────────────────────
    cf_thread = None
    if connect_cf:
        cf_thread = CrazyflieThread(CF_URI, drone_state)
        cf_thread.start()
    else:
        print("[info] Offline mode – pass --connect to stream from drone.")

    # ── pygame + OpenGL init ───────────────────────────────────────────────
    pygame.init()
    display_info = pygame.display.Info()
    WINDOW_W = 3/4 * display_info.current_w
    WINDOW_H = 3/4 * display_info.current_h
    pygame.display.set_mode((WINDOW_W, WINDOW_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Crazyflie 3D Visualizer")
    hud_font = pygame.font.SysFont("monospace", 15)

    # depth & blending
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glShadeModel(GL_SMOOTH)
    glClearColor(0.12, 0.13, 0.16, 1.0)

    # lighting – single diffuse key light (headlight-style: set before camera)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.28, 0.28, 0.28, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.85, 0.85, 0.85, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.20, 0.20, 0.20, 1.0])
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # perspective projection
    glMatrixMode(GL_PROJECTION); glLoadIdentity()
    gluPerspective(55., WINDOW_W / WINDOW_H, 0.005, 200.)
    glMatrixMode(GL_MODELVIEW)

    camera   = Camera()
    clock    = pygame.time.Clock()
    prev_xy  = None
    cur_btn  = None

    running = True
    while running:
        clock.tick(TARGET_FPS)

        # ── events ──────────────────────────────────────────────────────────
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False

            elif evt.type == KEYDOWN:
                if evt.key in (K_ESCAPE, K_q):
                    running = False

            elif evt.type == MOUSEBUTTONDOWN:
                prev_xy = pygame.mouse.get_pos()
                if   evt.button == 1: cur_btn = "rotate"
                elif evt.button == 3: cur_btn = "pan"
                elif evt.button == 4: camera.zoom(+1)   # scroll up   = zoom in
                elif evt.button == 5: camera.zoom(-1)   # scroll down = zoom out

            elif evt.type == MOUSEBUTTONUP:
                cur_btn = None; prev_xy = None

            elif evt.type == MOUSEMOTION and prev_xy and cur_btn:
                cx, cy   = pygame.mouse.get_pos()
                dx, dy   = cx - prev_xy[0], cy - prev_xy[1]
                prev_xy  = (cx, cy)
                if cur_btn == "rotate": camera.rotate(dx, dy)
                elif cur_btn == "pan":  camera.pan(dx, dy)

        # ── render 3D scene ─────────────────────────────────────────────────
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # headlight: set BEFORE camera so it stays fixed relative to viewer
        glLightfv(GL_LIGHT0, GL_POSITION, [1., 2., 4., 0.])

        camera.apply()

        # 1 ── world frame  (1 m axes, 5 cm radius sphere = 10 cm diameter)
        draw_frame(np.zeros(3), np.eye(3), axis_len=1.0, sphere_r=0.05)

        # 2 ── ground grid
        draw_ground(GROUND_N_CELLS, GROUND_CELL_M)

        # 3 ── lighthouse stations  (10 cm axes, 0.5 cm radius sphere = 1 cm diam)
        for _name, origin, rot in lighthouses:
            draw_frame(origin, rot, axis_len=0.10, sphere_r=0.005)

        # 4 ── crazyflie frame + body
        pos, rot = drone_state.get()
        draw_frame(pos, rot, axis_len=0.10, sphere_r=0.005)
        draw_quadrotor(pos, rot, DRONE_ARM_L, DRONE_BODY_YAW)

        # ── HUD overlay ─────────────────────────────────────────────────────
        status_str  = "CONNECTED" if drone_state.connected else "OFFLINE (default pose)"
        status_col  = (0.25, 1.0, 0.35) if drone_state.connected else (1.0, 0.55, 0.15)
        p = pos
        hud = [
            (f"Drone  : {status_str}",
             status_col),
            (f"Position : x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f} m",
             (0.90, 0.90, 0.90)),
            (f"LH stations : {len(lighthouses)}",
             (0.65, 0.65, 1.00)),
            ("LMB drag: rotate  |  RMB drag: pan  |  Scroll: zoom  |  Q: quit",
             (0.50, 0.50, 0.50)),
        ]
        draw_hud(hud, hud_font, WINDOW_W, WINDOW_H)

        pygame.display.flip()

    # ── cleanup ─────────────────────────────────────────────────────────────
    if cf_thread:
        cf_thread.stop()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
