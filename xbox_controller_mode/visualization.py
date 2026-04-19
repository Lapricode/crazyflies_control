#!/usr/bin/env python3
"""
Crazyflie 3D State Visualizer — Visualization Module

An interactive, hardware-accelerated 3D visualization tool for Crazyflie drones,
built using pygame and PyOpenGL. It provides real-time rendering of drone states,
camera controls, and configurable scene elements.

Controls:
    All controls for mouse, general, scene, drones, camera, control, etc., are accessible in-app via the Key Bindings menu (press "I").

Usage:
    Run without drones (add manually in-app):
        python visualization.py

    Run with a custom lighthouse configuration:
        python visualization.py --yaml my_lh.yaml

Dependencies:
    Required:
        pip3 install pygame PyOpenGL PyOpenGL_accelerate numpy pyyaml

    Optional (for real drone connectivity):
        pip3 install cflib
"""


import sys
import os
import math
import time
import numpy as np
import pygame
from pygame.locals import (
    DOUBLEBUF, OPENGL, QUIT, KEYDOWN, RESIZABLE, VIDEORESIZE,
    K_ESCAPE, K_i, K_w, K_s, K_d, K_f, K_l, K_h, K_t, K_r, K_a, K_c,
    K_F11, K_PAGEUP, K_PAGEDOWN, K_TAB, K_RETURN,
    K_UP, K_DOWN, K_LEFT, K_RIGHT,
    MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION,
    KMOD_SHIFT, KMOD_CTRL, KMOD_ALT,
)
from OpenGL.GL import *
from OpenGL.GLU import *
import yaml
from collections import deque

from crazyflie import DroneState, CrazyflieThread, MAX_PWM
from xbox_controller import XboxController


# ==============================================================================
# CONFIGURATION
# ==============================================================================

TARGET_FPS = 60

# Ground grid
GROUND_N_CELLS = 5    # half-extent: total grid spans 2*N cells each side
GROUND_CELL_M  = 1.0  # metres per cell

# Crazyflie geometry
DRONE_ARM_L    = 0.046            # distance from body centre to each motor (Crazyflie 2.x ~= 46 mm)
DRONE_BODY_YAW = -3/4 * math.pi  # assuming body_yaw0 = 0 is motor 1 at +y, motor 2 at +x, clockwise

# Files
LIGHTHOUSE_YAML = "config/lighthouse_config.yaml"  # configuration file for the lighthouse stations
DRONES_YAML     = "config/crazyflies_config.yaml"  # configuration file for drone fleet presets

# Key bindings info menu (hud_font, right side of screen)
INFO_MENU_LINES = [
    ("  KEY BINDINGS",                             (0.10, 0.10, 0.10)),
    ("",                                           (0.00, 0.00, 0.00)),
    ("  LMB drag       : rotate view",             (0.20, 0.20, 0.20)),
    ("  RMB drag       : pan view",                (0.20, 0.20, 0.20)),
    ("  Scroll         : zoom in / out",           (0.20, 0.20, 0.20)),
    ("  Middle-click   : reset view",              (0.20, 0.20, 0.20)),
    ("  F11            : toggle fullscreen",       (0.20, 0.20, 0.20)),
    ("  PgUp / PgDn    : scroll HUD",              (0.20, 0.20, 0.20)),
    ("  I              : close this menu",         (0.20, 0.20, 0.20)),
    ("  Escape         : quit",                    (0.20, 0.20, 0.20)),
    ("",                                           (0.00, 0.00, 0.00)),
    ("  Shift + W      : world frame on/off",      (0.45, 0.10, 0.10)),
    ("  Shift + D      : drone body on/off",       (0.45, 0.10, 0.10)),
    ("  Shift + F      : drone frame on/off",      (0.45, 0.10, 0.10)),
    ("  Shift + L      : lighthouses on/off",      (0.45, 0.10, 0.10)),
    ("  Shift + H      : HUD text on/off",         (0.45, 0.10, 0.10)),
    ("  Shift + C      : console prints on/off",   (0.45, 0.10, 0.10)),
    ("  Shift + T      : scene labels on/off",     (0.45, 0.10, 0.10)),
    ("  Shift + 0      : show/hide all drones",    (0.45, 0.10, 0.10)),
    ("  Shift + 1-8    : show/hide drone N",       (0.45, 0.10, 0.10)),
    ("  Alt   + 0      : reset camera tracking",   (0.45, 0.10, 0.45)),
    ("  Alt   + 1-8    : track drone N",           (0.45, 0.10, 0.45)),
    ("",                                           (0.00, 0.00, 0.00)),
    ("  Ctrl  + A      : add drone",               (0.10, 0.10, 0.45)),
    ("  Ctrl  + D      : delete drone",            (0.10, 0.10, 0.45)),
    ("  Ctrl  + R      : connect drone(s)",        (0.10, 0.10, 0.45)),
    ("  Ctrl  + S      : save drone config",       (0.10, 0.10, 0.45)),
    ("  Ctrl  + L      : load drone config",       (0.10, 0.10, 0.45)),
    ("  Ctrl  + P      : change drone params",     (0.10, 0.10, 0.45)),
    ("  Ctrl  + 1-8    : blink LED of drone N",    (0.10, 0.10, 0.45)),
    ("",                                           (0.00, 0.00, 0.00)),
    ("  Ctrl  + C      : flight controller panel", (0.10, 0.45, 0.10)),
    ("    F            : takeoff",                 (0.10, 0.45, 0.10)),
    ("    L            : land",                    (0.10, 0.45, 0.10)),
    ("    W            : up",                      (0.10, 0.45, 0.10)),
    ("    S            : down",                    (0.10, 0.45, 0.10)),
    ("    A / D        : yaw ccw / cw",            (0.10, 0.45, 0.10)),
    ("    Up / Down    : forward / backward",      (0.10, 0.45, 0.10)),
    ("    Left / Right : slide left / right",      (0.10, 0.45, 0.10)),
]


# ==============================================================================
# CAMERA
# ==============================================================================

class Camera:
    """
    Arcball-style camera:
        - azimuth/elevation rotation (compute az, el)
        - pan (compute lookat)
        - zoom (compute dist)
    """

    def __init__(self):
        self.reset_view()

    # apply to OpenGL modelview

    def apply(self):
        glTranslatef(0, 0, -self.dist)
        glRotatef(-self.el, 1, 0, 0)
        glRotatef( self.az, 0, 0, 1)
        glTranslatef(-self.lookat[0], -self.lookat[1], -self.lookat[2])

    # mouse interactions

    def reset_view(self):
        self.az     = -20.0                    # initial azimuth (degrees, around Z)
        self.el     =  60.0                    # initial elevation (degrees, up/down)
        self.lookat = np.array([0., 0., 0.])   # world look-at target point
        self.dist   =  5.0                     # initial distance from the world look-at point (m)

    def rotate(self, dx, dy):
        self.az  += dx * 0.4
        self.el   = max(-89., min(89., self.el - dy * 0.4))

    def pan(self, dx, dy):
        az = math.radians(self.az)
        el = math.radians(self.el)
        right = np.array([ math.cos(az), -math.sin(az), 0.])
        up    = np.array([ math.sin(az) * math.cos(el),
                           math.cos(az) * math.cos(el),
                           math.sin(el)])
        speed = 0.002 * self.dist
        self.lookat -= right * dx * speed
        self.lookat += up    * dy * speed

    def zoom(self, sign):
        factor = 0.99 if sign > 0 else 1.01
        self.dist = max(0.1, min(100., self.dist * factor))


# ==============================================================================
# DRONE ENTRY
# ==============================================================================

class DroneEntry:
    """
    Wraps a single Crazyflie's identity, state, thread, and visual properties.
    """

    def __init__(self, num, uri, default_pos):
        self.num         = num                                 # drone number (1-indexed, gaps allowed)
        self.uri         = uri                                 # full radio URI
        self.state       = DroneState(default_pos)             # shared thread-safe state
        self.thread      = None                                # CrazyflieThread when running
        self.visible     = True
        self.default_pos = np.asarray(default_pos, float)

    def start_connection(self):
        """Stop any existing thread and start a fresh one."""
        if self.thread is not None:
            self.thread.stop()
            self.thread.join(timeout = 2.0)
        self.state.connected = False
        self.thread = CrazyflieThread(self.uri, self.state)
        self.thread.start()
        print(f"[INFO] Connecting CF {self.num} → {self.uri}")

    def stop_connection(self):
        """Stop the connection thread cleanly."""
        if self.thread is not None:
            self.thread.stop()
            self.thread.join(timeout = 2.0)
            self.thread = None
        self.state.connected = False


# ==============================================================================
# MODAL DIALOG SYSTEM
# ==============================================================================

class TextField:
    """Single editable text field inside a ModalDialog."""

    DIGITS = "0123456789"
    HEX    = "0123456789ABCDEFabcdef"

    def __init__(self, label, default = "", allowed = None, hint = ""):
        self.label   = label
        self.value   = default
        self.allowed = allowed   # None = any printable; str = set of allowed characters
        self.hint    = hint      # extra hint shown beside the label

    def handle_key(self, evt):
        if evt.key == pygame.K_BACKSPACE:
            self.value = self.value[:-1]
        elif evt.unicode and (self.allowed is None or evt.unicode in self.allowed):
            self.value += evt.unicode


class ModalDialog:
    """
    Centered modal dialog with multiple text fields.
    Keyboard:
        Tab / Shift+Tab : cycle through fields
        Enter           : confirm
        Escape          : cancel
    """

    def __init__(self, title, fields):
        self.title     = title
        self.fields    = fields
        self.active    = 0       # index of the currently focused field
        self.confirmed = False
        self.cancelled = False

    def handle_event(self, evt):
        if evt.type != KEYDOWN:
            return
        mods = pygame.key.get_mods()
        if evt.key == K_ESCAPE:
            self.cancelled = True
        elif evt.key in (K_RETURN, pygame.K_KP_ENTER):
            self.confirmed = True
        elif evt.key == K_TAB:
            step = -1 if (mods & KMOD_SHIFT) else +1
            self.active = (self.active + step) % len(self.fields)
        else:
            self.fields[self.active].handle_key(evt)


def draw_modal(modal, font, win_w, win_h, cursor_on):
    """
    Render the ModalDialog as a centered panel over a dimmed background.
    """
    PAD       = 18
    LINE_H    = font.get_height() + 8
    PANEL_W   = 460
    PANEL_H   = (PAD
                 + LINE_H                                    # title
                 + PAD // 2                                  # gap
                 + len(modal.fields) * (LINE_H + LINE_H + 6) # label + input per field
                 + LINE_H                                    # bottom hint
                 + PAD)

    BG_COL     = (245, 244, 240, 230)
    FRAME_COL  = (60,  60,  60)
    TEXT_COL   = (20,  20,  20)
    TITLE_COL  = (10,  10,  80)
    HINT_COL   = (100, 100, 100)
    INPUT_COL  = (255, 255, 255)
    ACTIVE_COL = (210, 225, 255)

    surf = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
    surf.fill(BG_COL)
    pygame.draw.rect(surf, FRAME_COL, (0, 0, PANEL_W, PANEL_H), 2)

    y = PAD
    # title
    t = font.render(modal.title, True, TITLE_COL)
    surf.blit(t, ((PANEL_W - t.get_width()) // 2, y))
    y += LINE_H + PAD // 2

    for i, field in enumerate(modal.fields):
        # field label (with optional hint)
        label_str = field.label + (f"  [{field.hint}]" if field.hint else "") + " :"
        surf.blit(font.render(label_str, True, TEXT_COL), (PAD, y))
        y += LINE_H
        # input box
        box = pygame.Rect(PAD, y, PANEL_W - 2 * PAD, LINE_H)
        pygame.draw.rect(surf, ACTIVE_COL if i == modal.active else INPUT_COL, box)
        pygame.draw.rect(surf, FRAME_COL, box, 1)
        display = field.value + ("|" if i == modal.active and cursor_on else "")
        surf.blit(font.render(display, True, TEXT_COL), (box.x + 4, box.y + 2))
        y += LINE_H + 6

    # bottom keyboard hint
    hint = font.render("Enter: confirm   Tab: next field   Esc: cancel", True, HINT_COL)
    surf.blit(hint, ((PANEL_W - hint.get_width()) // 2, y))

    # upload as texture and draw centered with dimmed background
    sw, sh = surf.get_size()
    raw = pygame.image.tostring(surf, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sw, sh, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    ox = (win_w - sw) // 2
    oy = (win_h - sh) // 2

    _begin_2d(win_w, win_h)
    _draw_filled_rect(0, 0, win_w, win_h, 0., 0., 0., 0.4)  # dim backdrop
    _draw_quad_tex(tex, ox, oy, sw, sh)
    glDeleteTextures(1, [tex])
    _end_2d()


# ==============================================================================
# LOW-LEVEL OPENGL PRIMITIVES
# ==============================================================================

def _push_transform(origin, R3):
    """
    Push modelview matrix: world-space rigid body transform
    """
    m = np.eye(4, dtype = np.float32)
    m[:3, :3] = R3
    m[:3,  3] = origin
    glPushMatrix()
    glMultMatrixf(m.T.flatten())  # OpenGL stores matrices column-major


def _z_to_dir(d):
    """
    Compute (axis, angle_deg) rotation that maps +Z to direction d.
    Used to orient GLU cylinders/cones.
    """
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    z = np.array([0., 0., 1.])
    axis = np.cross(z, d)
    s    = np.linalg.norm(axis)
    c    = float(np.dot(z, d))
    if s < 1e-9:
        # parallel: no rotation or anti-parallel: rotate 180° around X
        return (np.array([1., 0., 0.]), 0.) if c > 0 else (np.array([1., 0., 0.]), 180.)
    return (axis / s, math.degrees(math.atan2(s, c)))


def _sphere(radius, slices = 20, stacks = 15):
    """
    Draw a sphere of specific radius
    """
    q = gluNewQuadric()
    gluQuadricNormals(q, GLU_SMOOTH)
    gluSphere(q, radius, slices, stacks)
    gluDeleteQuadric(q)


def _arrow(direction, length, shaft_r, tip_r, tip_len, slices = 20):
    """
    Draw an arrow from the current origin toward 'direction'.
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
# HIGH-LEVEL SCENE ELEMENTS
# ==============================================================================

AXIS_DIRS   = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # directions of the frame axes
AXIS_COLORS = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]   # X = red, Y = green, Z = blue


def draw_frame(origin, R, axis_len, sphere_r):
    """
    Coordinate frame: black origin sphere + red/green/blue axis arrows.
    Parameters:
        origin   : (3,) world position
        R        : (3, 3) rotation matrix (columns = frame axes in world)
        axis_len : arrow length in metres
        sphere_r : origin sphere radius in metres
    """
    shaft_r = axis_len * 0.01
    tip_r   = axis_len * 0.04
    tip_len = axis_len * 0.2

    _push_transform(origin, R)

    # origin sphere
    glColor3f(0.05, 0.05, 0.05)
    _sphere(sphere_r)

    for color, direction in zip(AXIS_COLORS, AXIS_DIRS):
        glColor3f(*color)
        _arrow(direction, axis_len, shaft_r, tip_r, tip_len)

    glPopMatrix()


def draw_ground(n_cells, cell_m):
    """
    Grey grid at z = 0.
    Parameters:
        n_cells : half-extent in cells (grid goes from -n*cell_m to +n*cell_m)
        cell_m  : cell side in metres
    """
    glDisable(GL_LIGHTING)
    glColor3f(0.2, 0.2, 0.2)
    glLineWidth(1.0)
    size = n_cells * cell_m
    glBegin(GL_LINES)
    for i in range(-n_cells, n_cells + 1):
        c = i * cell_m
        glVertex3f(c,    -size, 0.);  glVertex3f(c,    size, 0.)
        glVertex3f(-size, c,   0.);  glVertex3f(size,  c,   0.)
    glEnd()
    glEnable(GL_LIGHTING)


def draw_quadrotor(pwb, Rwb, arm_l, body_yaw0 = math.pi / 4, control_inputs = None):
    """
    Draw the quadrotor body at the given world pose.
    Parameters:
        pwb            : (3,) position of the body wrt world
        Rwb            : (3, 3) rotation matrix of the body wrt world
        arm_l          : arm half-length in metres (motor distance from centre)
        body_yaw0      : initial body yaw offset (radians)
        control_inputs : (4,) motor PWM values [m1, m2, m3, m4] (0 - 65535),
                         or None to skip control thrust visualization
    """

    # body geometry
    body_orient0 = np.array([
        [math.cos(body_yaw0), -math.sin(body_yaw0), 0.],
        [math.sin(body_yaw0),  math.cos(body_yaw0), 0.],
        [0.,                   0.,                  1.]
    ])

    p_body    = np.array([0., 0., 0.])
    p_antenna = arm_l / 2 * np.array([-1. / math.sqrt(2), 1. / math.sqrt(2), 0.])
    p_motor1  = np.array([0.,  arm_l, 0.])
    p_motor2  = np.array([ arm_l, 0., 0.])
    p_motor3  = np.array([0., -arm_l, 0.])
    p_motor4  = np.array([-arm_l, 0., 0.])

    local_pts = np.vstack([p_body, p_antenna, p_motor1, p_motor2, p_motor3, p_motor4]).T  # (3, 6)
    body_pts  = body_orient0 @ local_pts              # (3, 6) - apply body yaw
    world_pts = (Rwb @ body_pts).T + pwb              # (6, 3) - world transform
    pB, pA, pM1, pM2, pM3, pM4 = world_pts
    pMs = [pM1, pM2, pM3, pM4]

    # render the quadrotor
    motor_r = arm_l * 0.15
    body_r  = arm_l * 0.1

    glDisable(GL_LIGHTING)

    # arms: centre → motors
    glColor3f(0.8, 0.2, 0.2)
    glLineWidth(10.0)
    glBegin(GL_LINES)
    for pM in pMs:
        glVertex3f(*pB)
        glVertex3f(*pM)
    glEnd()

    # antenna wire
    glColor3f(0.2, 0.2, 0.2)
    glLineWidth(5.0)
    glBegin(GL_LINES)
    glVertex3f(*pB)
    glVertex3f(*pA)
    glEnd()

    # motor thrust visualization: blue vertical bars above each motor,
    # proportional to PWM (0 - 65535), max length = arm_l
    # (geometry logic preserved verbatim from reference quadrotor_visualize)
    if control_inputs is not None:
        controls_max_length = arm_l
        ctrl = np.asarray(control_inputs, float)
        control_lengths = controls_max_length * (ctrl / MAX_PWM)
        motor_body_pts = body_pts[:, 2:].copy()              # (3, 4) - motor cols only
        motor_body_pts[2, :] += control_lengths              # shift along body Z
        pCs = (Rwb @ motor_body_pts).T + pwb                 # (4, 3) - world frame
        glColor3f(0.1, 0.2, 0.9)
        glLineWidth(5.0)
        glBegin(GL_LINES)
        for i in range(4):
            glVertex3f(*pMs[i])
            glVertex3f(*pCs[i])
        glEnd()

    glEnable(GL_LIGHTING)

    # antenna tip
    glColor3f(0.1, 0.1, 0.1)
    glPushMatrix()
    glTranslatef(*pA)
    _sphere(arm_l * 0.04)
    glPopMatrix()

    # centre sphere (blue-tinted body)
    glColor3f(0.1, 0.1, 0.4)
    glPushMatrix()
    glTranslatef(*pB)
    _sphere(body_r)
    glPopMatrix()

    # motor spheres (green)
    glColor3f(0.20, 0.8, 0.20)
    for pM in pMs:
        glPushMatrix()
        glTranslatef(*pM)
        _sphere(motor_r)
        glPopMatrix()


# ==============================================================================
# LIGHTHOUSE YAML LOADER
# ==============================================================================

def load_lighthouses(yaml_path):
    """
    Parse a Crazyflie lighthouse system configuration YAML.
    Returns list of (label, origin_vec3, rotation_3x3) sorted by index.
    Index 0 → label "1", index 1 → label "2", etc.
    """
    if not os.path.exists(yaml_path):
        print(f"[WARN] Lighthouse YAML not found: {yaml_path!r} - no stations shown.")
        return []
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    stations = []
    for idx, geo in sorted(data.get("geos", {}).items()):
        origin = np.array(geo["origin"], dtype = float)  # origin
        rot    = np.array(geo["rotation"], dtype = float)  # 3×3 row-major rotation matrix
        stations.append((str(idx + 1), origin, rot))
    print(f"[INFO] Loaded {len(stations)} lighthouse station(s).")
    return stations


# ==============================================================================
# TEXT RENDERING HELPERS
# ==============================================================================

def _text_to_texture(font, text, color_rgb):
    """
    Render a single text string to an RGBA OpenGL texture.
    Returns (tex_id, pixel_width, pixel_height).
    """
    col255 = tuple(int(c * 255) for c in color_rgb)
    surf   = font.render(text, True, col255)
    tw, th = surf.get_size()
    canvas = pygame.Surface((tw, th), pygame.SRCALPHA)
    canvas.fill((0, 0, 0, 0))
    canvas.blit(surf, (0, 0))
    raw = pygame.image.tostring(canvas, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    return tex, tw, th


def _begin_2d(win_w, win_h):
    """Switch to 2-D orthographic projection, disable depth/lighting."""
    glDisable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(0, win_w, win_h, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()


def _end_2d():
    """Restore 3-D projection and state after a 2-D draw pass."""
    glDisable(GL_TEXTURE_2D)
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)


def _draw_quad_tex(tex, x, y, tw, th):
    """Blit a 2-D textured quad (top-left corner at x, y)."""
    glColor4f(1, 1, 1, 1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glBegin(GL_QUADS)
    glTexCoord2f(0, 1);  glVertex2f(x,      y)
    glTexCoord2f(1, 1);  glVertex2f(x + tw, y)
    glTexCoord2f(1, 0);  glVertex2f(x + tw, y + th)
    glTexCoord2f(0, 0);  glVertex2f(x,      y + th)
    glEnd()


def _draw_filled_rect(x, y, w, h, r, g, b, a):
    """Draw a solid (possibly transparent) rectangle in current 2-D mode."""
    glDisable(GL_TEXTURE_2D)
    glColor4f(r, g, b, a)
    glBegin(GL_QUADS)
    glVertex2f(x,     y)
    glVertex2f(x + w, y)
    glVertex2f(x + w, y + h)
    glVertex2f(x,     y + h)
    glEnd()
    glEnable(GL_TEXTURE_2D)


def draw_hud_scrollable(lines, font, win_w, win_h, scroll_offset):
    """
    Render a list of HUD lines on the upper-left corner with scrolling support.

    Line entries:
        (text_str, rgb_float_tuple) : normal text line
        (None, None)                : thin horizontal separator

    Returns the total item count (for clamping the scroll offset externally).
    """
    if not lines:
        return 0

    PAD    = 10
    LINE_H = font.get_height() + 4
    SEP_H  = 10   # pixels occupied by a separator entry
    avail  = win_h - 2 * PAD

    # skip the first scroll_offset items
    visible = lines[scroll_offset:]

    # pre-measure the maximum text width among all lines (not just visible ones)
    tw_max = 160
    for text, _col in lines:
        if text is not None:
            w = font.size(text)[0]
            if w > tw_max:
                tw_max = w
    surf_w = tw_max + 12

    surf = pygame.Surface((surf_w, avail), pygame.SRCALPHA)
    surf.fill((0, 0, 0, 0))

    y = 0
    for text, col in visible:
        if y >= avail:
            break
        if text is None:
            # separator: thin dark horizontal line
            pygame.draw.line(surf, (40, 40, 40, 200),
                             (0, y + SEP_H // 2), (surf_w - 4, y + SEP_H // 2), 1)
            y += SEP_H
        else:
            col255 = tuple(int(c * 255) for c in col)
            t = font.render(text, True, col255)
            surf.blit(t, (0, y))
            y += LINE_H

    # scroll indicator
    can_up   = scroll_offset > 0
    can_down = scroll_offset + (avail // LINE_H) + 2 < len(lines)
    if can_up or can_down:
        ind_str = ("▲ " if can_up else "  ") + "PgUp/PgDn" + (" ▼" if can_down else "  ")
        ind = font.render(ind_str, True, (50, 50, 50))
        surf.blit(ind, (0, avail - font.get_height() - 2))

    sw, sh = surf.get_size()
    raw = pygame.image.tostring(surf, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sw, sh, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    _begin_2d(win_w, win_h)
    _draw_filled_rect(PAD - 4, PAD - 4, sw + 8, sh + 8, 0.9, 0.9, 0.9, 0.8)
    _draw_quad_tex(tex, PAD, PAD, sw, sh)
    glDeleteTextures(1, [tex])
    _end_2d()

    return len(lines)


def draw_info_menu(lines, font, win_w, win_h):
    """
    Render the key bindings info panel on the right side of the screen.
    """
    if not lines:
        return
    surfaces = [font.render(text, True, tuple(int(c * 255) for c in col))
                for text, col in lines]
    panel_w = max(s.get_width() for s in surfaces) + 20
    lh      = surfaces[0].get_height() + 4
    panel_h = lh * len(surfaces) + 8

    x = win_w - panel_w - 10
    y = 10

    _begin_2d(win_w, win_h)

    # semi-transparent background panel
    _draw_filled_rect(x - 4, y - 4, panel_w + 8, panel_h + 8, 0.9, 0.9, 0.9, 0.8)

    # text lines
    for i, (surf, (_, col)) in enumerate(zip(surfaces, lines)):
        if surf.get_width() == 0:
            continue
        tw, th = surf.get_size()
        canvas = pygame.Surface((tw, th), pygame.SRCALPHA)
        canvas.fill((0, 0, 0, 0))
        canvas.blit(surf, (0, 0))
        raw = pygame.image.tostring(canvas, "RGBA", True)
        tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tw, th, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        _draw_quad_tex(tex, x + 4, y + i * lh + 4, tw, th)
        glDeleteTextures(1, [tex])

    _end_2d()


def draw_scene_labels(labels, font, win_w, win_h):
    """
    Project 3-D world positions to screen and render text labels using scene_font.
    Must be called after 3-D scene draw, while the 3-D modelview is still active.

    Parameters:
        labels  : list of (world_pos_3d, text_str, rgb_float_tuple)
        font    : pygame font object
        win_w   : window width in pixels
        win_h   : window height in pixels
    """
    if not labels:
        return

    # capture matrices while 3-D state is intact
    modelview  = glGetDoublev(GL_MODELVIEW_MATRIX)
    projection = glGetDoublev(GL_PROJECTION_MATRIX)
    viewport   = glGetIntegerv(GL_VIEWPORT)

    _begin_2d(win_w, win_h)

    for world_pos, text, color in labels:
        try:
            sx, sy, sz = gluProject(world_pos[0], world_pos[1], world_pos[2],
                                     modelview, projection, viewport)
        except Exception:
            continue
        # sz in [0, 1]: 0 = near plane, 1 = far plane; skip behind camera
        if sz < 0.0 or sz > 1.0:
            continue
        screen_x = sx
        screen_y = win_h - sy  # OpenGL Y goes up, pygame Y goes down

        tex, tw, th = _text_to_texture(font, text, color)
        # centre the label horizontally over the 3-D point
        _draw_quad_tex(tex, screen_x - tw / 2, screen_y - th / 2, tw, th)
        glDeleteTextures(1, [tex])

    _end_2d()


# ==============================================================================
# RESIZE HELPER
# ==============================================================================

def _on_resize(w, h):
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, w / h, 0.005, 200.)
    glMatrixMode(GL_MODELVIEW)


# ==============================================================================
# FLEET HELPERS
# ==============================================================================

def _default_pos(drone_num):
    """
    Compute the default spawn position from the 1-indexed drone number.
    Groups of 4 drones are placed on concentric rings at radii 1, 2, 3, ...
    within each ring the four corners are: (-r, +r), (+r, +r), (+r, -r), (-r, -r).
    This extends infinitely so no lookup table cap exists.
    """
    group  = (drone_num - 1) // 4          # ring index (0, 1, 2, ...)
    idx    = (drone_num - 1) % 4           # position within the ring (0-3)
    r      = float(group + 1)              # ring radius in metres
    signs  = [(-1., 1.), (1., 1.), (1., -1.), (-1., -1.)]
    sx, sy = signs[idx]
    return (sx * r, sy * r, 0.)


def _save_drones_config(drones):
    """
    Save the current drone fleet to config/crazyflies_config.yaml as a numbered entry.
    The file contains numbered configurations (1, 2, 3, ...). Each save appends
    a new numbered entry with the current fleet's URIs.
    """
    import os
    os.makedirs("config", exist_ok=True)

    # read existing configs
    configs = {}
    if os.path.exists(DRONES_YAML):
        with open(DRONES_YAML) as f:
            data = yaml.safe_load(f) or {}
            if isinstance(data, dict) and "configs" in data:
                configs = data["configs"]

    # find next config number
    next_num = max(map(int, configs.keys()), default=0) + 1 if configs else 1

    # build new config entry
    uris = {num: entry.uri for num, entry in sorted(drones.items())}
    configs[str(next_num)] = {"drones": uris}

    # write back
    with open(DRONES_YAML, "w") as f:
        yaml.dump({"configs": configs}, f, default_flow_style=False, sort_keys=False)

    print(f"[CONFIG] Saved {len(uris)} drone(s) as config #{next_num} to {DRONES_YAML}")


def _load_drones_config(config_num, drones):
    """
    Load a numbered drone configuration from config/crazyflies_config.yaml.
    Clears the current fleet and spawns drones with URIs from the specified config.
    """
    import os
    if not os.path.exists(DRONES_YAML):
        print(f"[WARN] Config file not found: {DRONES_YAML}")
        return

    with open(DRONES_YAML) as f:
        data = yaml.safe_load(f) or {}

    configs = data.get("configs", {})
    cfg = configs.get(str(config_num))
    if cfg is None:
        print(f"[WARN] Config #{config_num} not found in {DRONES_YAML}")
        return

    drone_uris = cfg.get("drones", {})
    if not drone_uris:
        print(f"[WARN] Config #{config_num} has no drones.")
        return

    # clear current fleet
    for entry in list(drones.values()):
        entry.stop_connection()
    drones.clear()

    # add drones from config
    for num_str, uri in drone_uris.items():
        num = int(num_str)
        pos = _default_pos(num)
        drones[num] = DroneEntry(num, uri, pos)
        print(f"[CONFIG] Loaded CF {num} → {uri}")

    print(f"[CONFIG] Loaded config #{config_num} with {len(drone_uris)} drone(s).")


def _build_uri(radio_id, channel, datarate, address):
    return f"radio://{radio_id}/{channel}/{datarate}/{address}"


def _make_add_modal():
    return ModalDialog(
        title  = "Add New Drone",
        fields = [
            TextField("Radio ID",  "0",            TextField.DIGITS),
            TextField("Channel",   "80",           TextField.DIGITS),
            TextField("Datarate",  "2M",           None, "250K / 1M / 2M"),
            TextField("Address",   "E7E7E7E7E7",   TextField.HEX),
        ],
    )


def _make_delete_modal(drone_nums):
    nums_str = ", ".join(str(n) for n in sorted(drone_nums)) or "none"
    return ModalDialog(
        title  = "Delete Drone",
        fields = [
            TextField("Drone number", "", TextField.DIGITS, f"current: {nums_str}"),
        ],
    )


def _make_connect_modal(drone_nums):
    nums_str = ", ".join(str(n) for n in sorted(drone_nums)) or "none"
    return ModalDialog(
        title  = "Connect to Drone",
        fields = [
            TextField("Drone number  (0 = all)", "0", TextField.DIGITS,
                      f"available: {nums_str}"),
        ],
    )


def _make_load_modal():
    """Modal to ask which config number to load from crazyflies_config.yaml."""
    return ModalDialog(
        title  = "Load Drone Config",
        fields = [
            TextField("Config number", "1", TextField.DIGITS, "1-indexed"),
        ],
    )


def _make_param_modal(drone_nums):
    """Modal to change parameters for a specific drone."""
    nums_str = ", ".join(str(n) for n in sorted(drone_nums)) or "none"
    return ModalDialog(
        title  = "Change Drone Parameters",
        fields = [
            TextField("Drone number", "", TextField.DIGITS, f"available: {nums_str}"),
            TextField("LED value (0-255)", "", TextField.DIGITS, "led.bitmask"),
            TextField("Controller", "", TextField.DIGITS, "stabilizer.controller"),
            TextField("Estimator", "", TextField.DIGITS, "stabilizer.estimator"),
        ],
    )


def _handle_modal_confirm(modal, drones):
    """
    Execute the action described by the confirmed ModalDialog.
    """
    if modal.title == "Add New Drone":
        radio_id = modal.fields[0].value.strip() or "0"
        channel  = modal.fields[1].value.strip() or "80"
        datarate = modal.fields[2].value.strip() or "2M"
        address  = modal.fields[3].value.strip() or "E7E7E7E7E7"
        uri      = _build_uri(radio_id, channel, datarate, address)
        new_num  = max(drones.keys(), default = 0) + 1
        pos      = _default_pos(new_num)
        drones[new_num] = DroneEntry(new_num, uri, pos)
        print(f"[INFO] Added CF {new_num} → {uri}  (pos {pos})")
        print(f"[INFO] Use Ctrl+R to connect.")

    elif modal.title == "Delete Drone":
        raw = modal.fields[0].value.strip()
        if not raw.isdigit():
            print("[WARN] Invalid drone number for deletion.")
            return
        n = int(raw)
        if n in drones:
            drones[n].stop_connection()
            del drones[n]
            print(f"[INFO] Deleted CF {n}.")
        else:
            print(f"[WARN] Drone {n} not found in fleet.")

    elif modal.title == "Connect to Drone":
        raw = modal.fields[0].value.strip()
        if not raw.isdigit():
            print("[WARN] Invalid drone number for connection.")
            return
        n = int(raw)
        if n == 0:
            for entry in drones.values():
                entry.start_connection()
        elif n in drones:
            drones[n].start_connection()
        else:
            print(f"[WARN] Drone {n} not found in fleet.")

    elif modal.title == "Load Drone Config":
        raw = modal.fields[0].value.strip()
        if not raw.isdigit():
            print("[WARN] Invalid config number.")
            return
        config_num = int(raw)
        _load_drones_config(config_num, drones)

    elif modal.title == "Change Drone Parameters":
        drone_num_str = modal.fields[0].value.strip()
        led_str       = modal.fields[1].value.strip()
        ctrl_str      = modal.fields[2].value.strip()
        est_str       = modal.fields[3].value.strip()

        if not drone_num_str.isdigit():
            print("[WARN] Invalid drone number for parameter change.")
            return
        n = int(drone_num_str)
        if n not in drones:
            print(f"[WARN] Drone {n} not found in fleet.")
            return

        entry = drones[n]
        if not entry.state.connected or entry.thread is None:
            print(f"[WARN] CF {n} not connected - cannot change parameters.")
            return

        cf = entry.thread._cf
        if cf is None:
            print(f"[WARN] CF {n} connection handle unavailable.")
            return

        # LED value
        if led_str:
            try:
                led_val = int(led_str)
                if 0 <= led_val <= 255:
                    cf.param.set_value("led.bitmask", str(led_val))
                    print(f"[PARAM] CF {n}: led.bitmask = {led_val}")
                else:
                    print(f"[WARN] LED value must be 0-255.")
            except ValueError:
                print(f"[WARN] Invalid LED value: {led_str}")

        # Controller
        if ctrl_str:
            try:
                ctrl_val = int(ctrl_str)
                cf.param.set_value("stabilizer.controller", str(ctrl_val))
                # update local state
                est_id, _ = entry.state.get_control_config()
                entry.state.set_control_config(est_id, ctrl_val)
                print(f"[PARAM] CF {n}: stabilizer.controller = {ctrl_val}")
            except ValueError:
                print(f"[WARN] Invalid controller value: {ctrl_str}")

        # Estimator
        if est_str:
            try:
                est_val = int(est_str)
                cf.param.set_value("stabilizer.estimator", str(est_val))
                # update local state
                _, ctrl_id = entry.state.get_control_config()
                entry.state.set_control_config(est_val, ctrl_id)
                print(f"[PARAM] CF {n}: stabilizer.estimator = {est_val}")
            except ValueError:
                print(f"[WARN] Invalid estimator value: {est_str}")


def draw_controller_panel(ctrl_strs, ctrl_focused, font, win_w, win_h, cursor_on):
    """
    Render the flight controller panel in the bottom-right corner.

    Parameters:
        ctrl_strs   : list of 3 strings [drone_num, dx, dq]
        ctrl_focused: index (0-2) of the currently focused field
        font        : pygame font
        win_w, win_h: window size
        cursor_on   : bool - whether the text cursor blink phase is on
    """
    PAD    = 10
    LINE_H = font.get_height() + 6

    LABELS   = ["Drone (0=all)     ", "Linear step (m)   ", "Angular step (deg)"]
    FIELD_W  = 90

    ROWS = [
        ("FLIGHT CONTROLLER  [Ctrl+C]",      None),
        ("─" * 34,                           None),
        None,   # Drone field
        None,   # dx field
        None,   # dq field
        ("─" * 34,                           None),
        ("[F] Takeoff      [L] Land",        (0.10, 0.50, 0.10)),
        ("[W] Up           [S] Down",        (0.10, 0.50, 0.10)),
        ("[A] Yaw CCW      [D] Yaw CW",      (0.10, 0.50, 0.10)),
        ("[↑] Forward      [↓] Backward",    (0.10, 0.50, 0.10)),
        ("[←] Slide Left   [→] Slide Right", (0.10, 0.50, 0.10)),
        ("Tab: cycle fields",                (0.40, 0.40, 0.40)),
    ]

    # measure panel width
    tmp_surf = pygame.Surface((1, 1))
    max_w = max(font.size(r[0] if r else LABELS[0] + "  " + ctrl_strs[0])[0]
                for r in ROWS if r is not None)
    panel_w  = max(max_w + 24, FIELD_W + font.size(LABELS[0])[0] + 32)
    panel_h  = LINE_H * len(ROWS) + 2 * PAD

    surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    surf.fill((240, 240, 236, 210))
    pygame.draw.rect(surf, (80, 80, 80), (0, 0, panel_w, panel_h), 2)

    y = PAD
    field_idx = 0
    for row in ROWS:
        if row is None:
            # editable field row
            label = LABELS[field_idx]
            val   = ctrl_strs[field_idx]
            focused = (field_idx == ctrl_focused)
            lbl_surf = font.render(label + " : ", True, (30, 30, 30))
            surf.blit(lbl_surf, (PAD, y + 1))
            box_x = PAD + lbl_surf.get_width()
            box   = pygame.Rect(box_x, y, FIELD_W, LINE_H - 2)
            pygame.draw.rect(surf, (200, 218, 255) if focused else (255, 255, 255), box)
            pygame.draw.rect(surf, (60, 60, 60), box, 1)
            display = val + ("|" if focused and cursor_on else "")
            surf.blit(font.render(display, True, (10, 10, 10)), (box_x + 4, y + 2))
            field_idx += 1
        else:
            text, col = row
            col255 = tuple(int(c * 255) for c in col) if col else (20, 20, 80)
            surf.blit(font.render(text, True, col255), (PAD, y + 1))
        y += LINE_H

    # blit to screen as OpenGL texture (bottom-right corner)
    sw, sh = surf.get_size()
    raw = pygame.image.tostring(surf, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sw, sh, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    ox = win_w - sw - 10
    oy = win_h - sh - 10

    _begin_2d(win_w, win_h)
    _draw_quad_tex(tex, ox, oy, sw, sh)
    glDeleteTextures(1, [tex])
    _end_2d()


def _ctrl_parse(ctrl_strs):
    """
    Parse controller panel field strings.
    Returns (drone_num: int, dx: float, dq: float).
    """
    try:    dn = int(ctrl_strs[0])
    except: dn = 0
    try:    dx = float(ctrl_strs[1])
    except: dx = 0.5
    try:    dq = float(ctrl_strs[2])
    except: dq = 15.0
    return dn, dx, dq


def _send_flight_cmd(cmd, ctrl_strs, drones):
    """
    Dispatch a flight command to the selected drone(s).

    Commands:
        "takeoff"                : rise to dx height
        "land"                   : land
        "up"   / "down"          : translate ±dx in world Z
        "forward" / "backward"   : translate ±dx in body-frame forward (world XY)
        "slide_left"/"slide_right: translate ±dx in body-frame lateral (world XY)
        "yaw_ccw" / "yaw_cw"     : rotate ±dq degrees around Z

    Forward/backward and slide movements are expressed in body frame:
    the world-frame deltas are computed from the drone's current yaw.
    """
    dn, dx, dq = _ctrl_parse(ctrl_strs)
    targets = (list(drones.values()) if dn == 0
               else ([drones[dn]] if dn in drones else []))

    for entry in targets:
        t = entry.thread
        if t is None or not entry.state.connected:
            print(f"[CTRL] CF {entry.num} not connected - command '{cmd}' skipped.")
            continue

        if cmd == "takeoff":
            t.takeoff(height = dx)

        elif cmd == "land":
            t.land()

        elif cmd == "up":
            t.go_to(dz = +dx)

        elif cmd == "down":
            t.go_to(dz = -dx)

        elif cmd in ("forward", "backward", "slide_left", "slide_right"):
            # resolve body-frame movement into world frame using current yaw
            _, _, _, _, yaw, _, _ = entry.state.get()
            yr = math.radians(yaw)
            # body +x = forward, body +y = left (ENU convention)
            fwd = np.array([ math.cos(yr),  math.sin(yr)])
            lft = np.array([-math.sin(yr),  math.cos(yr)])
            if   cmd == "forward":     vec = fwd * dx
            elif cmd == "backward":    vec = fwd * (-dx)
            elif cmd == "slide_left":  vec = lft * dx
            elif cmd == "slide_right": vec = lft * (-dx)
            t.go_to(dx = float(vec[0]), dy = float(vec[1]))

        elif cmd == "yaw_cw":
            t.go_to(dyaw_deg = -dq)

        elif cmd == "yaw_ccw":
            t.go_to(dyaw_deg = +dq)


# allowed characters for controller panel field editing
_CTRL_ALLOWED = [
    "0123456789",            # drone number: digits only
    "-0123456789.",          # dx: float
    "-0123456789.",          # dq: float
]


class StreamTee:
    """
    Mirrors stdout/stderr to the real terminal and also stores each printed line
    in a deque for the in-app console panel.
    """
    def __init__(self, stream, line_buffer):
        self._stream = stream
        self._buffer = line_buffer
        self._partial = ""

    def write(self, text):
        self._stream.write(text)

        # Preserve partial lines between writes.
        text = text.replace("\r", "\n")
        parts = (self._partial + text).split("\n")
        self._partial = parts.pop()
        for line in parts:
            self._buffer.append(line)

    def flush(self):
        self._stream.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _wrap_console_line(text, font, max_width):
    """Word-wrap one console line to fit the panel width."""
    if not text:
        return [""]
    words = text.expandtabs(4).split(" ")
    lines = []
    cur = ""

    for word in words:
        candidate = word if not cur else cur + " " + word
        if font.size(candidate)[0] <= max_width:
            cur = candidate
        else:
            if cur:
                lines.append(cur)
            cur = word

            # Hard-break a single long token if needed
            while font.size(cur)[0] > max_width and len(cur) > 1:
                for i in range(1, len(cur) + 1):
                    if font.size(cur[:i])[0] > max_width:
                        lines.append(cur[:i - 1])
                        cur = cur[i - 1:]
                        break

    if cur:
        lines.append(cur)
    return lines or [""]


def draw_console_panel(console_lines, font, win_w, win_h, cursor_on):
    """
    Render a bottom-middle console panel showing captured stdout/stderr lines.
    """
    PAD = 10
    LINE_H = font.get_height() + 5
    BOTTOM_RESERVED = 10

    # Keep it centered and modest in width so it stays visually separate.
    PANEL_W = max(400, min(int(win_w * 0.40), 700))
    inner_w = PANEL_W - 2 * PAD

    # Wrap and keep only the newest visible lines.
    wrapped = []
    for raw in list(console_lines)[-250:]:
        wrapped.extend(_wrap_console_line(raw, font, inner_w))

    # Keep only the newest rows that can actually be shown
    max_rows = max(5, min(10, int(win_h * 0.30) // LINE_H))
    MAX_BUFFER = max_rows
    if len(console_lines) > MAX_BUFFER:
        console_lines.popleft()
    rows = console_lines if console_lines else ["[no console output yet]"]

    PANEL_H = LINE_H * (len(rows) + 1) + 2 * PAD
    x = (win_w - PANEL_W) // 2
    y = win_h - PANEL_H - BOTTOM_RESERVED

    surf = pygame.Surface((PANEL_W, PANEL_H), pygame.SRCALPHA)
    surf.fill((20, 20, 20, 210))
    pygame.draw.rect(surf, (170, 170, 170), (0, 0, PANEL_W, PANEL_H), 2)

    surf.blit(font.render("CONSOLE  [Shift+C]", True, (235, 235, 235)), (PAD, PAD))
    yy = PAD + LINE_H
    for line in rows:
        surf.blit(font.render(line, True, (230, 230, 230)), (PAD, yy))
        yy += LINE_H

    sw, sh = surf.get_size()
    raw = pygame.image.tostring(surf, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sw, sh, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    _begin_2d(win_w, win_h)
    _draw_quad_tex(tex, x, y, sw, sh)
    glDeleteTextures(1, [tex])
    _end_2d()


def _build_hud_lines(drones, lighthouses, camera, show_info_menu, show_ctrl_panel, xbox):
    """
    Build the complete scrollable HUD line list.

    Returns list of:
        (text_str, rgb_float_tuple)  - normal line
        (None, None)                 - thin separator
    """
    info_hint    = "[i] close menu" if show_info_menu else "[i] key bindings"
    ctrl_hint    = "[Ctrl+C] close panel" if show_ctrl_panel else "[Ctrl+C] flight panel"
    lines = []

    # general info (always visible at the top)
    lines.append((f"Ground Grid   : {GROUND_CELL_M:.3f} m/cell  "
                  f"({GROUND_N_CELLS * 2} x {GROUND_N_CELLS * 2} cells)",
                  (0.20, 0.20, 0.20)))
    lines.append((f"Camera Lookat : ({camera.lookat[0]:.3f}, "
                  f"{camera.lookat[1]:.3f}, {camera.lookat[2]:.3f})",
                  (0.20, 0.20, 0.20)))
    lines.append(("PgUp/PgDn: scroll          " + info_hint,
                  (0.30, 0.30, 0.30)))
    
    lines.append((None, None))
    lines.append((f"LightHouse Stations : {len(lighthouses)}",
                  (0.70, 0.10, 0.70)))
    for lh_name, origin, _ in lighthouses:
        lines.append((f"  LH {lh_name} : ({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f})",
                      (0.60, 0.10, 0.60)))

    lines.append((None, None))
    lines.append((ctrl_hint, (0.10, 0.50, 0.10)))

    # Xbox controller status
    if xbox is not None:
        if xbox.is_connected():
            xbox_col  = (0.10, 0.50, 0.50)
            xbox_name = xbox.controller_name()
            xbox_str  = f"Xbox : {xbox_name}  [{xbox.mode.upper()}]  CF {xbox.controlled_num}"
        else:
            xbox_col = (0.40, 0.40, 0.40)
            xbox_str = "Xbox : not connected"
        lines.append((xbox_str, xbox_col))

    if not drones:
        lines.append((None, None))
        lines.append(("  No drones in fleet.  Use Ctrl+A to add one.", (0.50, 0.50, 0.50)))
        return lines

    for num, entry in sorted(drones.items()):
        lines.append((None, None))  # separator before each drone block

        # drone header line
        vis_tag = "" if entry.visible else "  [HIDDEN]"
        lines.append((f"CF {num}{vis_tag} : {entry.uri}", (0.05, 0.05, 0.05)))

        pwb, Rwb, roll, pitch, yaw, vbat, motors_pwm = entry.state.get()
        est_id, ctrl_id = entry.state.get_control_config()

        status_str = "CONNECTED" if entry.state.connected else "DISCONNECTED"
        status_col = (0.1, 0.6, 0.1) if entry.state.connected else (0.9, 0.2, 0.2)
        lines.append((f"  Status       : {status_str}", status_col))

        pos_col = (0.10, 0.10, 0.50)
        lines.append((f"  Position (m) : x={pwb[0]:+.3f}  y={pwb[1]:+.3f}  z={pwb[2]:+.3f}",
                      pos_col))
        lines.append((f"  Euler (deg)  : r={roll:+.2f}  p={pitch:+.2f}  y={yaw:+.2f}",
                      pos_col))

        bat_pct = entry.state.battery_pct()
        bat_col = ((0.1, 0.6, 0.1) if bat_pct > 30
                   else (0.9, 0.5, 0.0) if bat_pct > 15
                   else (0.9, 0.1, 0.1))
        lines.append((f"  Battery      : {bat_pct:.1f} %  ({vbat:.2f} V)", bat_col))

        m = 100.0 * motors_pwm / MAX_PWM
        lines.append((f"  Motors %     : m1={m[0]:.1f}  m2={m[1]:.1f}  m3={m[2]:.1f}  m4={m[3]:.1f}",
                      (0.70, 0.45, 0.10)))

        lines.append((f"  Estimator    : {est_id} ({entry.state.estimator_name()})",
                      (0.35, 0.10, 0.55)))
        lines.append((f"  Controller   : {ctrl_id} ({entry.state.controller_name()})",
                      (0.35, 0.10, 0.55)))

    return lines


# ==============================================================================
# ENTRY POINT
# ==============================================================================

def main():
    # parse CLI
    yaml_path = LIGHTHOUSE_YAML
    for i, a in enumerate(sys.argv):
        if a == "--yaml" and i + 1 < len(sys.argv):
            yaml_path = sys.argv[i + 1]

    # lighthouse data and drone fleet
    lighthouses = load_lighthouses(yaml_path)
    drones      = {}   # dict[int, DroneEntry] - keyed by drone number (1-indexed, gaps allowed)

    # pygame + OpenGL init
    pygame.init()
    pygame.joystick.init()
    display_info = pygame.display.Info()
    WINDOW_W = int(3/4 * display_info.current_w)
    WINDOW_H = int(3/4 * display_info.current_h)
    pygame.display.set_mode((WINDOW_W, WINDOW_H), DOUBLEBUF | OPENGL | RESIZABLE)
    pygame.display.set_caption("Crazyflie 3D Visualizer")

    # fonts
    hud_font     = pygame.font.SysFont(name = "consolas", size = 13, bold = True)
    console_font = pygame.font.SysFont(name = "consolas", size = 11, bold = True)
    control_font = pygame.font.SysFont(name = "lucidaconsole", size = 13, bold = True)
    info_font    = pygame.font.SysFont(name = "couriernew", size = 11, bold = True)
    scene_font   = pygame.font.SysFont(name = "arial", size = 14, bold = True, italic = True)
    modal_font   = pygame.font.SysFont(name = "consolas", size = 14, bold = True)

    # in-app console buffer (captures stdout/stderr and keeps the terminal too)
    console_lines = deque(maxlen = 100)
    sys.stdout = StreamTee(sys.stdout, console_lines)
    sys.stderr = StreamTee(sys.stderr, console_lines)

    # depth & blending
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glShadeModel(GL_SMOOTH)
    glClearColor(0.7, 0.7, 0.7, 1.0)

    # lighting - single diffuse key light (headlight-style: set before camera)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.3,  0.3,  0.3,  1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.85, 0.85, 0.85, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.20, 0.20, 0.20, 1.0])
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    _on_resize(WINDOW_W, WINDOW_H)

    camera        = Camera()
    clock         = pygame.time.Clock()
    prev_mouse_xy = None
    cur_button    = None

    # visibility flags
    show_world_frame = True
    show_drone_body  = True
    show_drone_frame = True
    show_lighthouses = True
    show_hud_text    = True
    show_scene_text  = True
    show_console     = False
    show_info_menu   = False

    # HUD scroll state
    hud_scroll      = 0
    hud_total_lines = 0

    # active modal dialog (None when no dialog is open)
    modal = None

    # flight controller panel state
    show_ctrl_panel = False
    ctrl_focused    = 0                          # 0 = drone, 1 = dx, 2 = dq
    ctrl_strs       = ["0", "0.50", "15.0"]     # editable field values

    # camera tracking state (None = manual, int = track drone N)
    camera_track_num = None

    # Xbox controller (connects automatically if a joystick is detected)
    xbox = XboxController(drones)
    xbox.start()

    print("[INFO] No drones in fleet. Use Ctrl+A to add one, then Ctrl+R to connect.")

    running = True
    while running:
        clock.tick(TARGET_FPS)
        cursor_on = int(time.time() * 2) % 2 == 0   # 0.5 s blink period for modal cursor

        # events
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False

            elif evt.type == VIDEORESIZE:
                WINDOW_W, WINDOW_H = evt.w, evt.h
                pygame.display.set_mode((WINDOW_W, WINDOW_H), DOUBLEBUF | OPENGL | RESIZABLE)
                _on_resize(WINDOW_W, WINDOW_H)

            elif evt.type == KEYDOWN:
                # modal intercepts all keyboard input
                if modal is not None:
                    modal.handle_event(evt)
                    if modal.confirmed:
                        _handle_modal_confirm(modal, drones)
                        modal = None
                    elif modal.cancelled:
                        modal = None
                    continue

                mods = pygame.key.get_mods()

                # controller panel intercepts Tab and editable-character input
                # (flight action keys are still active while panel is open)
                if show_ctrl_panel and not (mods & (KMOD_SHIFT | KMOD_CTRL)):
                    if evt.key == K_TAB:
                        ctrl_focused = (ctrl_focused + 1) % 3
                        continue
                    if evt.key == pygame.K_BACKSPACE:
                        ctrl_strs[ctrl_focused] = ctrl_strs[ctrl_focused][:-1]
                        continue
                    if (evt.unicode and
                            (evt.unicode in _CTRL_ALLOWED[ctrl_focused] or
                             (ctrl_focused > 0 and evt.unicode == '-' and not ctrl_strs[ctrl_focused]))):
                        ctrl_strs[ctrl_focused] += evt.unicode
                        continue

                if evt.key == K_ESCAPE:
                    running = False

                elif evt.key == K_F11:
                    pygame.display.toggle_fullscreen()
                    WINDOW_W, WINDOW_H = pygame.display.get_surface().get_size()
                    _on_resize(WINDOW_W, WINDOW_H)

                elif evt.key == K_i:
                    show_info_menu = not show_info_menu

                # HUD scrolling
                elif evt.key == K_PAGEUP:
                    hud_scroll = max(0, hud_scroll - 5)
                elif evt.key == K_PAGEDOWN:
                    hud_scroll = min(max(0, hud_total_lines - 1), hud_scroll + 5)

                # Shift + letter : visibility toggles
                elif evt.key == K_w and (mods & KMOD_SHIFT):
                    show_world_frame = not show_world_frame
                elif evt.key == K_d and (mods & KMOD_SHIFT):
                    show_drone_body = not show_drone_body
                elif evt.key == K_f and (mods & KMOD_SHIFT):
                    show_drone_frame = not show_drone_frame
                elif evt.key == K_l and (mods & KMOD_SHIFT):
                    show_lighthouses = not show_lighthouses
                elif evt.key == K_h and (mods & KMOD_SHIFT):
                    show_hud_text = not show_hud_text
                elif evt.key == K_c and (mods & KMOD_SHIFT) and not (mods & KMOD_CTRL):
                    show_console = not show_console
                elif evt.key == K_t and (mods & KMOD_SHIFT):
                    show_scene_text = not show_scene_text

                # Shift + numkey : show/hide individual drones (0 = all)
                elif (mods & KMOD_SHIFT) and pygame.K_0 <= evt.key <= pygame.K_9:
                    n = evt.key - pygame.K_0
                    if n == 0:
                        # toggle all: if any are visible, hide all; otherwise show all
                        any_visible = any(e.visible for e in drones.values())
                        for e in drones.values():
                            e.visible = not any_visible
                    elif n in drones:
                        drones[n].visible = not drones[n].visible

                # Ctrl + A : add drone (limit 8)
                elif evt.key == K_a and (mods & KMOD_CTRL):
                    if len(drones) >= 8:
                        print("[INFO] Maximum of 8 drones reached. Delete a drone to add another.")
                    else:
                        modal = _make_add_modal()

                # Ctrl + D : delete drone
                elif evt.key == K_d and (mods & KMOD_CTRL):
                    if drones:
                        modal = _make_delete_modal(set(drones.keys()))
                    else:
                        print("[INFO] No drones in fleet to delete.")

                # Ctrl + R : connect drone(s)
                elif evt.key == K_r and (mods & KMOD_CTRL):
                    if drones:
                        modal = _make_connect_modal(set(drones.keys()))
                    else:
                        print("[INFO] No drones in fleet. Use Ctrl+A to add one first.")

                # Ctrl + S : save drone config
                elif evt.key == K_s and (mods & KMOD_CTRL):
                    if drones:
                        _save_drones_config(drones)
                    else:
                        print("[INFO] No drones to save.")

                # Ctrl + L : load drone config
                elif evt.key == K_l and (mods & KMOD_CTRL):
                    modal = _make_load_modal()

                # Ctrl + P : change drone parameters
                elif evt.key == pygame.K_p and (mods & KMOD_CTRL):
                    if drones:
                        modal = _make_param_modal(set(drones.keys()))
                    else:
                        print("[INFO] No drones in fleet.")

                # Ctrl + C : toggle flight controller panel
                elif evt.key == K_c and (mods & KMOD_CTRL):
                    show_ctrl_panel = not show_ctrl_panel

                # Ctrl + numkey (1-8) : blink LED of drone N
                elif (mods & KMOD_CTRL) and pygame.K_1 <= evt.key <= pygame.K_8:
                    n = evt.key - pygame.K_0
                    if n in drones and drones[n].thread is not None:
                        drones[n].thread.blink_led()
                    else:
                        print(f"[INFO] CF {n} not connected - cannot blink LED.")

                # Alt + 0 : reset camera tracking
                elif evt.key == pygame.K_0 and (mods & KMOD_ALT):
                    camera_track_num = None
                    print("[INFO] Camera tracking disabled.")

                # Alt + numkey (1-8) : track drone N
                elif (mods & KMOD_ALT) and pygame.K_1 <= evt.key <= pygame.K_8:
                    n = evt.key - pygame.K_0
                    if n in drones:
                        camera_track_num = n
                        print(f"[INFO] Camera tracking CF {n}.")
                    else:
                        print(f"[INFO] CF {n} not in fleet.")

                # flight action keys (active when controller panel is open)
                elif show_ctrl_panel and not (mods & (KMOD_SHIFT | KMOD_CTRL)):
                    if evt.key == K_f:
                        _send_flight_cmd("takeoff",     ctrl_strs, drones)
                    elif evt.key == K_l:
                        _send_flight_cmd("land",        ctrl_strs, drones)
                    elif evt.key == K_w:
                        _send_flight_cmd("up",          ctrl_strs, drones)
                    elif evt.key == K_s:
                        _send_flight_cmd("down",        ctrl_strs, drones)
                    elif evt.key == K_a:
                        _send_flight_cmd("yaw_ccw",     ctrl_strs, drones)
                    elif evt.key == K_d:
                        _send_flight_cmd("yaw_cw",      ctrl_strs, drones)
                    elif evt.key == K_UP:
                        _send_flight_cmd("forward",     ctrl_strs, drones)
                    elif evt.key == K_DOWN:
                        _send_flight_cmd("backward",    ctrl_strs, drones)
                    elif evt.key == K_LEFT:
                        _send_flight_cmd("slide_left",  ctrl_strs, drones)
                    elif evt.key == K_RIGHT:
                        _send_flight_cmd("slide_right", ctrl_strs, drones)

            elif evt.type == MOUSEBUTTONDOWN:
                if modal is not None:
                    pass  # ignore mouse clicks while a modal is open
                elif evt.button == 1:
                    prev_mouse_xy = pygame.mouse.get_pos()
                    cur_button = "rotate"
                elif evt.button == 2:
                    camera.reset_view()  # middle-click: reset view
                elif evt.button == 3:
                    prev_mouse_xy = pygame.mouse.get_pos()
                    cur_button = "pan"
                elif evt.button == 4: camera.zoom(+1)  # scroll up   = zoom in
                elif evt.button == 5: camera.zoom(-1)  # scroll down = zoom out

            elif evt.type == MOUSEBUTTONUP:
                cur_button    = None
                prev_mouse_xy = None

            elif evt.type == MOUSEMOTION and prev_mouse_xy and cur_button and modal is None:
                cx, cy   = pygame.mouse.get_pos()
                dx, dy   = cx - prev_mouse_xy[0], cy - prev_mouse_xy[1]
                prev_mouse_xy = (cx, cy)
                if   cur_button == "rotate": camera.rotate(dx, dy)
                elif cur_button == "pan":    camera.pan(dx, dy)

        # render 3D scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # headlight: set before camera so it stays fixed relative to viewer
        glLightfv(GL_LIGHT0, GL_POSITION, [0., 0., 5., 0.])

        # update camera tracking
        if camera_track_num is not None and camera_track_num in drones:
            pwb, _, _, _, _, _, _ = drones[camera_track_num].state.get()
            camera.lookat = pwb.copy()

        camera.apply()
            
        # collect scene labels to project after the 3-D pass
        scene_labels = []

        # 1 - world frame
        if show_world_frame:
            draw_frame(np.zeros(3), np.eye(3), axis_len = 0.5, sphere_r = 0.01)
            if show_scene_text:
                scene_labels.append((np.array([0., 0., 0.60]), "world frame", (0.05, 0.05, 0.05)))

        # 2 - ground grid
        draw_ground(GROUND_N_CELLS, GROUND_CELL_M)

        # 3 - lighthouse stations
        if show_lighthouses:
            for lh_name, origin, rot in lighthouses:
                draw_frame(origin, rot, axis_len = 0.10, sphere_r = 0.01)
                if show_scene_text:
                    scene_labels.append((origin + np.array([0., 0., 0.1]),
                                         f"LH {lh_name}", (0.70, 0.10, 0.70)))

        # 4 - all drones in the fleet
        for num, entry in sorted(drones.items()):
            if not entry.visible:
                continue
            pwb, Rwb, roll, pitch, yaw, vbat, motors_pwm = entry.state.get()
            if show_drone_frame:
                draw_frame(pwb, Rwb, axis_len = DRONE_ARM_L, sphere_r = 0.002)
            if show_drone_body:
                ctrl = motors_pwm if entry.state.connected else None
                draw_quadrotor(pwb, Rwb, DRONE_ARM_L, DRONE_BODY_YAW, control_inputs = ctrl)
            if show_scene_text:
                scene_labels.append((pwb + np.array([0., 0., 0.10]),
                                     f"CF {num}", (0.10, 0.10, 0.50)))

        # project and render scene labels (must happen before _begin_2d)
        if show_scene_text:
            draw_scene_labels(scene_labels, scene_font, WINDOW_W, WINDOW_H)

        # HUD overlay (left side, scrollable)
        if show_hud_text:
            hud_lines       = _build_hud_lines(drones, lighthouses, camera,
                                               show_info_menu, show_ctrl_panel, xbox)
            hud_total_lines = draw_hud_scrollable(hud_lines, hud_font, WINDOW_W, WINDOW_H, hud_scroll)

        # key bindings info menu (right side)
        if show_info_menu:
            draw_info_menu(INFO_MENU_LINES, info_font, WINDOW_W, WINDOW_H)

        # console overlay (bottom-middle)
        if show_console:
            draw_console_panel(console_lines, console_font, WINDOW_W, WINDOW_H, cursor_on)
            
        # flight controller panel (bottom-right corner)
        if show_ctrl_panel:
            draw_controller_panel(ctrl_strs, ctrl_focused, control_font, WINDOW_W, WINDOW_H, cursor_on)

        # modal dialog (centered, with dimmed background)
        if modal is not None:
            draw_modal(modal, modal_font, WINDOW_W, WINDOW_H, cursor_on)

        pygame.display.flip()

    # cleanup
    xbox.stop()
    for entry in drones.values():
        entry.stop_connection()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
