#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 SEW-EURODRIVE, Author: Leon Rafael Schönfeld
# SPDX-License-Identifier: MIT
"""
SEW Multimodal AMR Dataset Viewer
=================================
Interactive viewer for the SEW Multimodal AMR Dataset. The dataset follows the
KITTI layout, so if you have ever used a KITTI viewer you already know this one:
it shows the RGB image on the left and the thermal image on the right, with the
ToF point cloud projected on top and the KITTI / YOLO labels drawn over both.

The ToF point cloud is coloured by **range** on the RGB pane (jet, 1-6 m) and by
**amplitude** on the thermal pane (jet, fixed 0-2900 window), so a single look
tells you both how far each return is and how strong it is. A colour bar on each
pane labels its scale (left = range in metres, right = amplitude); 'c' toggles
them. Ground-truth labels are solid; model **predictions** (if you point the
config at a predictions folder) are dashed.

The same script also opens a plain KITTI download - just point it at a different
config (``-c config_kitti.yaml``). Dataset paths and format flags live in the YAML
config files, so you never edit this script to view your own copy.

Usage
-----
    python SEW_Dataset_viewer.py                          # interactive, config_sew.yaml
    python SEW_Dataset_viewer.py -c config_kitti.yaml     # a plain KITTI download
    python SEW_Dataset_viewer.py --split test             # fills {split} in the paths
    python SEW_Dataset_viewer.py -f 000123                # open at a specific frame id
    python SEW_Dataset_viewer.py --frames df_frames.txt   # only the listed frame ids
    python SEW_Dataset_viewer.py --export                 # batch-render every frame to disk

The ``--frames`` flag takes a plain text file of frame ids (one per line, '#'
comments allowed) and restricts the viewer to just those frames - e.g. the
failure_analysis explorer's downloaded ``df_frames_*.txt`` so you can step
through exactly the frames of interest. Ids match ignoring zero-padding, so
'15992' finds '015992'.

Controls (interactive)
----------------------
    n / -> / space   next frame          b / <-   previous frame
    m / v            skip +100 / -100     g        go to frame id (terminal)
    s                save the current view to the output folder
    h                toggle the on-screen help
    q / ESC          quit

  Overlay keys
    1  ToF point cloud (range on rgb | amplitude on thermal)
    2  3D boxes (KITTI)          3  thermal 2D YOLO boxes
    4  rgb 2D YOLO boxes         5  rgb 2D KITTI box (bbox cols)
    p  predictions - cycle off / 2D / 3D / 2D+3D (dashed; needs a pred_* folder)
    c  colour bars (left = range m | right = amplitude)
    t  sensor sync-spread bar
"""

import os
import sys
import math
import json
import argparse
from dataclasses import dataclass

import numpy as np
import cv2

FONT = cv2.FONT_HERSHEY_SIMPLEX

# Arrow key codes returned by cv2.waitKeyEx (GTK and Qt backends)
ARROW_NEXT = {65363, 65364, 16777236, 16777237}  # right, down
ARROW_PREV = {65361, 65362, 16777234, 16777235}  # left,  up

# --- ToF point-cloud overlay: fixed look (deliberately not user-adjustable) ----
JET = cv2.COLORMAP_JET
TOF_POINT_SIZE = 3        # splat size in px
TOF_OPACITY    = 0.6      # blend weight of the points over the image
TOF_DEPTH_MIN_M = 1.0     # jet range (depth) window on the rgb pane, metres
TOF_DEPTH_MAX_M = 6.0
# Amplitude window on the thermal pane. FIXED (not a per-frame stretch) so a colour
# means the same amplitude on every frame - that is what makes the amplitude colour
# bar meaningful. The Espros ToF amplitude saturates at ~2892 (measured across the
# dataset with --amp-stats), so the window tops out just above that: a red point is
# a fully-saturated return. Re-check with --amp-stats if the sensor/pipeline changes.
TOF_AMP_MIN = 0.0
TOF_AMP_MAX = 2900.0

# Sensor colours + sync timeline half-width (timestamps sync-spread bar).
SENSOR_COL = {"RGB": (255, 170, 60), "THERMAL": (0, 165, 255), "TOF": (80, 255, 80)}
SYNC_WINDOW_MS = 10.0

# Prediction overlay cycle (the 'p' key steps through these in order).
PRED_OFF, PRED_2D, PRED_3D, PRED_BOTH = 0, 1, 2, 3
PRED_LABELS = {PRED_OFF: "off", PRED_2D: "2D", PRED_3D: "3D", PRED_BOTH: "2D+3D"}


# =====================================================================
# Configuration
# =====================================================================
@dataclass
class Config:
    name: str
    split: str
    base_dir: str
    label_folder: str
    calib_folder: str
    image_rgb_folder: str
    image_thermal_folder: str
    bin_folder: str
    pcd_folder: str
    use_pcd: bool
    yolo_thermal_folder: str
    yolo_rgb_folder: str
    pred_yolo_rgb_folder: str
    pred_yolo_thermal_folder: str
    pred_kitti_folder: str
    timestamps_folder: str
    output_folder: str
    image_ext: str
    show_tof_pcl: bool
    show_3d_label: bool
    show_thermal_label: bool
    show_rgb_label: bool
    show_rgb_kitti_2d: bool
    show_predictions: bool
    show_colorbars: bool
    display_max_width: int
    display_max_height: int


def _read_yaml(path):
    """Load a flat YAML config. Uses PyYAML if available, otherwise a tiny
    built-in parser so the viewer has no hard dependency on PyYAML."""
    try:
        import yaml
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except ImportError:
        data = {}
        with open(path, "r") as f:
            for raw in f:
                line = raw.split("#", 1)[0].rstrip()
                if not line.strip() or ":" not in line:
                    continue
                key, val = line.split(":", 1)
                data[key.strip()] = val.strip()
        return data


def _as_str(val, default=""):
    if val is None:
        return default
    return str(val).strip().strip('"').strip("'")


def _as_bool(val, default=False):
    if isinstance(val, bool):
        return val
    if val is None:
        return default
    return _as_str(val).lower() in ("true", "1", "yes", "on")


def _as_int(val, default):
    try:
        return int(val)
    except (TypeError, ValueError):
        return default


def _resolve(base, p):
    """Absolute paths stay as-is; relative ones are joined onto base_dir.
    Empty strings mean 'this data type is not available'."""
    p = _as_str(p)
    if not p:
        return ""
    p = os.path.expanduser(p)
    return p if os.path.isabs(p) else os.path.join(base, p)


def load_config(path, split_override=None):
    raw = _read_yaml(path)
    base = os.path.expanduser(_as_str(raw.get("base_dir")))

    # Dataset split (e.g. train / test / val). A `--split` CLI value wins over the
    # config's `split:`. Any folder path may contain a `{split}` token, which is
    # replaced here - so pointing the viewer at the test split is a one-word change.
    split = _as_str(split_override) if split_override else _as_str(raw.get("split"))

    def _split_path(val):
        s = _as_str(val)
        if "{split}" in s:
            if not split:
                print(f"[!] path '{s}' uses {{split}} but no split is set - add 'split:' to "
                      f"the config or pass --split", file=sys.stderr)
            s = s.replace("{split}", split)
        return s

    ext = _as_str(raw.get("image_ext"), ".jpg")
    if ext and not ext.startswith("."):
        ext = "." + ext

    def R(key):  # resolve a {split}-aware folder path onto base_dir
        return _resolve(base, _split_path(raw.get(key)))

    config_dir = os.path.dirname(os.path.abspath(path))

    def resolve_output(val):
        return _resolve(config_dir, val)

    # Timestamps default to <split>/04_timestamps (next to the calib folder).
    calib = R("calib_folder")
    timestamps = R("timestamps_folder")
    if not timestamps and calib:
        timestamps = os.path.join(os.path.dirname(calib), "04_timestamps")

    return Config(
        name=os.path.basename(path),
        split=split,
        base_dir=base,
        label_folder=R("label_folder"),
        calib_folder=calib,
        image_rgb_folder=R("image_rgb_folder"),
        image_thermal_folder=R("image_thermal_folder"),
        bin_folder=R("bin_folder"),
        pcd_folder=R("pcd_folder"),
        use_pcd=_as_bool(raw.get("use_pcd"), False),
        yolo_thermal_folder=R("yolo_thermal_folder"),
        yolo_rgb_folder=R("yolo_rgb_folder"),
        pred_yolo_rgb_folder=R("pred_yolo_rgb_folder"),
        pred_yolo_thermal_folder=R("pred_yolo_thermal_folder"),
        pred_kitti_folder=R("pred_kitti_folder"),
        timestamps_folder=timestamps,
        output_folder=resolve_output(raw.get("output_folder")) or os.path.join(config_dir, "output"),
        image_ext=ext,
        show_tof_pcl=_as_bool(raw.get("show_tof_pcl"), True),
        show_3d_label=_as_bool(raw.get("show_3d_label"), True),
        show_thermal_label=_as_bool(raw.get("show_thermal_label"), False),
        show_rgb_label=_as_bool(raw.get("show_rgb_label"), False),
        show_rgb_kitti_2d=_as_bool(raw.get("show_rgb_kitti_2d"), False),
        show_predictions=_as_bool(raw.get("show_predictions"), False),
        show_colorbars=_as_bool(raw.get("show_colorbars"), False),
        display_max_width=_as_int(raw.get("display_max_width"), 1800),
        display_max_height=_as_int(raw.get("display_max_height"), 1000),
    )


# =====================================================================
# KITTI labels + calibration (camera-2 rectified coords)
# =====================================================================
def read_labels(label_path):
    """KITTI object labels: 2D bbox (image_2 px) + 3D box (cam coords)."""
    boxes = []
    if not label_path or not os.path.exists(label_path):
        return boxes
    with open(label_path, "r") as f:
        for line in f.readlines():
            parts = line.strip().split()
            if len(parts) < 15:
                continue
            bl, bt, br, bb = map(float, parts[4:8])
            h, w, l = map(float, parts[8:11])
            x, y, z = map(float, parts[11:14])
            boxes.append({
                "cls": parts[0],
                "bbox": (bl, bt, br, bb),
                "h": h, "w": w, "l": l,
                "x": x, "y": y, "z": z,
                "ry": float(parts[14]),
            })
    return boxes


def read_calib(calib_path):
    """Read P2, P3, Tr_velo_to_cam, R0_rect (KITTI-compatible).

    The RGB and thermal cameras share one reference frame, so a single
    Tr_velo_to_cam brings the ToF/laser cloud into that frame and P2 / P3 project
    it into the RGB / thermal image planes respectively - P3 carries the thermal
    camera's own intrinsics plus the inter-camera baseline in its 4th column.
    (Older calib files also shipped a redundant Tr_velo_to_thermal; it is no
    longer written and is ignored if present.)"""
    P2 = P3 = Tr = R0 = None
    if os.path.exists(calib_path):
        with open(calib_path, "r") as f:
            for line in f:
                if line.startswith("P2:"):
                    P2 = np.array(list(map(float, line.split(":")[1].split()))).reshape(3, 4)
                elif line.startswith("P3:"):
                    P3 = np.array(list(map(float, line.split(":")[1].split()))).reshape(3, 4)
                elif line.startswith("Tr_velo_to_cam:"):
                    Tr = np.array(list(map(float, line.split(":")[1].split()))).reshape(3, 4)
                elif line.startswith("R0_rect:"):
                    R0 = np.array(list(map(float, line.split(":")[1].split()))).reshape(3, 3)

    # Sensible fallbacks so the viewer degrades gracefully on partial calib:
    if P3 is None and P2 is not None:
        P3 = P2.copy()                      # no thermal projection -> reuse rgb
    if R0 is None:
        R0 = np.eye(3, dtype=float)
    return P2, P3, Tr, R0


def make_manual_R0():
    """Manual R0 override (tilt in degrees, for debugging / alignment). Both
    datasets keep this at identity; the KITTI labels are already rectified."""
    tilt_down_deg  = 0.0   # pitch (around X)
    tilt_right_deg = 0.0   # yaw   (around Y)
    theta = np.deg2rad(tilt_down_deg)
    phi   = np.deg2rad(tilt_right_deg)
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(theta), -math.sin(theta)],
                   [0, math.sin(theta),  math.cos(theta)]])
    Ry = np.array([[math.cos(phi), 0, math.sin(phi)],
                   [0, 1, 0],
                   [-math.sin(phi), 0, math.cos(phi)]])
    return Ry @ Rx


def box_corners_in_cam(box):
    """3D box corners in camera-2 rectified coords (KITTI)."""
    h, w, l = box["h"], box["w"], box["l"]
    x, y, z = box["x"], box["y"], box["z"]
    ry      = box["ry"]

    x_c = [ l/2,  l/2, -l/2, -l/2,  l/2,  l/2, -l/2, -l/2]
    y_c = [ h/2,  h/2,  h/2,  h/2, -h/2, -h/2, -h/2, -h/2]
    z_c = [ w/2, -w/2, -w/2,  w/2,  w/2, -w/2, -w/2,  w/2]
    corners = np.vstack((x_c, y_c, z_c))

    R_y = np.array([[ np.cos(ry), 0, np.sin(ry)],
                    [ 0,          1, 0         ],
                    [-np.sin(ry), 0, np.cos(ry)]])
    corners = R_y @ corners
    corners = corners + np.array([x, y - h/2, z]).reshape(3, 1)
    return corners     # KITTI labels are already rectified; no R0 here


def project_corners(P, corners):
    corners_h = np.vstack((corners, np.ones((1, corners.shape[1]))))
    proj = P @ corners_h
    proj = proj / proj[2, :]
    return proj[:2, :].T


# =====================================================================
# Drawing primitives: solid + dashed lines / rectangles
# =====================================================================
def _line(img, p1, p2, color, thickness=2, dashed=False, dash=7):
    """A straight (solid) or dashed line between two integer points."""
    p1 = (int(p1[0]), int(p1[1]))
    p2 = (int(p2[0]), int(p2[1]))
    if not dashed:
        cv2.line(img, p1, p2, color, thickness, cv2.LINE_AA)
        return
    length = int(math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
    if length == 0:
        return
    for i in range(0, length, dash * 2):
        t0, t1 = i / length, min(i + dash, length) / length
        sp = (int(p1[0] + (p2[0] - p1[0]) * t0), int(p1[1] + (p2[1] - p1[1]) * t0))
        ep = (int(p1[0] + (p2[0] - p1[0]) * t1), int(p1[1] + (p2[1] - p1[1]) * t1))
        cv2.line(img, sp, ep, color, thickness, cv2.LINE_AA)


def _rect(img, p1, p2, color, thickness=2, dashed=False):
    if not dashed:
        cv2.rectangle(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, thickness)
        return
    x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
    _line(img, (x1, y1), (x2, y1), color, thickness, True)
    _line(img, (x2, y1), (x2, y2), color, thickness, True)
    _line(img, (x2, y2), (x1, y2), color, thickness, True)
    _line(img, (x1, y2), (x1, y1), color, thickness, True)


def draw_3d_box(img, pts2d, corners_cam, cls, x_offset, region_min, region_max,
                color=(255, 255, 255), dashed=False, tag_prefix=""):
    """Draw one 3D box wireframe clipped to the pane, plus class + closest z.
    GT boxes are solid white; predictions pass dashed=True + a class colour."""
    pts = pts2d.astype(int)
    pts[:, 0] += x_offset

    img_h = img.shape[0]
    xs, ys = pts[:, 0], pts[:, 1]
    # Only skip when the box is *entirely* outside this pane.
    if (np.all(xs < region_min) or np.all(xs >= region_max) or
            np.all(ys < 0) or np.all(ys >= img_h)):
        return

    lines = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    clip_rect = (region_min, 0, region_max - region_min, img_h)
    for i, j in lines:
        p1 = (int(pts[i, 0]), int(pts[i, 1]))
        p2 = (int(pts[j, 0]), int(pts[j, 1]))
        inside, cp1, cp2 = cv2.clipLine(clip_rect, p1, p2)
        if inside:
            _line(img, cp1, cp2, color, 2, dashed=dashed)

    min_z    = float(np.min(corners_cam[2, :]))
    y_max    = int(np.clip(np.max(pts[:, 1]), 0, img_h - 1))
    x_center = int(np.mean(pts[:, 0]))

    scale, thick, line_h = 0.5, 1, 15
    texts = (tag_prefix + cls, f"{min_z:.1f}m")
    base_y = y_max + line_h
    last_y = base_y + line_h * (len(texts) - 1)
    if last_y > img_h - 1:
        base_y -= last_y - (img_h - 1)
    for k, text in enumerate(texts):
        (tw, _), _ = cv2.getTextSize(text, FONT, scale, thick)
        tx = int(np.clip(x_center, region_min, max(region_min, region_max - tw)))
        ty = int(np.clip(base_y + line_h * k, line_h, img_h - 1))
        cv2.putText(img, text, (tx, ty), FONT, scale, color, thick, cv2.LINE_AA)


# =====================================================================
# ToF point cloud -> coloured image overlay
# =====================================================================
def dilate(img, size):
    if size <= 1:
        return img
    return cv2.dilate(img, np.ones((size, size), np.uint8))


def load_bin(path):
    """(xyz Nx3, intensity N) from a KITTI-style float32 x,y,z,intensity .bin."""
    if not path or not os.path.exists(path):
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32)
    raw = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
    return raw[:, :3], raw[:, 3]


def load_pcd(path):
    """(xyz Nx3, intensity N) from a simple XYZ[I] text .pcd. Missing intensity
    falls back to 1.0 so the thermal-pane colouring still has something to show."""
    pts, inten = [], []
    if not path or not os.path.exists(path):
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.float32)
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if (not line or line.startswith("#") or
                    line.startswith("VERSION") or "FIELDS" in line):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                i = float(parts[3]) if len(parts) >= 4 else 1.0
            except ValueError:
                continue
            pts.append([x, y, z])
            inten.append(i)
    return np.array(pts, dtype=np.float32), np.array(inten, dtype=np.float32)


def load_tof_cloud(cfg, name):
    """The ToF point cloud (xyz, intensity). Reads the .bin by default; set
    `use_pcd: true` (with a pcd_folder) in the config to read the .pcd instead -
    the two carry the same information."""
    if cfg.use_pcd and cfg.pcd_folder:
        return load_pcd(os.path.join(cfg.pcd_folder, name + ".pcd"))
    bin_path = os.path.join(cfg.bin_folder, name + ".bin") if cfg.bin_folder else ""
    return load_bin(bin_path)


def project_to_image(pts, intensity, P, Tr, R0, W, H):
    """Project ToF points into a camera. Returns per-visible-point arrays
    (u, v, rng, amp): integer pixel, Euclidean range (m), intensity."""
    empty = (np.empty(0, int), np.empty(0, int),
             np.empty(0, np.float32), np.empty(0, np.float32))
    if pts.shape[0] == 0 or P is None or Tr is None:
        return empty

    Tr_full = np.eye(4)
    Tr_full[:3, :4] = Tr
    pts_h = np.hstack((pts, np.ones((pts.shape[0], 1))))
    cam = (Tr_full @ pts_h.T).T
    front = cam[:, 2] > 0.1                 # keep points in front of the camera
    cam, amp = cam[front], intensity[front]
    if cam.shape[0] == 0:
        return empty

    cam_rect = (R0 @ cam[:, :3].T).T
    rect_h = np.hstack((cam_rect, np.ones((cam_rect.shape[0], 1))))
    proj = (P @ rect_h.T).T
    u = proj[:, 0] / proj[:, 2]
    v = proj[:, 1] / proj[:, 2]
    rng = np.linalg.norm(cam_rect, axis=1)

    ui = np.round(u).astype(int)
    vi = np.round(v).astype(int)
    inb = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    return ui[inb], vi[inb], rng[inb].astype(np.float32), amp[inb].astype(np.float32)


def build_maps(u, v, rng, amp, W, H):
    """Splat visible points into dense depth/intensity images, z-buffered so the
    nearest point wins each pixel. Empty pixels stay 0."""
    depth_map = np.zeros((H, W), np.float32)
    amp_map = np.zeros((H, W), np.float32)
    if u.size:
        order = np.argsort(-rng)            # far -> near, so near overwrites
        depth_map[v[order], u[order]] = rng[order]
        amp_map[v[order], u[order]] = amp[order]
    return depth_map, amp_map


def project_maps(pts, inten, P, Tr, R0, W, H):
    """Convenience: project + splat -> (depth_map, amp_map, n_points)."""
    u, v, rng, amp = project_to_image(pts, inten, P, Tr, R0, W, H)
    depth_map, amp_map = build_maps(u, v, rng, amp, W, H)
    return depth_map, amp_map, int(u.size)


def colorize_map(value_map, vmin, vmax):
    """Sparse value map -> jet BGR over the FIXED window [vmin, vmax]; empty
    (<=0) pixels stay black. Fixed windows are the point: a colour then reads the
    same on every frame, which is what the colour bars annotate. Used for both
    range (metres) on rgb and amplitude (16-bit) on thermal."""
    valid = value_map > 0
    if vmax <= vmin:
        vmax = vmin + 1e-3
    norm = np.clip((value_map - vmin) / (vmax - vmin), 0, 1)
    bgr = cv2.applyColorMap((norm * 255).astype(np.uint8), JET)
    bgr[~valid] = 0
    return bgr


def overlay_points(pane, color_map, point_size=TOF_POINT_SIZE, opacity=TOF_OPACITY):
    """Splat a coloured point map onto a camera pane (in place). `pane` may be a
    slice-view of the stitched image."""
    layer = dilate(color_map, point_size)
    mask = layer.sum(2) > 0
    if mask.any():
        pane[mask] = (opacity * layer[mask] + (1 - opacity) * pane[mask]).astype(np.uint8)


def draw_colorbar(img, pane_x0, pane_x1, vmin, vmax, title, fmt="{:.1f}"):
    """Vertical JET colour scale near the right edge of a pane, with a `title`
    (e.g. 'range' / 'amplitude') above it and vmin/vmax labels beside it, so it is
    obvious which pane shows what."""
    h = img.shape[0]
    bar_h, bar_w = int(h * 0.6), 14
    y0 = (h - bar_h) // 2
    x = pane_x1 - bar_w - 60
    if x < pane_x0:
        return
    grad = np.repeat(np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(-1, 1), bar_w, axis=1)
    img[y0:y0 + bar_h, x:x + bar_w] = cv2.applyColorMap(grad, JET)
    cv2.rectangle(img, (x, y0), (x + bar_w, y0 + bar_h), (255, 255, 255), 1)
    # title above the bar, right-aligned to the pane edge so it never overflows
    (tw, _), _ = cv2.getTextSize(title, FONT, 0.45, 1)
    tx = int(np.clip(pane_x1 - tw - 6, pane_x0 + 2, max(pane_x0 + 2, pane_x1 - tw - 2)))
    _label(img, title, (tx, max(14, y0 - 8)), (255, 255, 255), 0.45)
    for frac, val in ((0.0, vmax), (1.0, vmin)):
        ty = int(np.clip(y0 + frac * bar_h, 10, h - 4))
        cv2.putText(img, fmt.format(val), (x + bar_w + 4, ty + 4),
                    FONT, 0.4, (255, 255, 255), 1, cv2.LINE_AA)


# =====================================================================
# 2D labels (YOLO + KITTI bbox), solid = GT, dashed = predictions
# =====================================================================
CLASS_COLORS = {
    "person":    (0, 255,   0),
    "bicycle":   (255, 255, 0),
    "doll":      (0,   0, 255),
    "slidecar":  (255,   0, 0),
    "vegetation": (0, 180, 0),
    "curb":      (0, 165, 255),
}
ID2CLS = {
    0: "person",
    1: "bicycle",
    2: "slidecar",
    3: "doll",
    4: "vegetation",
    5: "curb",
}


def read_yolo_labels(path, img_w, img_h):
    """YOLO boxes `cls xc yc w h [dist] [conf]` (normalised). The 6th column is the
    SEW distance when present (GT); extra columns (a prediction confidence) are
    ignored. Works for both GT and prediction files."""
    boxes = []
    if not path or not os.path.exists(path):
        return boxes
    with open(path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 5:
                continue
            try:
                cls_id = int(float(parts[0]))
                xc = float(parts[1]) * img_w
                yc = float(parts[2]) * img_h
                w  = float(parts[3]) * img_w
                h  = float(parts[4]) * img_h
            except ValueError:
                continue
            dist = None
            if len(parts) >= 6:
                try:
                    dist = float(parts[5])
                except ValueError:
                    dist = None
            boxes.append({
                "cls_id": cls_id,
                "x1": int(xc - w / 2), "y1": int(yc - h / 2),
                "x2": int(xc + w / 2), "y2": int(yc + h / 2),
                "dist": dist,
            })
    return boxes


def draw_yolo_boxes(img, boxes, x_offset=0, dashed=False, tag_prefix=""):
    for b in boxes:
        cls_name = ID2CLS.get(b["cls_id"], "unknown")
        color    = CLASS_COLORS.get(cls_name, (255, 255, 255))
        x1, y1 = b["x1"] + x_offset, b["y1"]
        x2, y2 = b["x2"] + x_offset, b["y2"]
        if x2 <= x1 or y2 <= y1:
            continue
        _rect(img, (x1, y1), (x2, y2), color, 2, dashed=dashed)
        tag = tag_prefix + cls_name
        if b["dist"] is not None:
            tag += f" {b['dist']:.1f}m"
        cv2.putText(img, tag, (x1, max(12, y1 - 5)), FONT, 0.5, color, 1, cv2.LINE_AA)


def draw_kitti_2d_boxes(img, boxes, x_offset=0, dashed=False, tag_prefix=""):
    """Draw the 2D bbox part of KITTI labels (columns 4-7) on the rgb pane."""
    for b in boxes:
        bl, bt, br, bb = b["bbox"]
        cls   = b["cls"]
        color = CLASS_COLORS.get(cls.lower(), (0, 165, 255))  # orange fallback
        x1, y1, x2, y2 = int(bl) + x_offset, int(bt), int(br) + x_offset, int(bb)
        if x2 <= x1 or y2 <= y1:                   # skip degenerate / DontCare rows
            continue
        _rect(img, (x1, y1), (x2, y2), color, 2, dashed=dashed)
        cv2.putText(img, tag_prefix + cls, (x1, max(12, y1 - 5)),
                    FONT, 0.5, color, 1, cv2.LINE_AA)


# =====================================================================
# Timestamps: per-sensor keyframe capture-time spread (RGB / THERMAL / TOF)
# =====================================================================
def _label(img, text, org, color=(255, 255, 255), scale=0.55):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, 1, cv2.LINE_AA)


def load_timestamps(ts_folder, name):
    """Keyframe capture time (integer nanoseconds) for RGB/THERMAL/TOF, or None."""
    if not ts_folder:
        return None
    path = os.path.join(ts_folder, name + ".json")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    out = {}
    for s in ("RGB", "THERMAL", "TOF"):
        ts = data.get(s, {}).get("key_frame", {}).get("timestamp")
        if ts and "sec" in ts and "nsec" in ts:
            out[s] = int(ts["sec"]) * 1_000_000_000 + int(ts["nsec"])
    return out or None


def sync_deltas(ts):
    """(deltas_ms relative to TOF, spread_ms). TOF is the projection anchor, so
    the delta reads as 'how far each image was captured from the ToF frame'."""
    if not ts:
        return None, 0.0
    ref = ts.get("TOF", min(ts.values()))
    deltas = {s: (t - ref) / 1e6 for s, t in ts.items()}
    spread = (max(ts.values()) - min(ts.values())) / 1e6
    return deltas, spread


def draw_sync_bar(img, deltas, spread, x, y, w, win_ms=SYNC_WINDOW_MS):
    """Compact temporal-alignment gauge: a timeline with ToF pinned at the centre
    (0) and RGB/THERMAL placed by their ms offset; the spread headline goes
    green -> orange -> red as the sensors drift apart."""
    _panel(img, x - 10, y - 30, w + 20, 74, alpha=0.55)
    good = (80, 255, 80) if spread <= 5 else (0, 200, 255) if spread <= win_ms else (60, 60, 255)
    _label(img, f"sync spread {spread:.2f} ms   (0 = ToF capture)", (x, y - 14), good, 0.48)

    xc = x + w // 2
    cv2.line(img, (x, y), (x + w, y), (180, 180, 180), 1)
    cv2.line(img, (xc, y - 6), (xc, y + 6), (120, 120, 120), 1)   # ToF reference tick
    scale = (w / 2) / win_ms
    for s in ("RGB", "THERMAL", "TOF"):
        if s not in deltas:
            continue
        px = int(np.clip(xc + deltas[s] * scale, x, x + w))
        cv2.circle(img, (px, y), 5, SENSOR_COL[s], -1)
        cv2.circle(img, (px, y), 5, (20, 20, 20), 1)

    lx = x
    for s in ("RGB", "THERMAL", "TOF"):
        if s not in deltas:
            continue
        txt = f"{s} {deltas[s]:+.2f}"
        _label(img, txt, (lx, y + 26), SENSOR_COL[s], 0.45)
        (tw, _), _ = cv2.getTextSize(txt, FONT, 0.45, 1)
        lx += tw + 16
    _label(img, f"+/-{win_ms:.0f}ms", (x + w - 58, y + 26), (150, 150, 150), 0.4)


# =====================================================================
# Render one frame -> stitched BGR image (rgb | thermal)
# =====================================================================
def render_frame(name, cfg, flags):
    """Build the stitched rgb|thermal visualization for one frame.
    Raises ValueError if the images are missing / unreadable."""
    label_path   = os.path.join(cfg.label_folder, name + ".txt") if cfg.label_folder else ""
    calib_path   = os.path.join(cfg.calib_folder, name + ".txt") if cfg.calib_folder else ""
    rgb_path     = os.path.join(cfg.image_rgb_folder, name + cfg.image_ext)
    thermal_path = os.path.join(cfg.image_thermal_folder, name + cfg.image_ext)

    rgb_img     = cv2.imread(rgb_path)
    thermal_img = cv2.imread(thermal_path)
    if rgb_img is None:
        raise ValueError(f"cannot read RGB image: {rgb_path}")
    if thermal_img is None:
        raise ValueError(f"cannot read thermal image: {thermal_path}")

    boxes_gt = read_labels(label_path)
    P2, P3, Tr, _ = read_calib(calib_path)
    R0 = make_manual_R0()      # identity for both datasets (manual tilt is 0)

    if rgb_img.shape[0] != thermal_img.shape[0]:      # match heights so hstack works
        h = min(rgb_img.shape[0], thermal_img.shape[0])
        rgb_img, thermal_img = rgb_img[:h], thermal_img[:h]

    stitched  = np.hstack([rgb_img, thermal_img])
    H         = rgb_img.shape[0]
    rgb_w     = rgb_img.shape[1]
    thermal_w = thermal_img.shape[1]
    total_w   = rgb_w + thermal_w
    have_left  = P2 is not None and Tr is not None
    have_right = P3 is not None and Tr is not None

    # --- ToF point cloud: range-coloured on rgb, amplitude-coloured on thermal ---
    if flags["tof_pcl"]:
        pts, inten = load_tof_cloud(cfg, name)
        if pts.shape[0]:
            if have_left:
                d_map, _, _ = project_maps(pts, inten, P2, Tr, R0, rgb_w, H)
                overlay_points(stitched[:, :rgb_w],
                               colorize_map(d_map, TOF_DEPTH_MIN_M, TOF_DEPTH_MAX_M))
            if have_right:
                _, a_map, _ = project_maps(pts, inten, P3, Tr, R0, thermal_w, H)
                overlay_points(stitched[:, rgb_w:total_w],
                               colorize_map(a_map, TOF_AMP_MIN, TOF_AMP_MAX))

    # --- ground-truth 3D boxes (solid white, both panes) ---
    if flags["box3d"] and P2 is not None:
        for box in boxes_gt:
            corners = box_corners_in_cam(box)
            draw_3d_box(stitched, project_corners(P2, corners).copy(), corners, box["cls"], 0, 0, rgb_w)
            draw_3d_box(stitched, project_corners(P3, corners).copy(), corners, box["cls"], rgb_w, rgb_w, total_w)

    # --- ground-truth 2D YOLO (thermal / rgb) ---
    if flags["thermal_yolo"] and cfg.yolo_thermal_folder:
        tb = read_yolo_labels(os.path.join(cfg.yolo_thermal_folder, name + ".txt"), thermal_w, H)
        draw_yolo_boxes(stitched, tb, x_offset=rgb_w)
    if flags["rgb_yolo"] and cfg.yolo_rgb_folder:
        rb = read_yolo_labels(os.path.join(cfg.yolo_rgb_folder, name + ".txt"), rgb_w, H)
        draw_yolo_boxes(stitched, rb, x_offset=0)

    # --- ground-truth rgb 2D KITTI bbox ---
    if flags["rgb_kitti_2d"] and boxes_gt:
        draw_kitti_2d_boxes(stitched, boxes_gt, x_offset=0)

    # --- predictions (dashed), cycled with 'p': off / 2D / 3D / 2D+3D ---
    # 2D = YOLO in its own pane (rgb->rgb, thermal->thermal); 3D = KITTI in both.
    if flags["predictions"]:
        show_2d = flags["predictions"] in (PRED_2D, PRED_BOTH)
        show_3d = flags["predictions"] in (PRED_3D, PRED_BOTH)
        if show_3d and cfg.pred_kitti_folder and P2 is not None:
            for box in read_labels(os.path.join(cfg.pred_kitti_folder, name + ".txt")):
                corners = box_corners_in_cam(box)
                color = CLASS_COLORS.get(box["cls"].lower(), (0, 165, 255))
                draw_3d_box(stitched, project_corners(P2, corners).copy(), corners, box["cls"],
                            0, 0, rgb_w, color=color, dashed=True, tag_prefix="P:")
                draw_3d_box(stitched, project_corners(P3, corners).copy(), corners, box["cls"],
                            rgb_w, rgb_w, total_w, color=color, dashed=True, tag_prefix="P:")
        if show_2d and cfg.pred_yolo_rgb_folder:
            pr = read_yolo_labels(os.path.join(cfg.pred_yolo_rgb_folder, name + ".txt"), rgb_w, H)
            draw_yolo_boxes(stitched, pr, x_offset=0, dashed=True, tag_prefix="P:")
        if show_2d and cfg.pred_yolo_thermal_folder:
            pt = read_yolo_labels(os.path.join(cfg.pred_yolo_thermal_folder, name + ".txt"), thermal_w, H)
            draw_yolo_boxes(stitched, pt, x_offset=rgb_w, dashed=True, tag_prefix="P:")

    # --- colour bars: range on the rgb pane, amplitude on the thermal pane ---
    if flags["tof_pcl"] and flags["colorbars"]:
        draw_colorbar(stitched, 0, rgb_w, TOF_DEPTH_MIN_M, TOF_DEPTH_MAX_M, "range", "{:.1f}m")
        draw_colorbar(stitched, rgb_w, total_w, TOF_AMP_MIN, TOF_AMP_MAX, "amplitude", "{:.0f}")

    return stitched


# =====================================================================
# On-screen overlay / chrome
# =====================================================================
def _panel(img, x, y, w, h, alpha=0.55):
    """Darken a rectangle in place so overlaid text stays readable."""
    x2, y2 = min(x + w, img.shape[1]), min(y + h, img.shape[0])
    x, y = max(x, 0), max(y, 0)
    sub = img[y:y2, x:x2]
    if sub.size:
        cv2.addWeighted(np.zeros_like(sub), alpha, sub, 1 - alpha, 0, sub)


NAV_HELP = [
    "n / ->/ space  next frame",
    "b / <-         previous frame",
    "m +100   v -100   g  goto id",
    "s  save view    h  toggle help",
    "q / ESC  quit",
]
OVERLAY_HELP = [
    "1 ToF point cloud (range|amplitude)",
    "2 3D boxes      3 thermal 2D YOLO",
    "4 rgb 2D YOLO   5 rgb 2D KITTI box",
    "p predictions cycle off/2D/3D/2D+3D",
    "c colour bars (range | amplitude)",
    "t sync-spread bar",
]


def draw_overlay(disp, name, idx, total, cfg, flags, show_sync, show_help, error=None):
    h, w = disp.shape[:2]

    # top bar: frame index + name (left), layer status (right)
    _panel(disp, 0, 0, w, 34)
    cv2.putText(disp, f"[{idx + 1}/{total}]  {name}", (10, 23),
                FONT, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    layers = [("tof", flags["tof_pcl"]), ("3d", flags["box3d"]),
              ("th-yolo", flags["thermal_yolo"]), ("rgb-yolo", flags["rgb_yolo"]),
              ("rgb-2d", flags["rgb_kitti_2d"]), ("cbar", flags["colorbars"])]
    status = "  ".join(f"{n}:{'on' if v else 'off'}" for n, v in layers)
    status += f"  pred:{PRED_LABELS[flags['predictions']]}"
    (tw, _), _ = cv2.getTextSize(status, FONT, 0.5, 1)
    cv2.putText(disp, status, (max(10, w - tw - 10), 22),
                FONT, 0.5, (200, 220, 255), 1, cv2.LINE_AA)

    cv2.putText(disp, "h: help", (max(10, w - 78), h - 12),
                FONT, 0.5, (180, 180, 180), 1, cv2.LINE_AA)

    if error:
        _panel(disp, 0, h // 2 - 22, w, 44, alpha=0.6)
        cv2.putText(disp, "ERROR", (10, h // 2 - 2), FONT, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(disp, error[:110], (10, h // 2 + 20), FONT, 0.5, (200, 200, 255), 1, cv2.LINE_AA)

    if show_help:
        lines = NAV_HELP + ["", "Overlay keys"] + OVERLAY_HELP
        lh, pad, panel_w = 20, 12, 360
        _panel(disp, pad, 40, panel_w, pad * 2 + lh * len(lines), alpha=0.68)
        y = 40 + pad + 13
        for line in lines:
            cv2.putText(disp, line, (pad + 12, y), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y += lh

    if show_sync and error is None:
        deltas, spread = sync_deltas(load_timestamps(cfg.timestamps_folder, name))
        if deltas:
            draw_sync_bar(disp, deltas, spread, 14, h - 36, min(440, w - 40))
        else:
            _label(disp, "no timestamps for this frame", (14, h - 16), (150, 150, 150), 0.45)


# =====================================================================
# Display helpers
# =====================================================================
def fit_to_display(img, max_w, max_h):
    h, w = img.shape[:2]
    if w == 0 or h == 0:
        return img.copy()
    scale = min(max_w / w, max_h / h)
    scale = max(0.1, min(scale, 4.0))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(img, None, fx=scale, fy=scale, interpolation=interp)


def window_alive(win):
    try:
        return cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return True


def placeholder_canvas(cfg):
    w = min(1280, cfg.display_max_width)
    h = min(480, cfg.display_max_height)
    return np.zeros((h, w, 3), dtype=np.uint8)


def save_current(cfg, name, base_img):
    if base_img is None:
        print("  [!] nothing to save (frame failed to render)")
        return
    os.makedirs(cfg.output_folder, exist_ok=True)
    out = os.path.join(cfg.output_folder, name + ".jpg")
    big = cv2.resize(base_img, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
    cv2.imwrite(out, big)
    print(f"  saved {out}")


# =====================================================================
# Frame discovery / navigation
# =====================================================================
def _list_ids(folder, exts):
    if not folder or not os.path.isdir(folder):
        return []
    out = []
    for f in os.listdir(folder):
        low = f.lower()
        for e in exts:
            if low.endswith(e):
                out.append(f[:len(f) - len(e)])
                break
    return sorted(set(out))


def list_frames(cfg):
    """Frame ids, taken from the first modality that has any. The label folder is
    the canonical dataset index; the rest are fallbacks for partial downloads."""
    for folder, exts in (
        (cfg.label_folder, (".txt",)),
        (cfg.image_rgb_folder, (cfg.image_ext.lower(),)),
        (cfg.bin_folder, (".bin",)),
        (cfg.image_thermal_folder, (cfg.image_ext.lower(),)),
    ):
        ids = _list_ids(folder, exts)
        if ids:
            return ids
    return []


def find_frame(names, query):
    """Resolve a user query to an index: exact id, or numeric id ignoring
    zero-padding (so '123' matches '000123'). Returns None if not found."""
    if query in names:
        return names.index(query)
    if query.lstrip("-").isdigit():
        q = int(query)
        for i, nm in enumerate(names):
            if nm.isdigit() and int(nm) == q:
                return i
    return None


def read_frame_ids(path):
    """Read a plain list of frame ids (one per line) from a text file - e.g. the
    failure_analysis explorer's `df_frames_*.txt`. Blank lines and everything
    after a '#' are ignored."""
    ids = []
    with open(path, "r") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if line:
                ids.append(line)
    return ids


def filter_frames(names, wanted):
    """Restrict `names` (dataset order) to the requested ids, matching like
    find_frame. Returns (filtered_names, missing)."""
    name_set = set(names)
    num_index = {int(nm): nm for nm in names if nm.isdigit()}
    resolved, missing = set(), []
    for q in wanted:
        if q in name_set:
            resolved.add(q)
        elif q.lstrip("-").isdigit() and int(q) in num_index:
            resolved.add(num_index[int(q)])
        else:
            missing.append(q)
    filtered = [nm for nm in names if nm in resolved]
    return filtered, missing


def prompt_jump(names, cur):
    print(f"\n  current: [{cur + 1}/{len(names)}] {names[cur]}")
    try:
        query = input("  go to frame id (empty = cancel): ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return cur
    if not query:
        return cur
    j = find_frame(names, query)
    if j is None:
        print(f"  [!] frame '{query}' not found")
        return cur
    print(f"  -> {names[j]}")
    return j


# =====================================================================
# Interactive + export
# =====================================================================
def print_intro(cfg, names):
    print("=" * 64)
    print(" SEW Multimodal AMR Dataset Viewer   (rgb | thermal)")
    print("=" * 64)
    print(f" config   : {cfg.name}" + (f"   split: {cfg.split}" if cfg.split else ""))
    print(f" base_dir : {cfg.base_dir}")
    print(f" frames   : {len(names)}   ({names[0]} ... {names[-1]})")
    print(" controls : n/b next/prev | m/v +-100 | g goto | 1-5,p layers | c bars | t sync | s save | h help | q quit")
    print("=" * 64)


def run_interactive(cfg, flags, names, start_frame=None):
    if not names:
        print("[!] no frames found. Checked:")
        print(f"      label folder : {cfg.label_folder}")
        print(f"      rgb folder   : {cfg.image_rgb_folder}")
        print("    Check `base_dir` and the folder paths in the config file.")
        return

    idx = 0
    if start_frame:
        j = find_frame(names, start_frame)
        if j is None:
            print(f"[!] start frame '{start_frame}' not found; starting at the first frame")
        else:
            idx = j

    print_intro(cfg, names)

    win = "SEW Dataset Viewer  -  " + cfg.name
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1400, 800)
    except cv2.error as e:
        print(f"[!] could not open a display window: {e}")
        print("    Run with --export to render frames to disk instead.")
        return

    show_help = False
    show_sync = False
    need_render = True
    base_img = None

    while True:
        if need_render:
            name = names[idx]
            error = None
            try:
                base_img = render_frame(name, cfg, flags)
                disp = fit_to_display(base_img, cfg.display_max_width, cfg.display_max_height)
            except Exception as e:              # keep the viewer alive on a bad frame
                base_img = None
                error = str(e)
                disp = placeholder_canvas(cfg)
            draw_overlay(disp, name, idx, len(names), cfg, flags, show_sync, show_help, error)
            cv2.imshow(win, disp)
            need_render = False

        key = cv2.waitKeyEx(50)
        if key == -1:
            if not window_alive(win):
                break
            continue

        if key in (ord('q'), 27):
            break
        elif key == ord('n') or key == ord(' ') or key in ARROW_NEXT:
            idx = (idx + 1) % len(names); need_render = True
        elif key == ord('b') or key in ARROW_PREV:
            idx = (idx - 1) % len(names); need_render = True
        elif key == ord('m'):
            idx = min(idx + 100, len(names) - 1); need_render = True
        elif key == ord('v'):
            idx = max(idx - 100, 0); need_render = True
        elif key == ord('g'):
            idx = prompt_jump(names, idx); need_render = True
        elif key == ord('1'):
            flags["tof_pcl"] = not flags["tof_pcl"]; need_render = True
        elif key == ord('2'):
            flags["box3d"] = not flags["box3d"]; need_render = True
        elif key == ord('3'):
            flags["thermal_yolo"] = not flags["thermal_yolo"]; need_render = True
        elif key == ord('4'):
            flags["rgb_yolo"] = not flags["rgb_yolo"]; need_render = True
        elif key == ord('5'):
            flags["rgb_kitti_2d"] = not flags["rgb_kitti_2d"]; need_render = True
        elif key == ord('p'):
            flags["predictions"] = (flags["predictions"] + 1) % 4; need_render = True
        elif key == ord('c'):
            flags["colorbars"] = not flags["colorbars"]; need_render = True
        elif key == ord('t'):
            show_sync = not show_sync; need_render = True
        elif key == ord('h'):
            show_help = not show_help; need_render = True
        elif key == ord('s'):
            save_current(cfg, names[idx], base_img)

        if not window_alive(win):
            break

    cv2.destroyAllWindows()


def run_export(cfg, flags, names):
    if not names:
        print("[!] no frames found to export.")
        return
    os.makedirs(cfg.output_folder, exist_ok=True)
    ok = 0
    for name in names:
        try:
            img = render_frame(name, cfg, flags)
        except Exception as e:
            print(f"  skip {name}: {e}")
            continue
        big = cv2.resize(img, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
        out = os.path.join(cfg.output_folder, name + ".jpg")
        cv2.imwrite(out, big)
        ok += 1
        print(f"  saved {out}")
    print(f"\nDone. {ok}/{len(names)} frames exported to {cfg.output_folder}")


def run_amp_stats(cfg, names):
    """Debug (headless): print the ToF amplitude (= point-cloud intensity, the
    values coloured on the thermal pane) per frame, then a summary, to help pick
    TOF_AMP_MAX. Reads the same source the viewer colours (.bin, or .pcd when
    use_pcd). Tip: narrow with --split / --frames, or pipe to a file."""
    if not names:
        print("[!] no frames found."); return
    src = "pcd" if cfg.use_pcd else "bin"
    print(f"# ToF amplitude stats over {len(names)} frame(s)   (source: {src})")
    print(f"{'frame':>12}  {'n_pts':>8}  {'amp_min':>10}  {'amp_mean':>10}  {'amp_max':>10}")
    per_frame_max, gmax = [], 0.0
    for name in names:
        _, inten = load_tof_cloud(cfg, name)
        if inten.size == 0:
            print(f"{name:>12}  {0:>8}  {'-':>10}  {'-':>10}  {'-':>10}")
            continue
        amin, amean, amax = float(inten.min()), float(inten.mean()), float(inten.max())
        per_frame_max.append(amax)
        gmax = max(gmax, amax)
        print(f"{name:>12}  {inten.size:>8}  {amin:>10.1f}  {amean:>10.1f}  {amax:>10.1f}")

    if not per_frame_max:
        print("[!] no frames had any points."); return
    arr = np.array(per_frame_max)
    p50, p90, p99 = (float(x) for x in np.percentile(arr, [50, 90, 99]))
    print("-" * 58)
    print(f" frames with points : {len(per_frame_max)}/{len(names)}")
    print(f" global amp max     : {gmax:.1f}")
    print(f" per-frame max  p50 / p90 / p99 : {p50:.1f} / {p90:.1f} / {p99:.1f}")
    print(f" -> a good TOF_AMP_MAX is around p99 = {p99:.0f} (round up); "
          f"the current constant is {TOF_AMP_MAX:.0f}")


# =====================================================================
# Entry point
# =====================================================================
def resolve_config_path(arg):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if not arg:
        return os.path.join(script_dir, "config_sew.yaml")
    if os.path.isabs(arg) or os.path.exists(arg):
        return os.path.abspath(arg)
    return os.path.join(script_dir, arg)  # look next to the script


def resolve_names(cfg, frames_file):
    """Full frame list, optionally restricted to the ids in `frames_file`."""
    all_names = list_frames(cfg)
    if not frames_file:
        return all_names
    if not os.path.exists(frames_file):
        print(f"[!] frames file not found: {frames_file}")
        sys.exit(1)
    wanted = read_frame_ids(frames_file)
    names, missing = filter_frames(all_names, wanted)
    print(f"[frames] filter '{os.path.basename(frames_file)}': "
          f"{len(names)}/{len(wanted)} requested frame(s) present in the dataset")
    if missing:
        preview = ", ".join(missing[:10]) + (" ..." if len(missing) > 10 else "")
        print(f"[frames] {len(missing)} not found in {cfg.name}: {preview}")
    if not names:
        print("[!] none of the requested frame ids are in this dataset - nothing to show.")
        sys.exit(1)
    return names


def main():
    ap = argparse.ArgumentParser(
        description="SEW Multimodal AMR Dataset Viewer (KITTI-compatible).")
    ap.add_argument("-c", "--config", default=None,
                    help="dataset config YAML (default: config_sew.yaml next to this script)")
    ap.add_argument("--split", default=None,
                    help="dataset split to view (e.g. train / test / val); fills the {split} token "
                         "in the config's folder paths and overrides the config's 'split:'.")
    ap.add_argument("-f", "--frame", default=None,
                    help="frame id to open at startup, e.g. 000123")
    ap.add_argument("--frames", default=None, metavar="FILE",
                    help="restrict the viewer to the frame ids listed in FILE (one id per line, "
                         "e.g. a failure_analysis df_frames_*.txt).")
    ap.add_argument("--export", action="store_true",
                    help="batch-render every frame to the output folder and exit")
    ap.add_argument("--amp-stats", action="store_true",
                    help="debug: print the ToF amplitude (intensity) max per frame + a summary, then "
                         "exit (headless; helps pick TOF_AMP_MAX). Respects --split / --frames.")
    args = ap.parse_args()

    cfg_path = resolve_config_path(args.config)
    if not os.path.exists(cfg_path):
        print(f"[!] config not found: {cfg_path}")
        sys.exit(1)

    cfg = load_config(cfg_path, split_override=args.split)
    flags = {
        "tof_pcl":      cfg.show_tof_pcl,
        "box3d":        cfg.show_3d_label,
        "thermal_yolo": cfg.show_thermal_label,
        "rgb_yolo":     cfg.show_rgb_label,
        "rgb_kitti_2d": cfg.show_rgb_kitti_2d,
        "predictions":  PRED_BOTH if cfg.show_predictions else PRED_OFF,
        "colorbars":    cfg.show_colorbars,
    }
    names = resolve_names(cfg, args.frames)

    if args.amp_stats:
        run_amp_stats(cfg, names)
    elif args.export:
        run_export(cfg, flags, names)
    else:
        run_interactive(cfg, flags, names, args.frame)


if __name__ == "__main__":
    main()
