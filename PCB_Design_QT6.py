#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCB MASTER SUITE (Universal Edition: PCB_aster-suite_3D.py)
Supports Python 3.8–3.13+ on Windows with seamless PyQt5 / PyQt6 dual-compatibility.
Features:
  1. Schematic Capture & Live Forward-Annotation Netlist Sync
  2. SPICE Time-Domain Transient & FFT Circuit Simulator
  3. Interactive 2D PCB CAD Canvas & A* Grid Autorouter
  4. 3D Orbiting Board & Thermal Extrusion Viewer
  5. Parametric Footprint & Package Generator
  6. Multi-Layer Physical Stackup & Materials Manager
  7. DC MNA IR-Drop & IPC-2152 Current Density Solver
  8. Coupled 3D Finite-Volume Electrothermal FEA
  9. High-Speed Transmission Line & Signal Integrity Synthesizer
 10. PDN Decoupling & Plane Cavity Resonance Analyzer
 11. Fabrication CAM Exporter (RS-274X Gerber, Drill, BOM, CPL)
 12. Native KiCad 6/7/8/9 (.kicad_pcb) & DXF Mechanical Importers
"""

from __future__ import annotations

import csv
import dataclasses
from dataclasses import dataclass, field
import heapq
import json
import math
import pathlib
import re
import subprocess
import sys
import uuid
from typing import Any, Dict, List, Optional, Set, Tuple


# ============================================================================
# 0. DUAL-COMPATIBILITY QT ABSTRACTION & AUTOMATIC BOOTSTRAPPER
# ============================================================================

def _ensure_runtime_dependencies() -> None:
    if getattr(sys, "frozen", False):
        return  # Frozen inside PyInstaller .exe

    def _is_avail(mod: str) -> bool:
        try:
            __import__(mod)
            return True
        except ImportError:
            return False

    required = [("numpy", "numpy"), ("scipy", "scipy"), ("pyqtgraph", "pyqtgraph")]
    missing = [pip_pkg for mod_name, pip_pkg in required if not _is_avail(mod_name)]

    has_qt = _is_avail("PyQt6") or _is_avail("PyQt5")
    if not has_qt:
        # Prefer PyQt6 on Python 3.13+ where PyQt5 wheels do not exist
        qt_pkg = "PyQt6" if sys.version_info >= (3, 13) else "PyQt5"
        missing.append(qt_pkg)

    if missing:
        print(f"[PCB Master Suite] Installing missing packages into {sys.executable}:")
        print(f"                   {', '.join(missing)}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", *missing])
        print("[PCB Master Suite] Dependencies verified successfully!\n")


_ensure_runtime_dependencies()

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

# Dynamic PyQt6 / PyQt5 cross-version import bridge
try:
    from PyQt6 import QtCore, QtGui, QtWidgets
    from PyQt6.QtCore import Qt, QPointF, QRectF
    QT_VERSION = 6
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtCore import Qt, QPointF, QRectF
    QT_VERSION = 5

import pyqtgraph as pg
pg.setConfigOptions(antialias=True, background="#141820", foreground="#dce3ea")

# Unified Enum & Event Access across Qt5 / Qt6
LeftButton = getattr(getattr(Qt, 'MouseButton', Qt), 'LeftButton')
RightButton = getattr(getattr(Qt, 'MouseButton', Qt), 'RightButton')
MiddleButton = getattr(getattr(Qt, 'MouseButton', Qt), 'MiddleButton')
AltModifier = getattr(getattr(Qt, 'KeyboardModifier', Qt), 'AltModifier')
Key_R = getattr(getattr(Qt, 'Key', Qt), 'Key_R')
Key_Escape = getattr(getattr(Qt, 'Key', Qt), 'Key_Escape')
SolidLine = getattr(getattr(Qt, 'PenStyle', Qt), 'SolidLine')
DashLine = getattr(getattr(Qt, 'PenStyle', Qt), 'DashLine')
DotLine = getattr(getattr(Qt, 'PenStyle', Qt), 'DotLine')
NoBrush = getattr(getattr(Qt, 'BrushStyle', Qt), 'NoBrush')
RoundCap = getattr(getattr(Qt, 'PenCapStyle', Qt), 'RoundCap')
RoundJoin = getattr(getattr(Qt, 'PenJoinStyle', Qt), 'RoundJoin')
AlignCenter = getattr(getattr(Qt, 'AlignmentFlag', Qt), 'AlignCenter')
Horizontal = getattr(getattr(Qt, 'Orientation', Qt), 'Horizontal')
StrongFocus = getattr(getattr(Qt, 'FocusPolicy', Qt), 'StrongFocus')


def mouse_pos_f(event: QtGui.QMouseEvent) -> QPointF:
    if hasattr(event, "position"):
        p = event.position()
        return QPointF(p.x(), p.y())
    elif hasattr(event, "localPos"):
        return event.localPos()
    return QPointF(float(event.x()), float(event.y()))


# ============================================================================
# 1. CORE DOMAIN MODEL (SCHEMATIC + 2D/3D PCB + PHYSICS)
# ============================================================================

@dataclass
class CopperLayer:
    layer_id: str
    name: str
    display_number: int
    thickness_um: float = 35.0
    conductivity_s_m: float = 5.80e7
    tempco_per_k: float = 0.00393
    thermal_k_w_mk: float = 385.0
    color_hex: str = "#e04f4f"
    visible: bool = True


@dataclass
class DielectricLayer:
    diel_id: str
    name: str
    thickness_um: float = 1500.0
    er: float = 4.4
    loss_tangent: float = 0.02
    thermal_k_w_mk: float = 0.32


@dataclass
class Pad:
    pad_id: str
    pin_number: str
    pin_name: str
    rel_x_mm: float
    rel_y_mm: float
    width_mm: float
    height_mm: float
    layer_id: str
    shape: str = "RECT"
    drill_mm: float = 0.0
    net_name: str = ""
    is_thermal_pad: bool = False


@dataclass
class Component:
    comp_id: str
    refdes: str
    package_name: str
    value: str
    x_mm: float
    y_mm: float
    width_mm: float
    height_mm: float
    rotation_deg: float = 0.0
    layer_id: str = "L1"
    power_w: float = 0.0
    r_theta_jb: float = 10.0
    r_theta_ja: float = 45.0
    max_tj_c: float = 105.0
    height_3d_mm: float = 2.0
    sch_x: float = 100.0
    sch_y: float = 100.0
    pads: List[Pad] = field(default_factory=list)

    def pad_world_pos(self, pad: Pad) -> Tuple[float, float]:
        rad = math.radians(self.rotation_deg)
        ca, sa = math.cos(rad), math.sin(rad)
        wx = self.x_mm + (pad.rel_x_mm * ca - pad.rel_y_mm * sa)
        wy = self.y_mm + (pad.rel_x_mm * sa + pad.rel_y_mm * ca)
        return wx, wy


@dataclass
class SchematicWire:
    wire_id: str
    from_comp_id: str
    from_pin_number: str
    to_comp_id: str
    to_pin_number: str
    net_name: str


@dataclass
class Trace:
    trace_id: str
    net_name: str
    layer_id: str
    x1_mm: float
    y1_mm: float
    x2_mm: float
    y2_mm: float
    width_mm: float = 0.35

    def length_mm(self) -> float:
        return math.hypot(self.x2_mm - self.x1_mm, self.y2_mm - self.y1_mm)


@dataclass
class Via:
    via_id: str
    net_name: str
    x_mm: float
    y_mm: float
    outer_dia_mm: float = 0.70
    drill_dia_mm: float = 0.35
    plating_um: float = 25.0


@dataclass
class CopperZone:
    zone_id: str
    net_name: str
    layer_id: str
    points_mm: List[Tuple[float, float]]
    is_keepout: bool = False


@dataclass
class DecapStage:
    name: str
    count: int
    c_uf: float
    esr_mohm: float
    esl_nh: float


@dataclass
class DesignRules:
    min_clearance_mm: float = 0.15
    min_trace_width_mm: float = 0.15
    min_via_drill_mm: float = 0.20
    min_annular_ring_mm: float = 0.12
    edge_clearance_mm: float = 0.25


class BoardProject:
    def __init__(self) -> None:
        self.name: str = "Mixed_Signal_Power_Board"
        self.width_mm: float = 80.0
        self.height_mm: float = 55.0
        self.rules: DesignRules = DesignRules()
        self.layers: List[CopperLayer] = []
        self.dielectrics: List[DielectricLayer] = []
        self.nets: Set[str] = {"GND", "VIN_12V", "VOUT_3V3", "SW_NODE", "CLK_100M", "DATA_0", "DATA_1"}
        self.components: Dict[str, Component] = {}
        self.sch_wires: Dict[str, SchematicWire] = {}
        self.traces: Dict[str, Trace] = {}
        self.vias: Dict[str, Via] = {}
        self.zones: Dict[str, CopperZone] = {}

        self.dc_sources: Dict[str, float] = {}
        self.dc_sinks: Dict[str, float] = {}
        self.ambient_temp_c: float = 25.0
        self.convection_h_w_m2k: float = 15.0
        self.decap_bank: List[DecapStage] = [
            DecapStage("Bulk_Polymer_47uF", 2, 47.0, 15.0, 1.8),
            DecapStage("MLCC_0805_4.7uF", 4, 4.7, 5.0, 0.85),
            DecapStage("MLCC_0402_100nF", 12, 0.1, 12.0, 0.45),
            DecapStage("MLCC_0201_10nF", 8, 0.01, 25.0, 0.30),
        ]

    def get_layer(self, lid: str) -> CopperLayer:
        for l in self.layers:
            if l.layer_id == lid:
                return l
        return self.layers[0]

    def all_world_pads(self) -> Dict[str, Tuple[Component, Pad, float, float]]:
        out: Dict[str, Tuple[Component, Pad, float, float]] = {}
        for comp in self.components.values():
            for pad in comp.pads:
                wx, wy = comp.pad_world_pos(pad)
                out[pad.pad_id] = (comp, pad, wx, wy)
        return out

    def sync_schematic_to_pcb(self) -> int:
        updated = 0
        for wire in self.sch_wires.values():
            self.nets.add(wire.net_name)
            for cid, pnum in ((wire.from_comp_id, wire.from_pin_number), (wire.to_comp_id, wire.to_pin_number)):
                if cid in self.components:
                    for pad in self.components[cid].pads:
                        if pad.pin_number == pnum and pad.net_name != wire.net_name:
                            pad.net_name = wire.net_name
                            updated += 1
        return updated

    def to_json(self) -> str:
        data = {
            "name": self.name,
            "width_mm": self.width_mm,
            "height_mm": self.height_mm,
            "rules": dataclasses.asdict(self.rules),
            "layers": [dataclasses.asdict(l) for l in self.layers],
            "dielectrics": [dataclasses.asdict(d) for d in self.dielectrics],
            "nets": sorted(list(self.nets)),
            "components": {k: dataclasses.asdict(v) for k, v in self.components.items()},
            "sch_wires": {k: dataclasses.asdict(w) for k, w in self.sch_wires.items()},
            "traces": {k: dataclasses.asdict(v) for k, v in self.traces.items()},
            "vias": {k: dataclasses.asdict(v) for k, v in self.vias.items()},
            "zones": {k: dataclasses.asdict(v) for k, v in self.zones.items()},
            "dc_sources": self.dc_sources,
            "dc_sinks": self.dc_sinks,
            "ambient_temp_c": self.ambient_temp_c,
            "convection_h_w_m2k": self.convection_h_w_m2k,
            "decap_bank": [dataclasses.asdict(s) for s in self.decap_bank],
        }
        return json.dumps(data, indent=2)

    @classmethod
    def from_json(cls, raw: str) -> "BoardProject":
        d = json.loads(raw)
        b = cls()
        b.name = d.get("name", "Project")
        b.width_mm = float(d.get("width_mm", 80.0))
        b.height_mm = float(d.get("height_mm", 55.0))
        b.rules = DesignRules(**d.get("rules", {}))
        b.layers = [CopperLayer(**l) for l in d.get("layers", [])]
        b.dielectrics = [DielectricLayer(**dl) for dl in d.get("dielectrics", [])]
        b.nets = set(d.get("nets", []))
        b.components = {}
        for k, cd in d.get("components", {}).items():
            pads = [Pad(**p) for p in cd.get("pads", [])]
            c_copy = dict(cd)
            c_copy["pads"] = pads
            b.components[k] = Component(**c_copy)
        b.sch_wires = {k: SchematicWire(**w) for k, w in d.get("sch_wires", {}).items()}
        b.traces = {k: Trace(**t) for k, t in d.get("traces", {}).items()}
        b.vias = {k: Via(**v) for k, v in d.get("vias", {}).items()}
        b.zones = {k: CopperZone(**z) for k, z in d.get("zones", {}).items()}
        b.dc_sources = {k: float(v) for k, v in d.get("dc_sources", {}).items()}
        b.dc_sinks = {k: float(v) for k, v in d.get("dc_sinks", {}).items()}
        b.ambient_temp_c = float(d.get("ambient_temp_c", 25.0))
        b.convection_h_w_m2k = float(d.get("convection_h_w_m2k", 15.0))
        if "decap_bank" in d:
            b.decap_bank = [DecapStage(**s) for s in d["decap_bank"]]
        return b


# ============================================================================
# 2. UNIVERSAL CAD FORMAT IMPORTERS (KICAD .kicad_pcb & DXF OUTLINE)
# ============================================================================

class CADFormatImporter:
    @staticmethod
    def parse_sexpr(text: str) -> List[Any]:
        token_pattern = re.compile(r'\(|\)|"[^"\\]*(?:\\.[^"\\]*)*"|[^\s()]+')
        tokens = token_pattern.findall(text)
        stack: List[List[Any]] = [[]]
        for token in tokens:
            if token == "(":
                new_list: List[Any] = []
                stack[-1].append(new_list)
                stack.append(new_list)
            elif token == ")":
                if len(stack) > 1:
                    stack.pop()
            else:
                if token.startswith('"') and token.endswith('"'):
                    val: Any = token[1:-1].replace('\\"', '"')
                else:
                    try:
                        val = int(token)
                    except ValueError:
                        try:
                            val = float(token)
                        except ValueError:
                            val = token
                stack[-1].append(val)
        return stack[0][0] if stack[0] else []

    @classmethod
    def import_kicad_pcb(cls, file_path: str) -> BoardProject:
        content = pathlib.Path(file_path).read_text(encoding="utf-8", errors="ignore")
        tree = cls.parse_sexpr(content)
        if not tree or tree[0] != "kicad_pcb":
            raise ValueError("File is not a valid KiCad PCB (.kicad_pcb) file.")

        b = BoardProject()
        b.name = pathlib.Path(file_path).stem
        b.components.clear()
        b.traces.clear()
        b.vias.clear()
        b.zones.clear()

        net_map: Dict[int, str] = {0: ""}
        for item in tree:
            if isinstance(item, list) and len(item) >= 3 and item[0] == "net":
                net_map[int(item[1])] = str(item[2])
                b.nets.add(str(item[2]))

        layer_remap = {"F.Cu": "L1", "B.Cu": "L4", "In1.Cu": "L2", "In2.Cu": "L3"}
        min_x, max_x = float("inf"), float("-inf")
        min_y, max_y = float("inf"), float("-inf")

        def update_bbox(x: float, y: float) -> None:
            nonlocal min_x, max_x, min_y, max_y
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)

        raw_segments: List[Tuple[float, float, float, float, float, str, str]] = []
        raw_vias: List[Tuple[float, float, float, float, str]] = []
        raw_footprints: List[Dict[str, Any]] = []

        for item in tree:
            if not isinstance(item, list) or not item:
                continue
            tag = item[0]

            if tag in ("gr_line", "gr_rect"):
                d = {sub[0]: sub[1:] for sub in item[1:] if isinstance(sub, list) and sub}
                layer_name = str(d.get("layer", [""])[0])
                if "Edge.Cuts" in layer_name:
                    if tag == "gr_line" and "start" in d and "end" in d:
                        update_bbox(float(d["start"][0]), float(d["start"][1]))
                        update_bbox(float(d["end"][0]), float(d["end"][1]))

            elif tag == "segment":
                d = {sub[0]: sub[1:] for sub in item[1:] if isinstance(sub, list) and sub}
                if "start" in d and "end" in d:
                    x1, y1 = float(d["start"][0]), float(d["start"][1])
                    x2, y2 = float(d["end"][0]), float(d["end"][1])
                    w = float(d.get("width", [0.25])[0])
                    lay = layer_remap.get(str(d.get("layer", ["F.Cu"])[0]), "L1")
                    net_id = int(d.get("net", [0])[0])
                    net_name = net_map.get(net_id, "")
                    update_bbox(x1, y1)
                    update_bbox(x2, y2)
                    raw_segments.append((x1, y1, x2, y2, w, lay, net_name))

            elif tag == "via":
                d = {sub[0]: sub[1:] for sub in item[1:] if isinstance(sub, list) and sub}
                if "at" in d:
                    vx, vy = float(d["at"][0]), float(d["at"][1])
                    size = float(d.get("size", [0.8])[0])
                    drill = float(d.get("drill", [0.4])[0])
                    net_id = int(d.get("net", [0])[0])
                    net_name = net_map.get(net_id, "")
                    update_bbox(vx, vy)
                    raw_vias.append((vx, vy, size, drill, net_name))

            elif tag in ("footprint", "module"):
                fp_info: Dict[str, Any] = {"pads": []}
                for sub in item[1:]:
                    if not isinstance(sub, list) or not sub:
                        continue
                    if sub[0] == "at":
                        fp_info["x"] = float(sub[1])
                        fp_info["y"] = float(sub[2])
                        fp_info["rot"] = float(sub[3]) if len(sub) > 3 else 0.0
                    elif sub[0] == "property" and len(sub) >= 3:
                        if sub[1] == "Reference":
                            fp_info["refdes"] = str(sub[2])
                        elif sub[1] == "Value":
                            fp_info["val"] = str(sub[2])
                    elif sub[0] == "fp_text" and len(sub) >= 3 and sub[1] == "reference":
                        fp_info["refdes"] = str(sub[2])
                    elif sub[0] == "pad" and len(sub) >= 5:
                        p_num = str(sub[1])
                        p_shape = "CIRCLE" if sub[3] == "circle" else "RECT"
                        pd = {elem[0]: elem[1:] for elem in sub[4:] if isinstance(elem, list) and elem}
                        px = float(pd.get("at", [0.0, 0.0])[0])
                        py = float(pd.get("at", [0.0, 0.0])[1])
                        pw = float(pd.get("size", [1.2, 1.2])[0])
                        ph = float(pd.get("size", [1.2, 1.2])[1])
                        p_net_id = int(pd.get("net", [0])[0])
                        p_net = net_map.get(p_net_id, "")
                        fp_info["pads"].append((p_num, px, py, pw, ph, p_shape, p_net))

                if "x" in fp_info:
                    update_bbox(fp_info["x"], fp_info["y"])
                    raw_footprints.append(fp_info)

        if min_x != float("inf") and min_y != float("inf"):
            ox = min_x - 5.0
            oy = min_y - 5.0
            b.width_mm = max(20.0, round(max_x - min_x + 10.0, 1))
            b.height_mm = max(20.0, round(max_y - min_y + 10.0, 1))
        else:
            ox, oy = 0.0, 0.0

        for x1, y1, x2, y2, w, lay, net in raw_segments:
            tid = f"TR_KC_{uuid.uuid4().hex[:6]}"
            b.traces[tid] = Trace(tid, net, lay, x1 - ox, y1 - oy, x2 - ox, y2 - oy, w)

        for vx, vy, size, drill, net in raw_vias:
            vid = f"VIA_KC_{uuid.uuid4().hex[:6]}"
            b.vias[vid] = Via(vid, net, vx - ox, vy - oy, size, drill)

        for fp in raw_footprints:
            ref = fp.get("refdes", f"U{len(b.components) + 1}")
            val = fp.get("val", "KICAD_PART")
            cid = f"COMP_{ref}"
            comp_pads: List[Pad] = []
            for pnum, px, py, pw, ph, shape, pnet in fp["pads"]:
                comp_pads.append(Pad(
                    f"{ref}_P{pnum}", pnum, f"P{pnum}", px, py, pw, ph,
                    "L1", shape=shape, net_name=pnet
                ))
            b.components[cid] = Component(
                comp_id=cid, refdes=ref, package_name="KiCad_Footprint", value=val,
                x_mm=fp["x"] - ox, y_mm=fp["y"] - oy, width_mm=max(3.0, len(comp_pads) * 0.8),
                height_mm=max(3.0, len(comp_pads) * 0.8), rotation_deg=fp.get("rot", 0.0),
                pads=comp_pads
            )

        return b

    @classmethod
    def import_dxf_outline(cls, file_path: str, board: BoardProject) -> int:
        lines = pathlib.Path(file_path).read_text(encoding="utf-8", errors="ignore").splitlines()
        min_x, max_x = float("inf"), float("-inf")
        min_y, max_y = float("inf"), float("-inf")

        i = 0
        n_lines = len(lines)
        pts_found: List[Tuple[float, float]] = []

        while i < n_lines - 1:
            code = lines[i].strip()
            val = lines[i + 1].strip()
            if code == "0" and val == "LINE":
                x1 = y1 = x2 = y2 = 0.0
                i += 2
                while i < n_lines - 1 and lines[i].strip() != "0":
                    c = lines[i].strip()
                    v = lines[i + 1].strip()
                    if c == "10": x1 = float(v)
                    elif c == "20": y1 = float(v)
                    elif c == "11": x2 = float(v)
                    elif c == "21": y2 = float(v)
                    i += 2
                pts_found.extend([(x1, y1), (x2, y2)])
                continue
            elif code == "0" and val == "LWPOLYLINE":
                i += 2
                while i < n_lines - 1 and lines[i].strip() != "0":
                    c = lines[i].strip()
                    v = lines[i + 1].strip()
                    if c == "10":
                        px = float(v)
                        py = float(lines[i + 3].strip()) if (i + 3 < n_lines and lines[i + 2].strip() == "20") else 0.0
                        pts_found.append((px, py))
                    i += 2
                continue
            i += 2

        if not pts_found:
            return 0

        for x, y in pts_found:
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)

        board.width_mm = max(10.0, round(max_x - min_x, 2))
        board.height_mm = max(10.0, round(max_y - min_y, 2))
        zid = f"ZONE_DXF_{uuid.uuid4().hex[:6]}"
        board.zones[zid] = CopperZone(
            zid, "GND", "L2",
            [(3.0, 3.0), (board.width_mm - 3.0, 3.0), (board.width_mm - 3.0, board.height_mm - 3.0), (3.0, board.height_mm - 3.0)],
            is_keepout=False
        )
        return len(pts_found)


# ============================================================================
# 3. SPICE TIME-DOMAIN TRANSIENT CIRCUIT SOLVER ENGINE
# ============================================================================

@dataclass
class SpiceElement:
    name: str
    elem_type: str
    node1: str
    node2: str
    value: float = 0.0
    wave_type: str = "DC"
    wave_params: Tuple[float, ...] = ()


@dataclass
class SpiceTransientResult:
    time_s: np.ndarray
    node_voltages: Dict[str, np.ndarray]
    branch_currents: Dict[str, np.ndarray]
    fft_freq_hz: np.ndarray
    fft_mag_db: np.ndarray
    summary_text: str


class SpiceTransientEngine:
    UNIT_SCALE: Dict[str, float] = {
        "f": 1e-15, "p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3,
        "k": 1e3, "meg": 1e6, "g": 1e9,
    }

    @classmethod
    def parse_val(cls, token: str) -> float:
        s = token.strip().lower()
        for suffix in ("meg", "f", "p", "n", "u", "m", "k", "g"):
            if s.endswith(suffix):
                return float(s[:-len(suffix)]) * cls.UNIT_SCALE[suffix]
        return float(s)

    @classmethod
    def parse_netlist(cls, netlist_text: str) -> List[SpiceElement]:
        elements: List[SpiceElement] = []
        for raw_line in netlist_text.splitlines():
            line = raw_line.split("*")[0].split(";")[0].strip()
            if not line or line.startswith("."):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            name = parts[0].upper()
            etype = name[0]
            n1, n2 = parts[1].upper(), parts[2].upper()

            if etype in ("R", "C", "L") and len(parts) >= 4:
                val = cls.parse_val(parts[3])
                elements.append(SpiceElement(name, etype, n1, n2, value=val))
            elif etype in ("V", "I"):
                rest = " ".join(parts[3:]).upper()
                if "PULSE" in rest:
                    inside = rest[rest.find("(") + 1:rest.rfind(")")].replace(",", " ").split()
                    vals = tuple(cls.parse_val(x) for x in inside)
                    elements.append(SpiceElement(name, etype, n1, n2, value=vals[0] if vals else 0.0, wave_type="PULSE", wave_params=vals))
                elif "SIN" in rest:
                    inside = rest[rest.find("(") + 1:rest.rfind(")")].replace(",", " ").split()
                    vals = tuple(cls.parse_val(x) for x in inside)
                    elements.append(SpiceElement(name, etype, n1, n2, value=vals[0] if vals else 0.0, wave_type="SIN", wave_params=vals))
                else:
                    val_tok = parts[-1].replace("DC", "").strip()
                    val = cls.parse_val(val_tok) if val_tok else 0.0
                    elements.append(SpiceElement(name, etype, n1, n2, value=val, wave_type="DC"))
            elif etype == "D":
                elements.append(SpiceElement(name, "D", n1, n2, value=1e-12))
        return elements

    @staticmethod
    def eval_source(el: SpiceElement, t: float) -> float:
        if el.wave_type == "PULSE" and len(el.wave_params) >= 7:
            v1, v2, td, tr, tf, pw, per = el.wave_params[:7]
            if t < td: return v1
            tau = (t - td) % max(1e-12, per)
            if tau < tr: return v1 + (v2 - v1) * (tau / max(1e-12, tr))
            elif tau < tr + pw: return v2
            elif tau < tr + pw + tf: return v2 - (v2 - v1) * ((tau - tr - pw) / max(1e-12, tf))
            return v1
        elif el.wave_type == "SIN" and len(el.wave_params) >= 3:
            vo, va, freq = el.wave_params[:3]
            return vo + va * math.sin(2.0 * math.pi * freq * t)
        return el.value

    @classmethod
    def simulate(
        cls, netlist_text: str, t_stop_s: float = 60e-6, dt_s: float = 0.1e-6, fft_node: str = ""
    ) -> SpiceTransientResult:
        elements = cls.parse_netlist(netlist_text)
        node_names: List[str] = ["0"]
        for el in elements:
            for n in (el.node1, el.node2):
                if n not in ("GND", "0") and n not in node_names:
                    node_names.append(n)

        def n_idx(n: str) -> int:
            return 0 if n in ("0", "GND") else node_names.index(n)

        num_v_nodes = len(node_names) - 1
        v_L_branches = [el for el in elements if el.elem_type in ("V", "L")]
        dim = num_v_nodes + len(v_L_branches)

        n_steps = max(10, min(8000, int(round(t_stop_s / dt_s)) + 1))
        t_vec = np.linspace(0.0, t_stop_s, n_steps)
        dt = t_vec[1] - t_vec[0]

        x_sol = np.zeros((n_steps, dim), dtype=np.float64)
        x_prev = np.zeros(dim, dtype=np.float64)

        def get_v(vec: np.ndarray, node: str) -> float:
            idx = n_idx(node)
            return 0.0 if idx == 0 else float(vec[idx - 1])

        has_diodes = any(el.elem_type == "D" for el in elements)
        max_nr_iter = 10 if has_diodes else 1

        for step_i, t in enumerate(t_vec):
            x_guess = x_prev.copy()
            for _nr in range(max_nr_iter):
                A = np.zeros((dim, dim), dtype=np.float64)
                rhs = np.zeros(dim, dtype=np.float64)
                for i in range(num_v_nodes):
                    A[i, i] = 1e-10

                def stamp_g(na: int, nb: int, g: float) -> None:
                    if na > 0: A[na - 1, na - 1] += g
                    if nb > 0: A[nb - 1, nb - 1] += g
                    if na > 0 and nb > 0:
                        A[na - 1, nb - 1] -= g
                        A[nb - 1, na - 1] -= g

                def stamp_i(na: int, nb: int, curr_a_to_b: float) -> None:
                    if na > 0: rhs[na - 1] -= curr_a_to_b
                    if nb > 0: rhs[nb - 1] += curr_a_to_b

                for el in elements:
                    na, nb = n_idx(el.node1), n_idx(el.node2)
                    if el.elem_type == "R":
                        stamp_g(na, nb, 1.0 / max(1e-6, el.value))
                    elif el.elem_type == "C":
                        if step_i == 0:
                            stamp_g(na, nb, 1e-9)
                        else:
                            g_eq = el.value / dt
                            v_prev_c = get_v(x_prev, el.node1) - get_v(x_prev, el.node2)
                            stamp_g(na, nb, g_eq)
                            stamp_i(na, nb, -g_eq * v_prev_c)
                    elif el.elem_type == "I":
                        stamp_i(na, nb, cls.eval_source(el, t))
                    elif el.elem_type == "D":
                        vd = get_v(x_guess, el.node1) - get_v(x_guess, el.node2)
                        vd_clamped = max(-5.0, min(0.85, vd))
                        vt, is_sat = 0.038, 1e-9
                        exp_v = math.exp(vd_clamped / vt)
                        id_val = is_sat * (exp_v - 1.0)
                        gd = (is_sat / vt) * exp_v + 1e-9
                        stamp_g(na, nb, gd)
                        stamp_i(na, nb, id_val - gd * vd_clamped)

                for b_idx, el in enumerate(v_L_branches):
                    row = num_v_nodes + b_idx
                    na, nb = n_idx(el.node1), n_idx(el.node2)
                    if na > 0:
                        A[row, na - 1] += 1.0
                        A[na - 1, row] += 1.0
                    if nb > 0:
                        A[row, nb - 1] -= 1.0
                        A[nb - 1, row] -= 1.0

                    if el.elem_type == "V":
                        rhs[row] = cls.eval_source(el, t)
                    elif el.elem_type == "L":
                        if step_i == 0:
                            A[row, row] -= 1e-3
                            rhs[row] = 0.0
                        else:
                            r_eq = el.value / dt
                            A[row, row] -= r_eq
                            rhs[row] = -r_eq * x_prev[row]

                x_new = np.linalg.solve(A, rhs)
                if np.max(np.abs(x_new - x_guess)) < 1e-5:
                    x_guess = x_new
                    break
                x_guess = x_new

            x_sol[step_i, :] = x_guess
            x_prev = x_guess

        node_voltages: Dict[str, np.ndarray] = {}
        for idx in range(1, len(node_names)):
            node_voltages[node_names[idx]] = x_sol[:, idx - 1]

        branch_currents: Dict[str, np.ndarray] = {}
        for b_idx, el in enumerate(v_L_branches):
            branch_currents[f"I({el.name})"] = x_sol[:, num_v_nodes + b_idx]

        target_node = fft_node if fft_node in node_voltages else (next(iter(node_voltages.keys()), ""))
        if target_node:
            sig = node_voltages[target_node]
            sig_ac = sig - np.mean(sig)
            win = np.hanning(len(sig_ac))
            spec = np.fft.rfft(sig_ac * win) / max(1, len(sig_ac) * 0.5)
            fft_freqs = np.fft.rfftfreq(len(sig_ac), d=dt)
            fft_mag_db = 20.0 * np.log10(np.maximum(1e-9, np.abs(spec)))
        else:
            fft_freqs = np.linspace(0, 1e6, 10)
            fft_mag_db = np.full(10, -120.0)

        lines = [
            "=== SPICE TRANSIENT SIMULATION REPORT ===",
            f"Time Step (dt) : {dt * 1e9:.2f} ns | Stop Time: {t_stop_s * 1e6:.2f} us ({n_steps} steps)",
            "",
            "Node Voltage Measurements (Steady-State Second Half):"
        ]
        half_idx = n_steps // 2
        for n_name, wave in node_voltages.items():
            w_ss = wave[half_idx:]
            lines.append(f"  V({n_name:10s}) : Avg = {np.mean(w_ss):7.3f} V | Vpp = {(np.max(w_ss) - np.min(w_ss)) * 1e3:6.2f} mV | Peak = {np.max(wave):7.3f} V")

        if branch_currents:
            lines.append("\nBranch Current Measurements:")
            for b_name, i_wave in branch_currents.items():
                lines.append(f"  {b_name:12s} : I_rms = {np.sqrt(np.mean(i_wave[half_idx:] ** 2)):7.3f} A | I_peak = {np.max(np.abs(i_wave)):7.3f} A")

        return SpiceTransientResult(
            time_s=t_vec,
            node_voltages=node_voltages,
            branch_currents=branch_currents,
            fft_freq_hz=fft_freqs,
            fft_mag_db=fft_mag_db,
            summary_text="\n".join(lines),
        )


SPICE_PRESET_BUCK = """* Preset 1: 12V to 3.3V Buck Converter Start-up & FPGA Load Step (U1 -> L1 -> C2 -> U2)
VSW SW_NODE 0 PULSE(0 12.0 0 15n 15n 0.56u 2.0u)
D1 0 SW_NODE D_SCHOTTKY
R_DCR SW_NODE L1_INT 0.035
L1 L1_INT VOUT_3V3 4.7u
R_ESR VOUT_3V3 C2_INT 0.008
C2 C2_INT 0 22u
R_LOAD VOUT_3V3 0 1.65
I_STEP VOUT_3V3 0 PULSE(0 1.5 45u 100n 100n 25u 100u)
"""

SPICE_PRESET_CLOCK_SI = """* Preset 2: 100 MHz High-Speed Clock Trace Ringing & ESD Clamping (CLK_100M)
VDRV DRV_OUT 0 PULSE(0 3.3 1n 0.35n 0.35n 4.65n 10n)
R_SRC DRV_OUT TRACE_IN 22.0
L_TRACE TRACE_IN CLK_100M 14n
R_TRACE CLK_100M 0 100k
C_LOAD CLK_100M 0 6.5p
VCC_3V3 VDD_3V3 0 DC 3.3
D_CLAMP_HI CLK_100M VDD_3V3 D_ESD
D_CLAMP_LO 0 CLK_100M D_ESD
"""


# ============================================================================
# 4. FOOTPRINT WIZARD & DEMO PROJECT BUILDER
# ============================================================================

class FootprintWizard:
    @staticmethod
    def create_qfn(
        refdes: str, value: str, x_mm: float, y_mm: float,
        body_mm: float = 7.0, pins_per_side: int = 4, pitch_mm: float = 0.8,
        power_w: float = 2.4, layer_id: str = "L1", sch_x: float = 520.0, sch_y: float = 220.0
    ) -> Component:
        cid = f"COMP_{refdes}"
        pads: List[Pad] = []
        total_pins = pins_per_side * 4
        span = (pins_per_side - 1) * pitch_mm
        half_b = body_mm / 2.0

        pin_idx = 1
        for i in range(pins_per_side):
            py = -span / 2.0 + i * pitch_mm
            pads.append(Pad(f"{refdes}_P{pin_idx}", str(pin_idx), f"IO{pin_idx}", -half_b, py, 0.8, 0.4, layer_id))
            pin_idx += 1
        for i in range(pins_per_side):
            px = -span / 2.0 + i * pitch_mm
            pads.append(Pad(f"{refdes}_P{pin_idx}", str(pin_idx), f"IO{pin_idx}", px, half_b, 0.4, 0.8, layer_id))
            pin_idx += 1
        for i in range(pins_per_side):
            py = span / 2.0 - i * pitch_mm
            pads.append(Pad(f"{refdes}_P{pin_idx}", str(pin_idx), f"IO{pin_idx}", half_b, py, 0.8, 0.4, layer_id))
            pin_idx += 1
        for i in range(pins_per_side):
            px = span / 2.0 - i * pitch_mm
            pads.append(Pad(f"{refdes}_P{pin_idx}", str(pin_idx), f"IO{pin_idx}", px, -half_b, 0.4, 0.8, layer_id))
            pin_idx += 1

        ep_size = round(body_mm * 0.55, 2)
        pads.append(Pad(
            f"{refdes}_EP", str(total_pins + 1), "EP_GND", 0.0, 0.0,
            ep_size, ep_size, layer_id, shape="RECT", net_name="GND", is_thermal_pad=True
        ))
        return Component(
            comp_id=cid, refdes=refdes, package_name=f"QFN-{total_pins}_EP", value=value,
            x_mm=x_mm, y_mm=y_mm, width_mm=body_mm, height_mm=body_mm,
            layer_id=layer_id, power_w=power_w, r_theta_jb=6.5, r_theta_ja=32.0,
            height_3d_mm=1.2, sch_x=sch_x, sch_y=sch_y, pads=pads
        )

    @staticmethod
    def create_soic(
        refdes: str, value: str, x_mm: float, y_mm: float,
        pin_count: int = 8, pitch_mm: float = 1.27, body_w_mm: float = 4.0,
        power_w: float = 1.2, layer_id: str = "L1", sch_x: float = 240.0, sch_y: float = 220.0
    ) -> Component:
        cid = f"COMP_{refdes}"
        half_n = max(1, pin_count // 2)
        span = (half_n - 1) * pitch_mm
        body_h = max(3.0, span + 1.8)
        pads: List[Pad] = []
        for i in range(half_n):
            py = -span / 2.0 + i * pitch_mm
            pnum_l = i + 1
            pads.append(Pad(f"{refdes}_P{pnum_l}", str(pnum_l), f"P{pnum_l}", -body_w_mm / 2.0, py, 1.4, 0.6, layer_id))
        for i in range(half_n):
            py = span / 2.0 - i * pitch_mm
            pnum_r = half_n + i + 1
            pads.append(Pad(f"{refdes}_P{pnum_r}", str(pnum_r), f"P{pnum_r}", body_w_mm / 2.0, py, 1.4, 0.6, layer_id))
        return Component(
            comp_id=cid, refdes=refdes, package_name=f"SOIC-{pin_count}", value=value,
            x_mm=x_mm, y_mm=y_mm, width_mm=body_w_mm, height_mm=body_h,
            layer_id=layer_id, power_w=power_w, r_theta_jb=11.0, r_theta_ja=48.0,
            height_3d_mm=1.75, sch_x=sch_x, sch_y=sch_y, pads=pads
        )

    @staticmethod
    def create_smd_2pin(
        refdes: str, value: str, x_mm: float, y_mm: float,
        pkg: str = "0805", net1: str = "", net2: str = "", power_w: float = 0.05,
        layer_id: str = "L1", sch_x: float = 100.0, sch_y: float = 100.0
    ) -> Component:
        dims = {
            "0402": (1.0, 0.5, 0.5, 0.5, 0.6, 0.5),
            "0603": (1.6, 0.8, 0.8, 0.8, 0.9, 0.8),
            "0805": (2.0, 1.25, 1.0, 1.25, 1.1, 1.1),
            "1206": (3.2, 1.6, 1.6, 1.6, 1.5, 1.3),
            "IND_6x6": (6.0, 6.0, 2.2, 2.0, 4.5, 4.2),
        }
        bw, bh, dx, pw, ph, h3d = dims.get(pkg, dims["0805"])
        pads = [
            Pad(f"{refdes}_P1", "1", "1", -dx, 0.0, pw, ph, layer_id, net_name=net1),
            Pad(f"{refdes}_P2", "2", "2", dx, 0.0, pw, ph, layer_id, net_name=net2),
        ]
        return Component(
            comp_id=f"COMP_{refdes}", refdes=refdes, package_name=pkg, value=value,
            x_mm=x_mm, y_mm=y_mm, width_mm=bw, height_mm=bh,
            layer_id=layer_id, power_w=power_w, r_theta_jb=18.0, r_theta_ja=65.0,
            height_3d_mm=h3d, sch_x=sch_x, sch_y=sch_y, pads=pads
        )


def build_default_demo_board() -> BoardProject:
    b = BoardProject()
    b.layers = [
        CopperLayer("L1", "Top_Signal_Power", 1, 35.0, 5.80e7, 0.00393, 385.0, "#e64a4a", True),
        CopperLayer("L2", "Inner_GND_Plane", 2, 35.0, 5.80e7, 0.00393, 385.0, "#38b866", True),
        CopperLayer("L3", "Inner_Power_Plane", 3, 35.0, 5.80e7, 0.00393, 385.0, "#e39b27", True),
        CopperLayer("L4", "Bottom_Signal", 4, 35.0, 5.80e7, 0.00393, 385.0, "#4692f0", True),
    ]
    b.dielectrics = [
        DielectricLayer("D1", "Prepreg_2116", 200.0, 4.2, 0.016, 0.35),
        DielectricLayer("D2", "FR4_Core_1.1mm", 1100.0, 4.5, 0.020, 0.30),
        DielectricLayer("D3", "Prepreg_2116", 200.0, 4.2, 0.016, 0.35),
    ]

    j1 = FootprintWizard.create_smd_2pin("J1", "12V_IN_CONN", 12.0, 26.0, "1206", "VIN_12V", "GND", 0.05, sch_x=90.0, sch_y=210.0)
    u1 = FootprintWizard.create_soic("U1", "TPS5430_BUCK", 28.0, 26.0, 8, 1.27, 4.0, power_w=1.85, sch_x=290.0, sch_y=210.0)
    u1.pads[0].net_name = "VIN_12V"
    u1.pads[1].net_name = "GND"
    u1.pads[4].net_name = "SW_NODE"
    u1.pads[7].net_name = "VOUT_3V3"

    l1 = FootprintWizard.create_smd_2pin("L1", "4.7uH_5A", 43.0, 26.0, "IND_6x6", "SW_NODE", "VOUT_3V3", 0.45, sch_x=480.0, sch_y=150.0)
    u2 = FootprintWizard.create_qfn("U2", "STM32H7_FPGA", 62.0, 26.0, 8.0, 4, 1.2, power_w=3.20, sch_x=690.0, sch_y=230.0)
    u2.pads[0].net_name = "VOUT_3V3"
    u2.pads[1].net_name = "DATA_0"
    u2.pads[2].net_name = "DATA_1"
    u2.pads[3].net_name = "CLK_100M"
    u2.pads[5].net_name = "GND"

    c1 = FootprintWizard.create_smd_2pin("C1", "10uF_25V", 20.0, 16.0, "0805", "VIN_12V", "GND", 0.01, sch_x=185.0, sch_y=360.0)
    c2 = FootprintWizard.create_smd_2pin("C2", "22uF_10V", 52.0, 16.0, "0805", "VOUT_3V3", "GND", 0.01, sch_x=545.0, sch_y=360.0)

    for comp in (j1, u1, l1, u2, c1, c2):
        b.components[comp.comp_id] = comp

    wires = [
        SchematicWire("W1", "COMP_J1", "1", "COMP_U1", "1", "VIN_12V"),
        SchematicWire("W2", "COMP_C1", "1", "COMP_U1", "1", "VIN_12V"),
        SchematicWire("W3", "COMP_U1", "5", "COMP_L1", "1", "SW_NODE"),
        SchematicWire("W4", "COMP_L1", "2", "COMP_U2", "1", "VOUT_3V3"),
        SchematicWire("W5", "COMP_C2", "1", "COMP_U2", "1", "VOUT_3V3"),
        SchematicWire("W6", "COMP_J1", "2", "COMP_U1", "2", "GND"),
        SchematicWire("W7", "COMP_C1", "2", "COMP_C2", "2", "GND"),
    ]
    for w in wires:
        b.sch_wires[w.wire_id] = w

    all_pads = b.all_world_pads()
    _, _, j1_x, j1_y = all_pads["J1_P1"]
    _, _, u1_p1_x, u1_p1_y = all_pads["U1_P1"]
    _, _, u1_p5_x, u1_p5_y = all_pads["U1_P5"]
    _, _, l1_p1_x, l1_p1_y = all_pads["L1_P1"]
    _, _, l1_p2_x, l1_p2_y = all_pads["L1_P2"]
    _, _, u2_p1_x, u2_p1_y = all_pads["U2_P1"]

    b.traces["TR_VIN_1"] = Trace("TR_VIN_1", "VIN_12V", "L1", j1_x, j1_y, u1_p1_x, j1_y, 0.8)
    b.traces["TR_VIN_2"] = Trace("TR_VIN_2", "VIN_12V", "L1", u1_p1_x, j1_y, u1_p1_x, u1_p1_y, 0.8)
    b.traces["TR_SW_1"] = Trace("TR_SW_1", "SW_NODE", "L1", u1_p5_x, u1_p5_y, l1_p1_x, l1_p1_y, 1.0)

    via1_x, via1_y = 49.5, 26.0
    via2_x, via2_y = 55.5, 24.2
    b.vias["VIA_VOUT_1"] = Via("VIA_VOUT_1", "VOUT_3V3", via1_x, via1_y, 0.8, 0.4)
    b.vias["VIA_VOUT_2"] = Via("VIA_VOUT_2", "VOUT_3V3", via2_x, via2_y, 0.8, 0.4)

    b.traces["TR_VOUT_1"] = Trace("TR_VOUT_1", "VOUT_3V3", "L1", l1_p2_x, l1_p2_y, via1_x, via1_y, 0.9)
    b.traces["TR_VOUT_L4"] = Trace("TR_VOUT_L4", "VOUT_3V3", "L4", via1_x, via1_y, via2_x, via2_y, 0.9)
    b.traces["TR_VOUT_2"] = Trace("TR_VOUT_2", "VOUT_3V3", "L1", via2_x, via2_y, u2_p1_x, u2_p1_y, 0.9)

    for dx in (-1.2, 1.2):
        for dy in (-1.2, 1.2):
            vid = f"VIA_TH_{dx}_{dy}"
            b.vias[vid] = Via(vid, "GND", 62.0 + dx, 26.0 + dy, 0.6, 0.3)

    b.zones["ZONE_GND_L2"] = CopperZone(
        "ZONE_GND_L2", "GND", "L2",
        [(3.0, 3.0), (77.0, 3.0), (77.0, 52.0), (3.0, 52.0)],
        is_keepout=False
    )

    b.dc_sources = {"J1_P1": 12.0, "L1_P1": 3.35, "J1_P2": 0.0, "U2_EP": 0.0}
    b.dc_sinks = {"U1_P1": 0.85, "U2_P1": 2.60}
    return b


# ============================================================================
# 5. DRC ENGINE & A* GRID AUTOROUTER
# ============================================================================

@dataclass
class DRCViolation:
    code: str
    message: str
    x_mm: float
    y_mm: float


class DRCEngine:
    @staticmethod
    def seg_to_seg_dist(
        ax1: float, ay1: float, ax2: float, ay2: float,
        bx1: float, by1: float, bx2: float, by2: float
    ) -> float:
        def pt_seg(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
            dx, dy = x2 - x1, y2 - y1
            lsq = dx * dx + dy * dy
            if lsq <= 1e-12:
                return math.hypot(px - x1, py - y1)
            t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / lsq))
            return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

        return min(
            pt_seg(ax1, ay1, bx1, by1, bx2, by2),
            pt_seg(ax2, ay2, bx1, by1, bx2, by2),
            pt_seg(bx1, by1, ax1, ay1, ax2, ay2),
            pt_seg(bx2, by2, ax1, ay1, ax2, ay2),
        )

    @classmethod
    def check_board(cls, board: BoardProject) -> List[DRCViolation]:
        violations: List[DRCViolation] = []
        rules = board.rules

        for tr in board.traces.values():
            if tr.width_mm < rules.min_trace_width_mm:
                violations.append(DRCViolation(
                    "MIN_WIDTH",
                    f"Trace {tr.trace_id} width {tr.width_mm:.2f} mm below min {rules.min_trace_width_mm:.2f} mm",
                    tr.x1_mm, tr.y1_mm
                ))
            for x, y in ((tr.x1_mm, tr.y1_mm), (tr.x2_mm, tr.y2_mm)):
                if (x < rules.edge_clearance_mm or y < rules.edge_clearance_mm or
                        x > board.width_mm - rules.edge_clearance_mm or
                        y > board.height_mm - rules.edge_clearance_mm):
                    violations.append(DRCViolation(
                        "EDGE_CLEARANCE", f"Trace {tr.trace_id} violates board edge clearance", x, y
                    ))

        for v in board.vias.values():
            if v.drill_dia_mm < rules.min_via_drill_mm:
                violations.append(DRCViolation(
                    "MIN_DRILL", f"Via {v.via_id} drill {v.drill_dia_mm:.2f} mm below min", v.x_mm, v.y_mm
                ))
            annular = 0.5 * (v.outer_dia_mm - v.drill_dia_mm)
            if annular < rules.min_annular_ring_mm:
                violations.append(DRCViolation(
                    "ANNULAR_RING", f"Via {v.via_id} annular ring {annular:.2f} mm below min", v.x_mm, v.y_mm
                ))

        t_list = list(board.traces.values())
        for i in range(len(t_list)):
            for j in range(i + 1, len(t_list)):
                t1, t2 = t_list[i], t_list[j]
                if t1.layer_id != t2.layer_id or t1.net_name == t2.net_name:
                    continue
                dist = cls.seg_to_seg_dist(
                    t1.x1_mm, t1.y1_mm, t1.x2_mm, t1.y2_mm,
                    t2.x1_mm, t2.y1_mm, t2.x2_mm, t2.y2_mm
                )
                clr = dist - 0.5 * (t1.width_mm + t2.width_mm)
                if clr < rules.min_clearance_mm:
                    violations.append(DRCViolation(
                        "TRACE_CLEARANCE",
                        f"Clearance {clr:.3f} mm between {t1.trace_id} ({t1.net_name}) & {t2.trace_id} ({t2.net_name})",
                        0.5 * (t1.x1_mm + t2.x1_mm), 0.5 * (t1.y1_mm + t2.y1_mm)
                    ))

        return violations


class AStarAutorouter:
    @staticmethod
    def compute_ratsnest(board: BoardProject) -> List[Tuple[str, float, float, float, float, str]]:
        net_points: Dict[str, List[Tuple[float, float, str]]] = {}
        for _cid, (_comp, pad, wx, wy) in board.all_world_pads().items():
            if pad.net_name and pad.net_name != "GND":
                net_points.setdefault(pad.net_name, []).append((wx, wy, pad.layer_id))

        airwires: List[Tuple[str, float, float, float, float, str]] = []
        for net_name, pts in net_points.items():
            if len(pts) < 2:
                continue
            for i in range(len(pts) - 1):
                x1, y1, l1 = pts[i]
                x2, y2, _l2 = pts[i + 1]
                connected = False
                for tr in board.traces.values():
                    if tr.net_name == net_name:
                        d1 = math.hypot(tr.x1_mm - x1, tr.y1_mm - y1) + math.hypot(tr.x2_mm - x2, tr.y2_mm - y2)
                        d2 = math.hypot(tr.x1_mm - x2, tr.y1_mm - y2) + math.hypot(tr.x2_mm - x1, tr.y2_mm - y1)
                        if min(d1, d2) < 1.5:
                            connected = True
                            break
                if not connected:
                    airwires.append((net_name, x1, y1, x2, y2, l1 if l1 != "ALL" else "L1"))
        return airwires

    @classmethod
    def autoroute_all(cls, board: BoardProject, grid_step_mm: float = 0.5, trace_width_mm: float = 0.30) -> int:
        airwires = cls.compute_ratsnest(board)
        nx = int(board.width_mm / grid_step_mm) + 1
        ny = int(board.height_mm / grid_step_mm) + 1
        routed_count = 0

        def to_grid(x: float, y: float) -> Tuple[int, int]:
            return (
                max(1, min(nx - 2, int(round(x / grid_step_mm)))),
                max(1, min(ny - 2, int(round(y / grid_step_mm)))),
            )

        for net_name, x1, y1, x2, y2, layer_id in airwires:
            sx, sy = to_grid(x1, y1)
            gx, gy = to_grid(x2, y2)

            blocked = np.zeros((nx, ny), dtype=bool)
            margin_cells = max(1, int(math.ceil((board.rules.min_clearance_mm + trace_width_mm) / grid_step_mm)))

            for _pid, (_comp, pad, wx, wy) in board.all_world_pads().items():
                if pad.net_name != net_name and (pad.layer_id == layer_id or pad.layer_id == "ALL"):
                    px, py = to_grid(wx, wy)
                    rx = max(1, int(math.ceil(0.5 * pad.width_mm / grid_step_mm)) + margin_cells)
                    ry = max(1, int(math.ceil(0.5 * pad.height_mm / grid_step_mm)) + margin_cells)
                    blocked[max(0, px - rx):min(nx, px + rx + 1), max(0, py - ry):min(ny, py + ry + 1)] = True

            for tr in board.traces.values():
                if tr.layer_id == layer_id and tr.net_name != net_name:
                    steps = max(2, int(tr.length_mm() / (0.5 * grid_step_mm)) + 1)
                    for s in range(steps + 1):
                        u = s / steps
                        tx = tr.x1_mm + u * (tr.x2_mm - tr.x1_mm)
                        ty = tr.y1_mm + u * (tr.y2_mm - tr.y1_mm)
                        cx, cy = to_grid(tx, ty)
                        blocked[
                            max(0, cx - margin_cells):min(nx, cx + margin_cells + 1),
                            max(0, cy - margin_cells):min(ny, cy + margin_cells + 1)
                        ] = True

            blocked[sx, sy] = False
            blocked[gx, gy] = False

            open_heap: List[Tuple[float, float, int, int, int, int]] = []
            heapq.heappush(open_heap, (math.hypot(gx - sx, gy - sy), 0.0, sx, sy, 0, 0))
            came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
            g_score: Dict[Tuple[int, int], float] = {(sx, sy): 0.0}
            dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

            found = False
            while open_heap:
                _f, g, cx, cy, pdx, pdy = heapq.heappop(open_heap)
                if (cx, cy) == (gx, gy):
                    found = True
                    break
                for dx, dy in dirs:
                    nx_c, ny_c = cx + dx, cy + dy
                    if 1 <= nx_c < nx - 1 and 1 <= ny_c < ny - 1 and not blocked[nx_c, ny_c]:
                        step_cost = 1.4142 if (dx != 0 and dy != 0) else 1.0
                        bend_penalty = 0.25 if (pdx != 0 or pdy != 0) and (dx != pdx or dy != pdy) else 0.0
                        ng = g + step_cost + bend_penalty
                        if ng < g_score.get((nx_c, ny_c), float("inf")):
                            g_score[(nx_c, ny_c)] = ng
                            came_from[(nx_c, ny_c)] = (cx, cy)
                            h = math.hypot(gx - nx_c, gy - ny_c)
                            heapq.heappush(open_heap, (ng + h, ng, nx_c, ny_c, dx, dy))

            if found:
                path: List[Tuple[int, int]] = [(gx, gy)]
                curr = (gx, gy)
                while curr in came_from:
                    curr = came_from[curr]
                    path.append(curr)
                path.reverse()

                simplified: List[Tuple[float, float]] = [(x1, y1)]
                for k in range(1, len(path) - 1):
                    d1 = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1])
                    d2 = (path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
                    if d1 != d2:
                        simplified.append((path[k][0] * grid_step_mm, path[k][1] * grid_step_mm))
                simplified.append((x2, y2))

                for k in range(len(simplified) - 1):
                    tid = f"AR_{net_name}_{uuid.uuid4().hex[:5]}"
                    board.traces[tid] = Trace(
                        tid, net_name, layer_id,
                        simplified[k][0], simplified[k][1],
                        simplified[k + 1][0], simplified[k + 1][1],
                        trace_width_mm
                    )
                routed_count += 1

        return routed_count


# ============================================================================
# 6. MULTI-PHYSICS SOLVERS (DC MNA, COUPLED 3D ELECTROTHERMAL FVM, SI, PDN)
# ============================================================================

@dataclass
class ElectrothermalResult:
    pad_voltages_v: Dict[str, float]
    trace_currents_a: Dict[str, float]
    trace_current_density_a_mm2: Dict[str, float]
    trace_losses_w: Dict[str, float]
    via_currents_a: Dict[str, float]
    via_losses_w: Dict[str, float]
    comp_tj_c: Dict[str, float]
    comp_tboard_c: Dict[str, float]
    grid_x_mm: np.ndarray
    grid_y_mm: np.ndarray
    top_temp_map_c: np.ndarray
    bot_temp_map_c: np.ndarray
    iterations: int
    max_delta_t_c: float
    total_joule_w: float
    total_comp_w: float


class PhysicsEngine:
    @staticmethod
    def solve_coupled_electrothermal(
        board: BoardProject, nx: int = 48, ny: int = 34, max_coupled_iter: int = 6
    ) -> ElectrothermalResult:
        gx = np.linspace(0.0, board.width_mm, nx)
        gy = np.linspace(0.0, board.height_mm, ny)
        dx_m = (board.width_mm / max(1, nx - 1)) * 1e-3
        dy_m = (board.height_mm / max(1, ny - 1)) * 1e-3
        cell_area_m2 = dx_m * dy_m

        top_temp = np.full((nx, ny), board.ambient_temp_c, dtype=np.float64)
        bot_temp = np.full((nx, ny), board.ambient_temp_c, dtype=np.float64)

        pad_world = board.all_world_pads()
        total_board_thick_m = sum(d.thickness_um for d in board.dielectrics) * 1e-6
        if total_board_thick_m <= 0:
            total_board_thick_m = 1.5e-3

        pad_voltages: Dict[str, float] = {}
        trace_currents: Dict[str, float] = {}
        trace_j_dens: Dict[str, float] = {}
        trace_losses: Dict[str, float] = {}
        via_currents: Dict[str, float] = {}
        via_losses: Dict[str, float] = {}
        max_dt = 0.0

        for it in range(max_coupled_iter):
            nodes: Dict[Tuple[str, int, int], int] = {}
            node_coords: List[Tuple[str, float, float]] = []

            def get_node(lid: str, x_mm: float, y_mm: float) -> int:
                key = (lid, int(round(x_mm * 20)), int(round(y_mm * 20)))
                if key not in nodes:
                    nodes[key] = len(node_coords)
                    node_coords.append((lid, x_mm, y_mm))
                return nodes[key]

            pad_node_map: Dict[str, int] = {}
            for pid, (_c, pad, wx, wy) in pad_world.items():
                lid = "L1" if pad.layer_id == "ALL" else pad.layer_id
                pad_node_map[pid] = get_node(lid, wx, wy)

            tr_edges: List[Tuple[str, int, int, float, float]] = []
            for tr in board.traces.values():
                lay = board.get_layer(tr.layer_id)
                n1 = get_node(tr.layer_id, tr.x1_mm, tr.y1_mm)
                n2 = get_node(tr.layer_id, tr.x2_mm, tr.y2_mm)
                mx = 0.5 * (tr.x1_mm + tr.x2_mm)
                my = 0.5 * (tr.y1_mm + tr.y2_mm)
                ix = min(nx - 1, max(0, int(mx / board.width_mm * (nx - 1))))
                iy = min(ny - 1, max(0, int(my / board.height_mm * (ny - 1))))
                t_local = float(top_temp[ix, iy]) if lay.display_number <= 2 else float(bot_temp[ix, iy])

                area_m2 = max(1e-12, (tr.width_mm * 1e-3) * (lay.thickness_um * 1e-6))
                rho_t = (1.0 / lay.conductivity_s_m) * (1.0 + lay.tempco_per_k * (t_local - 20.0))
                r_ohm = max(1e-6, rho_t * (tr.length_mm() * 1e-3) / area_m2)
                tr_edges.append((tr.trace_id, n1, n2, r_ohm, tr.width_mm * (lay.thickness_um * 1e-3)))

            via_edges: List[Tuple[str, int, int, float]] = []
            for v in board.vias.values():
                n1 = get_node("L1", v.x_mm, v.y_mm)
                n2 = get_node(board.layers[-1].layer_id, v.x_mm, v.y_mm)
                r_out = 0.5 * v.drill_dia_mm * 1e-3
                r_in = max(0.0, r_out - v.plating_um * 1e-6)
                area_v = max(1e-12, math.pi * (r_out * r_out - r_in * r_in))
                r_via = max(1e-6, (1.0 / 5.8e7) * total_board_thick_m / area_v)
                via_edges.append((v.via_id, n1, n2, r_via))

            n_total = max(1, len(node_coords))
            G = np.zeros((n_total, n_total), dtype=np.float64)
            rhs = np.zeros(n_total, dtype=np.float64)
            for i in range(n_total):
                G[i, i] = 1e-9

            for _tid, n1, n2, r_ohm, _area in tr_edges:
                g = 1.0 / r_ohm
                G[n1, n1] += g
                G[n2, n2] += g
                G[n1, n2] -= g
                G[n2, n1] -= g

            for _vid, n1, n2, r_via in via_edges:
                g = 1.0 / r_via
                G[n1, n1] += g
                G[n2, n2] += g
                G[n1, n2] -= g
                G[n2, n1] -= g

            for pid, sink_a in board.dc_sinks.items():
                if pid in pad_node_map:
                    rhs[pad_node_map[pid]] -= sink_a

            for pid, src_v in board.dc_sources.items():
                if pid in pad_node_map:
                    idx = pad_node_map[pid]
                    G[idx, :] = 0.0
                    G[idx, idx] = 1.0
                    rhs[idx] = src_v

            v_sol = np.linalg.solve(G, rhs)
            pad_voltages = {pid: float(v_sol[idx]) for pid, idx in pad_node_map.items()}

            trace_currents.clear()
            trace_j_dens.clear()
            trace_losses.clear()
            for tid, n1, n2, r_ohm, area_mm2 in tr_edges:
                curr = abs(float(v_sol[n1] - v_sol[n2])) / r_ohm
                trace_currents[tid] = curr
                trace_j_dens[tid] = curr / max(1e-6, area_mm2)
                trace_losses[tid] = curr * curr * r_ohm

            via_currents.clear()
            via_losses.clear()
            for vid, n1, n2, r_via in via_edges:
                curr = abs(float(v_sol[n1] - v_sol[n2])) / r_via
                via_currents[vid] = curr
                via_losses[vid] = curr * curr * r_via

            N = nx * ny
            rows: List[int] = []
            cols: List[int] = []
            vals: List[float] = []
            q_vec = np.zeros(2 * N, dtype=np.float64)

            t_cu_m = (board.layers[0].thickness_um if board.layers else 35.0) * 1e-6
            k_inplane = 385.0 * 0.35 + 0.32 * 0.65
            gx_cond = k_inplane * (t_cu_m + 0.5 * total_board_thick_m) * dy_m / dx_m
            gy_cond = k_inplane * (t_cu_m + 0.5 * total_board_thick_m) * dx_m / dy_m
            gz_diel = (0.32 * cell_area_m2) / total_board_thick_m
            g_conv = board.convection_h_w_m2k * cell_area_m2

            via_g_boost = np.zeros((nx, ny), dtype=np.float64)
            for v in board.vias.values():
                ix = min(nx - 1, max(0, int(v.x_mm / board.width_mm * (nx - 1))))
                iy = min(ny - 1, max(0, int(v.y_mm / board.height_mm * (ny - 1))))
                r_out = 0.5 * v.drill_dia_mm * 1e-3
                r_in = max(0.0, r_out - v.plating_um * 1e-6)
                area_v = math.pi * (r_out * r_out - r_in * r_in)
                via_g_boost[ix, iy] += (385.0 * area_v) / total_board_thick_m

            q_top_grid = np.zeros((nx, ny), dtype=np.float64)
            q_bot_grid = np.zeros((nx, ny), dtype=np.float64)

            for comp in board.components.values():
                if comp.power_w <= 0: continue
                x0 = max(0, int((comp.x_mm - 0.5 * comp.width_mm) / board.width_mm * (nx - 1)))
                x1 = min(nx - 1, int((comp.x_mm + 0.5 * comp.width_mm) / board.width_mm * (nx - 1)))
                y0 = max(0, int((comp.y_mm - 0.5 * comp.height_mm) / board.height_mm * (ny - 1)))
                y1 = min(ny - 1, int((comp.y_mm + 0.5 * comp.height_mm) / board.height_mm * (ny - 1)))
                n_cells = max(1, (x1 - x0 + 1) * (y1 - y0 + 1))
                f_board = comp.r_theta_ja / (comp.r_theta_ja + comp.r_theta_jb)
                p_cell = (comp.power_w * f_board) / n_cells
                q_top_grid[x0:x1 + 1, y0:y1 + 1] += p_cell

            for tid, loss_w in trace_losses.items():
                tr = board.traces[tid]
                mx = 0.5 * (tr.x1_mm + tr.x2_mm)
                my = 0.5 * (tr.y1_mm + tr.y2_mm)
                ix = min(nx - 1, max(0, int(mx / board.width_mm * (nx - 1))))
                iy = min(ny - 1, max(0, int(my / board.height_mm * (ny - 1))))
                q_top_grid[ix, iy] += loss_w

            for ix in range(nx):
                for iy in range(ny):
                    idx_t = ix * ny + iy
                    idx_b = N + idx_t

                    diag_t = g_conv + gz_diel + via_g_boost[ix, iy]
                    diag_b = g_conv + gz_diel + via_g_boost[ix, iy]

                    g_vert = gz_diel + via_g_boost[ix, iy]
                    rows.extend([idx_t, idx_b])
                    cols.extend([idx_b, idx_t])
                    vals.extend([-g_vert, -g_vert])

                    for nix, niy, g_lat in (
                        (ix - 1, iy, gx_cond), (ix + 1, iy, gx_cond),
                        (ix, iy - 1, gy_cond), (ix, iy + 1, gy_cond)
                    ):
                        if 0 <= nix < nx and 0 <= niy < ny:
                            n_t = nix * ny + niy
                            n_b = N + n_t
                            diag_t += g_lat
                            diag_b += g_lat
                            rows.extend([idx_t, idx_b])
                            cols.extend([n_t, n_b])
                            vals.extend([-g_lat, -g_lat])

                    rows.extend([idx_t, idx_b])
                    cols.extend([idx_t, idx_b])
                    vals.extend([diag_t, diag_b])

                    q_vec[idx_t] = g_conv * board.ambient_temp_c + q_top_grid[ix, iy]
                    q_vec[idx_b] = g_conv * board.ambient_temp_c + q_bot_grid[ix, iy]

            K_mat = sp.csr_matrix((vals, (rows, cols)), shape=(2 * N, 2 * N))
            t_vec = spla.spsolve(K_mat, q_vec)

            new_top = t_vec[:N].reshape((nx, ny))
            new_bot = t_vec[N:].reshape((nx, ny))
            max_dt = float(np.max(np.abs(new_top - top_temp)))
            top_temp = new_top
            bot_temp = new_bot
            if max_dt < 1e-3 and it >= 1:
                break

        comp_tj: Dict[str, float] = {}
        comp_tb: Dict[str, float] = {}
        for comp in board.components.values():
            ix = min(nx - 1, max(0, int(comp.x_mm / board.width_mm * (nx - 1))))
            iy = min(ny - 1, max(0, int(comp.y_mm / board.height_mm * (ny - 1))))
            tb = float(top_temp[ix, iy])
            comp_tb[comp.refdes] = tb
            f_board = comp.r_theta_ja / (comp.r_theta_ja + comp.r_theta_jb)
            comp_tj[comp.refdes] = tb + comp.power_w * f_board * comp.r_theta_jb

        return ElectrothermalResult(
            pad_voltages_v=pad_voltages,
            trace_currents_a=trace_currents,
            trace_current_density_a_mm2=trace_j_dens,
            trace_losses_w=trace_losses,
            via_currents_a=via_currents,
            via_losses_w=via_losses,
            comp_tj_c=comp_tj,
            comp_tboard_c=comp_tb,
            grid_x_mm=gx,
            grid_y_mm=gy,
            top_temp_map_c=top_temp,
            bot_temp_map_c=bot_temp,
            iterations=it + 1,
            max_delta_t_c=max_dt,
            total_joule_w=sum(trace_losses.values()) + sum(via_losses.values()),
            total_comp_w=sum(c.power_w for c in board.components.values()),
        )

    @staticmethod
    def compute_transmission_line(
        mode: str, w_mm: float, s_mm: float, h_um: float, t_um: float, er: float, tan_d: float, length_mm: float
    ) -> Dict[str, Any]:
        h_mm = max(0.01, h_um * 1e-3)
        t_mm = max(0.001, t_um * 1e-3)
        w_eff = w_mm + (t_mm / math.pi) * math.log(1.0 + (4.0 * math.e * h_mm) / max(1e-4, t_mm))
        u = w_eff / h_mm

        if mode == "Stripline":
            b_mm = 2.0 * h_mm + t_mm
            er_eff = er
            z0 = (60.0 / math.sqrt(er)) * math.log((1.9 * b_mm) / (0.8 * w_eff + t_mm))
        else:
            er_eff = 0.5 * (er + 1.0) + 0.5 * (er - 1.0) * ((1.0 + 12.0 / u) ** -0.5)
            if u <= 1.0:
                z0 = (60.0 / math.sqrt(er_eff)) * math.log(8.0 / u + 0.25 * u)
            else:
                z0 = (120.0 * math.pi) / (math.sqrt(er_eff) * (u + 1.393 + 0.667 * math.log(u + 1.444)))

        z0 = max(5.0, z0)
        k_coup = 1.0 / (1.0 + (s_mm / h_mm) ** 1.85)
        z_odd = z0 * (1.0 - 0.48 * math.exp(-0.96 * (s_mm / h_mm)))
        z_even = z0 * (1.0 + 0.48 * math.exp(-0.96 * (s_mm / h_mm)))
        z_diff = 2.0 * z_odd
        next_db = 20.0 * math.log10(max(1e-6, 0.5 * k_coup))

        c0 = 299.792458
        v_prop_mm_ns = c0 / math.sqrt(er_eff)
        delay_ps = (length_mm / v_prop_mm_ns) * 1000.0

        freqs_ghz = np.linspace(0.05, 20.0, 200)
        r_skin_per_m = np.sqrt(np.pi * freqs_ghz * 1e9 * 4e-7 * np.pi / 5.8e7) / (w_mm * 1e-3)
        alpha_c_db_m = 8.686 * r_skin_per_m / (2.0 * z0)
        alpha_d_db_m = 27.3 * (freqs_ghz * 1e9 / 2.9979e8) * math.sqrt(er_eff) * tan_d
        s21_db = -((alpha_c_db_m + alpha_d_db_m) * (length_mm * 1e-3))

        return {
            "z0_ohm": z0,
            "z_diff_ohm": z_diff,
            "z_odd_ohm": z_odd,
            "z_even_ohm": z_even,
            "er_eff": er_eff,
            "delay_ps": delay_ps,
            "next_db": next_db,
            "freqs_ghz": freqs_ghz,
            "s21_db": s21_db,
        }


# ============================================================================
# 7. FABRICATION CAM EXPORTER (GERBER RS-274X, EXCELLON, BOM, CPL)
# ============================================================================

class FabricationCAMExporter:
    @staticmethod
    def _mm_to_46(val_mm: float) -> str:
        return str(int(round(val_mm * 1_000_000)))

    @classmethod
    def export_all(cls, board: BoardProject, out_dir: str) -> List[str]:
        p = pathlib.Path(out_dir)
        p.mkdir(parents=True, exist_ok=True)
        written: List[str] = []

        for lay in board.layers:
            lines = [
                f"G04 PCB Master Suite - {lay.name}*",
                "%FSLAX46Y46*%",
                "%MOMM*%",
                "%LPD*%",
            ]
            dcode_map: Dict[Tuple[str, int, int], int] = {}
            next_d = 10

            def get_d(shape: str, w_mm: float, h_mm: float = 0.0) -> int:
                nonlocal next_d
                key = (shape, int(round(w_mm * 1000)), int(round(h_mm * 1000)))
                if key not in dcode_map:
                    dcode_map[key] = next_d
                    next_d += 1
                return dcode_map[key]

            for comp in board.components.values():
                for pad in comp.pads:
                    if pad.layer_id in (lay.layer_id, "ALL"):
                        get_d("C" if pad.shape == "CIRCLE" else "R", pad.width_mm, pad.height_mm)
            for v in board.vias.values():
                get_d("C", v.outer_dia_mm)
            for tr in board.traces.values():
                if tr.layer_id == lay.layer_id:
                    get_d("C", tr.width_mm)

            for (shape, w_um, h_um), dc in dcode_map.items():
                if shape == "C":
                    lines.append(f"%ADD{dc}C,{w_um / 1000.0:.4f}*")
                else:
                    lines.append(f"%ADD{dc}R,{w_um / 1000.0:.4f}X{h_um / 1000.0:.4f}*")

            for comp in board.components.values():
                for pad in comp.pads:
                    if pad.layer_id in (lay.layer_id, "ALL"):
                        wx, wy = comp.pad_world_pos(pad)
                        dc = get_d("C" if pad.shape == "CIRCLE" else "R", pad.width_mm, pad.height_mm)
                        lines.append(f"D{dc}*")
                        lines.append(f"X{cls._mm_to_46(wx)}Y{cls._mm_to_46(wy)}D03*")

            for v in board.vias.values():
                dc = get_d("C", v.outer_dia_mm)
                lines.append(f"D{dc}*")
                lines.append(f"X{cls._mm_to_46(v.x_mm)}Y{cls._mm_to_46(v.y_mm)}D03*")

            for tr in board.traces.values():
                if tr.layer_id == lay.layer_id:
                    dc = get_d("C", tr.width_mm)
                    lines.append(f"D{dc}*")
                    lines.append("G01*")
                    lines.append(f"X{cls._mm_to_46(tr.x1_mm)}Y{cls._mm_to_46(tr.y1_mm)}D02*")
                    lines.append(f"X{cls._mm_to_46(tr.x2_mm)}Y{cls._mm_to_46(tr.y2_mm)}D01*")

            lines.append("M02*")
            fn = p / f"{board.name}_{lay.layer_id}_{lay.name}.gbr"
            fn.write_text("\n".join(lines) + "\n", encoding="utf-8")
            written.append(str(fn))

        w46, h46 = cls._mm_to_46(board.width_mm), cls._mm_to_46(board.height_mm)
        gko_lines = [
            "G04 Board Outline*",
            "%FSLAX46Y46*%",
            "%MOMM*%",
            "%LPD*%",
            "%ADD10C,0.1500*%",
            "D10*",
            "G01*",
            "X0Y0D02*",
            f"X{w46}Y0D01*",
            f"X{w46}Y{h46}D01*",
            f"X0Y{h46}D01*",
            "X0Y0D01*",
            "M02*",
        ]
        gko_path = p / f"{board.name}_Outline.GKO"
        gko_path.write_text("\n".join(gko_lines) + "\n", encoding="utf-8")
        written.append(str(gko_path))

        drl_lines = ["M48", "METRIC,TZ"]
        tools: Dict[int, int] = {}
        for v in board.vias.values():
            k = int(round(v.drill_dia_mm * 1000))
            if k not in tools:
                tools[k] = len(tools) + 1
        for k_um, tid in tools.items():
            drl_lines.append(f"T{tid:02d}C{k_um / 1000.0:.3f}")
        drl_lines.extend(["%", "G90", "G05"])
        for v in board.vias.values():
            tid = tools[int(round(v.drill_dia_mm * 1000))]
            drl_lines.append(f"T{tid:02d}")
            drl_lines.append(f"X{v.x_mm:.4f}Y{v.y_mm:.4f}")
        drl_lines.append("M30")
        drl_path = p / f"{board.name}_PTH_Drill.drl"
        drl_path.write_text("\n".join(drl_lines) + "\n", encoding="utf-8")
        written.append(str(drl_path))

        bom_path = p / f"{board.name}_BOM.csv"
        with bom_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Designator", "Footprint", "Value", "Layer", "Power_W"])
            for c in board.components.values():
                writer.writerow([c.refdes, c.package_name, c.value, c.layer_id, f"{c.power_w:.2f}"])
        written.append(str(bom_path))

        cpl_path = p / f"{board.name}_PickAndPlace_CPL.csv"
        with cpl_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["Designator", "Mid X (mm)", "Mid Y (mm)", "Layer", "Rotation"])
            for c in board.components.values():
                writer.writerow([c.refdes, f"{c.x_mm:.3f}", f"{c.y_mm:.3f}", c.layer_id, f"{c.rotation_deg:.1f}"])
        written.append(str(cpl_path))

        return written


# ============================================================================
# 8. INTERACTIVE SCHEMATIC CAPTURE WIDGET (TAB 1)
# ============================================================================

class SchematicCanvasWidget(QtWidgets.QWidget):
    statusChanged = QtCore.pyqtSignal(str)
    schematicModified = QtCore.pyqtSignal()

    def __init__(self, board: BoardProject, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.board = board
        self.setMouseTracking(True)
        self.tool: str = "SELECT"
        self.active_net: str = "VOUT_3V3"
        self.zoom: float = 1.0
        self.pan_x: float = 20.0
        self.pan_y: float = 20.0

        self.selected_comp_id: Optional[str] = None
        self.dragging_comp: bool = False
        self.panning: bool = False
        self.last_mouse: QPointF = QPointF()
        self.wire_start_pin: Optional[Tuple[str, str, QPointF]] = None
        self.cursor_scene: QPointF = QPointF(0, 0)

    def scene_to_screen(self, pt: QPointF) -> QPointF:
        return QPointF(self.pan_x + pt.x() * self.zoom, self.pan_y + pt.y() * self.zoom)

    def screen_to_scene(self, pt: QPointF) -> QPointF:
        return QPointF((pt.x() - self.pan_x) / self.zoom, (pt.y() - self.pan_y) / self.zoom)

    def symbol_box_and_pins(self, comp: Component) -> Tuple[QRectF, Dict[str, Tuple[QPointF, QPointF, str]]]:
        n_pads = len(comp.pads)
        left_count = (n_pads + 1) // 2
        right_count = n_pads - left_count
        box_w = 110.0
        box_h = max(64.0, max(left_count, right_count) * 24.0 + 20.0)
        rect = QRectF(comp.sch_x - box_w / 2.0, comp.sch_y - box_h / 2.0, box_w, box_h)

        pin_map: Dict[str, Tuple[QPointF, QPointF, str]] = {}
        for idx, pad in enumerate(comp.pads):
            if idx < left_count:
                py = rect.top() + 20.0 + idx * 24.0
                p_edge = QPointF(rect.left(), py)
                p_tip = QPointF(rect.left() - 22.0, py)
            else:
                r_idx = idx - left_count
                py = rect.top() + 20.0 + r_idx * 24.0
                p_edge = QPointF(rect.right(), py)
                p_tip = QPointF(rect.right() + 22.0, py)
            pin_map[pad.pin_number] = (p_edge, p_tip, pad.net_name)
        return rect, pin_map

    def find_pin_near(self, scene_pt: QPointF, tol: float = 14.0) -> Optional[Tuple[str, str, QPointF]]:
        for cid, comp in self.board.components.items():
            _rect, pmap = self.symbol_box_and_pins(comp)
            for pnum, (_edge, tip, _net) in pmap.items():
                if math.hypot(scene_pt.x() - tip.x(), scene_pt.y() - tip.y()) <= tol:
                    return (cid, pnum, tip)
        return None

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.12 if event.angleDelta().y() > 0 else 1.0 / 1.12
        self.zoom = max(0.4, min(3.0, self.zoom * factor))
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        pos = mouse_pos_f(event)
        if event.button() == MiddleButton or (event.button() == LeftButton and event.modifiers() & AltModifier):
            self.panning = True
            self.last_mouse = pos
            return

        sc = self.screen_to_scene(pos)
        if event.button() == RightButton:
            self.wire_start_pin = None
            self.selected_comp_id = None
            self.update()
            return

        if self.tool == "SELECT":
            self.selected_comp_id = None
            for cid, comp in self.board.components.items():
                rect, _ = self.symbol_box_and_pins(comp)
                if rect.contains(sc):
                    self.selected_comp_id = cid
                    self.dragging_comp = True
                    break
            self.update()

        elif self.tool == "WIRE":
            hit = self.find_pin_near(sc)
            if hit:
                cid, pnum, tip = hit
                if self.wire_start_pin is None:
                    self.wire_start_pin = (cid, pnum, tip)
                else:
                    scid, spnum, _stip = self.wire_start_pin
                    if (scid, spnum) != (cid, pnum):
                        net = self.active_net
                        for pad in self.board.components[scid].pads:
                            if pad.pin_number == spnum and pad.net_name:
                                net = pad.net_name
                        wid = f"W_{uuid.uuid4().hex[:6]}"
                        self.board.sch_wires[wid] = SchematicWire(wid, scid, spnum, cid, pnum, net)
                        self.board.sync_schematic_to_pcb()
                        self.schematicModified.emit()
                    self.wire_start_pin = None
            self.update()

        elif self.tool == "NET_LABEL":
            hit = self.find_pin_near(sc)
            if hit:
                cid, pnum, _tip = hit
                for pad in self.board.components[cid].pads:
                    if pad.pin_number == pnum:
                        pad.net_name = self.active_net
                        self.board.nets.add(self.active_net)
                        self.schematicModified.emit()
                        break
            self.update()

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        pos = mouse_pos_f(event)
        if self.panning:
            d = pos - self.last_mouse
            self.pan_x += d.x()
            self.pan_y += d.y()
            self.last_mouse = pos
            self.update()
            return

        sc = self.screen_to_scene(pos)
        self.cursor_scene = sc
        if self.dragging_comp and self.selected_comp_id:
            comp = self.board.components[self.selected_comp_id]
            comp.sch_x = round(sc.x() / 10.0) * 10.0
            comp.sch_y = round(sc.y() / 10.0) * 10.0
            self.schematicModified.emit()

        self.statusChanged.emit(f"Schematic Cursor: ({sc.x():.0f}, {sc.y():.0f}) | Tool: {self.tool} | Active Net: {self.active_net}")
        self.update()

    def mouseReleaseEvent(self, _event: QtGui.QMouseEvent) -> None:
        self.panning = False
        self.dragging_comp = False

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing if QT_VERSION == 6 else QtGui.QPainter.Antialiasing, True)
        qp.fillRect(self.rect(), QtGui.QColor("#10151c"))

        qp.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 18), 1.0))
        step = 20.0 * self.zoom
        if step >= 8.0:
            x = self.pan_x % step
            while x < self.width():
                qp.drawLine(QPointF(x, 0), QPointF(x, self.height()))
                x += step
            y = self.pan_y % step
            while y < self.height():
                qp.drawLine(QPointF(0, y), QPointF(self.width(), y))
                y += step

        pin_lookup: Dict[Tuple[str, str], QPointF] = {}
        for cid, comp in self.board.components.items():
            _r, pmap = self.symbol_box_and_pins(comp)
            for pnum, (_edge, tip, _net) in pmap.items():
                pin_lookup[(cid, pnum)] = tip

        qp.setPen(QtGui.QPen(QtGui.QColor("#3fb950"), max(1.6, 2.2 * self.zoom)))
        font_net = QtGui.QFont("Consolas", max(7, int(8 * self.zoom)), QtGui.QFont.Weight.Bold if QT_VERSION == 6 else QtGui.QFont.Bold)
        qp.setFont(font_net)
        for wire in self.board.sch_wires.values():
            p1 = pin_lookup.get((wire.from_comp_id, wire.from_pin_number))
            p2 = pin_lookup.get((wire.to_comp_id, wire.to_pin_number))
            if p1 and p2:
                mid_x = 0.5 * (p1.x() + p2.x())
                s1 = self.scene_to_screen(p1)
                sm1 = self.scene_to_screen(QPointF(mid_x, p1.y()))
                sm2 = self.scene_to_screen(QPointF(mid_x, p2.y()))
                s2 = self.scene_to_screen(p2)
                qp.drawLine(s1, sm1)
                qp.drawLine(sm1, sm2)
                qp.drawLine(sm2, s2)
                qp.setPen(QtGui.QColor("#a5d6ff"))
                qp.drawText(QPointF(sm1.x() + 4, 0.5 * (sm1.y() + sm2.y())), wire.net_name)
                qp.setPen(QtGui.QPen(QtGui.QColor("#3fb950"), max(1.6, 2.2 * self.zoom)))

        if self.tool == "WIRE" and self.wire_start_pin is not None:
            _cid, _pnum, tip = self.wire_start_pin
            qp.setPen(QtGui.QPen(QtGui.QColor("#ffea47"), 2.0, DashLine))
            qp.drawLine(self.scene_to_screen(tip), self.scene_to_screen(self.cursor_scene))

        font_title = QtGui.QFont("Segoe UI", max(8, int(9 * self.zoom)), QtGui.QFont.Weight.Bold if QT_VERSION == 6 else QtGui.QFont.Bold)
        font_pin = QtGui.QFont("Consolas", max(7, int(8 * self.zoom)))
        for cid, comp in self.board.components.items():
            rect, pmap = self.symbol_box_and_pins(comp)
            s_tl = self.scene_to_screen(rect.topLeft())
            s_br = self.scene_to_screen(rect.bottomRight())
            s_rect = QRectF(s_tl, s_br)

            is_sel = (cid == self.selected_comp_id)
            qp.setBrush(QtGui.QColor("#1d2633" if not is_sel else "#263852"))
            qp.setPen(QtGui.QPen(QtGui.QColor("#58a6ff" if is_sel else "#d2a8ff"), 2.0))
            qp.drawRoundedRect(s_rect, 4.0, 4.0)

            qp.setFont(font_title)
            qp.setPen(QtGui.QColor("#ffdf5d"))
            qp.drawText(QPointF(s_rect.left() + 4, s_rect.top() - 5), f"{comp.refdes} : {comp.value}")

            qp.setFont(font_pin)
            for pnum, (edge, tip, net_name) in pmap.items():
                s_edge = self.scene_to_screen(edge)
                s_tip = self.scene_to_screen(tip)
                qp.setPen(QtGui.QPen(QtGui.QColor("#f0883e"), 2.0))
                qp.drawLine(s_edge, s_tip)
                qp.setBrush(QtGui.QColor("#ff7b72"))
                qp.drawEllipse(s_tip, 3.5 * self.zoom, 3.5 * self.zoom)

                qp.setPen(QtGui.QColor("#e6edf3"))
                tx = s_edge.x() + 5 if edge.x() < comp.sch_x else s_edge.x() - 24
                qp.drawText(QPointF(tx, s_edge.y() + 4), f"P{pnum}")

                if net_name:
                    qp.setPen(QtGui.QColor("#7ee787"))
                    nx_pos = s_tip.x() - 48 if tip.x() < comp.sch_x else s_tip.x() + 6
                    qp.drawText(QPointF(nx_pos, s_tip.y() - 4), net_name)


# ============================================================================
# 9. INTERACTIVE 2D PCB CAD CANVAS WIDGET (TAB 3)
# ============================================================================

class PCBCanvasWidget(QtWidgets.QWidget):
    statusChanged = QtCore.pyqtSignal(str)
    boardModified = QtCore.pyqtSignal()

    def __init__(self, board: BoardProject, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.board = board
        self.setMouseTracking(True)
        self.setFocusPolicy(StrongFocus)

        self.zoom_px_per_mm: float = 11.5
        self.pan_x_px: float = 55.0
        self.pan_y_px: float = 45.0
        self.grid_mm: float = 0.5

        self.active_tool: str = "SELECT"
        self.active_layer_id: str = "L1"
        self.active_net_name: str = "VOUT_3V3"
        self.active_trace_width_mm: float = 0.40

        self.selected_comp_id: Optional[str] = None
        self.dragging_comp: bool = False
        self.panning: bool = False
        self.last_mouse_pt: QPointF = QPointF()
        self.route_start_mm: Optional[Tuple[float, float]] = None
        self.cursor_mm: Tuple[float, float] = (0.0, 0.0)

        self.show_ratsnest: bool = True
        self.overlay_mode: str = "NONE"
        self.last_result: Optional[ElectrothermalResult] = None
        self.drc_violations: List[DRCViolation] = []

    def mm_to_screen(self, x_mm: float, y_mm: float) -> QPointF:
        sx = self.pan_x_px + x_mm * self.zoom_px_per_mm
        sy = self.pan_y_px + (self.board.height_mm - y_mm) * self.zoom_px_per_mm
        return QPointF(sx, sy)

    def screen_to_mm(self, sx: float, sy: float, snap: bool = True) -> Tuple[float, float]:
        x_mm = (sx - self.pan_x_px) / self.zoom_px_per_mm
        y_mm = self.board.height_mm - (sy - self.pan_y_px) / self.zoom_px_per_mm
        if snap and self.grid_mm > 0:
            x_mm = round(x_mm / self.grid_mm) * self.grid_mm
            y_mm = round(y_mm / self.grid_mm) * self.grid_mm
        return x_mm, y_mm

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.zoom_px_per_mm = max(3.0, min(85.0, self.zoom_px_per_mm * factor))
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        pos = mouse_pos_f(event)
        if event.button() == MiddleButton or (event.button() == LeftButton and event.modifiers() & AltModifier):
            self.panning = True
            self.last_mouse_pt = pos
            return

        x_mm, y_mm = self.screen_to_mm(pos.x(), pos.y(), snap=True)
        raw_x, raw_y = self.screen_to_mm(pos.x(), pos.y(), snap=False)

        if event.button() == RightButton:
            self.route_start_mm = None
            self.selected_comp_id = None
            self.update()
            return

        if self.active_tool == "SELECT":
            self.selected_comp_id = None
            for cid, comp in self.board.components.items():
                if (abs(raw_x - comp.x_mm) <= 0.55 * comp.width_mm and
                        abs(raw_y - comp.y_mm) <= 0.55 * comp.height_mm):
                    self.selected_comp_id = cid
                    self.dragging_comp = True
                    break
            self.update()

        elif self.active_tool == "ROUTE":
            for _pid, (_c, pad, wx, wy) in self.board.all_world_pads().items():
                if math.hypot(raw_x - wx, raw_y - wy) < 0.85:
                    x_mm, y_mm = wx, wy
                    if pad.net_name:
                        self.active_net_name = pad.net_name
                    break
            if self.route_start_mm is None:
                self.route_start_mm = (x_mm, y_mm)
            else:
                sx, sy = self.route_start_mm
                if math.hypot(x_mm - sx, y_mm - sy) > 0.05:
                    tid = f"TR_{uuid.uuid4().hex[:6]}"
                    self.board.traces[tid] = Trace(
                        tid, self.active_net_name, self.active_layer_id,
                        sx, sy, x_mm, y_mm, self.active_trace_width_mm
                    )
                    self.route_start_mm = (x_mm, y_mm)
                    self.boardModified.emit()
            self.update()

        elif self.active_tool == "VIA":
            vid = f"VIA_{uuid.uuid4().hex[:6]}"
            self.board.vias[vid] = Via(vid, self.active_net_name, x_mm, y_mm, 0.75, 0.35)
            self.boardModified.emit()
            self.update()

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        pos = mouse_pos_f(event)
        if self.panning:
            delta = pos - self.last_mouse_pt
            self.pan_x_px += delta.x()
            self.pan_y_px += delta.y()
            self.last_mouse_pt = pos
            self.update()
            return

        x_mm, y_mm = self.screen_to_mm(pos.x(), pos.y(), snap=True)
        self.cursor_mm = (x_mm, y_mm)
        if self.dragging_comp and self.selected_comp_id:
            comp = self.board.components[self.selected_comp_id]
            comp.x_mm = x_mm
            comp.y_mm = y_mm
            self.boardModified.emit()

        self.statusChanged.emit(
            f"X: {x_mm:6.2f} mm ({x_mm * 39.3701:6.1f} mil) | "
            f"Y: {y_mm:6.2f} mm ({y_mm * 39.3701:6.1f} mil) | "
            f"Tool: {self.active_tool} | Layer: {self.active_layer_id} | Net: {self.active_net_name}"
        )
        self.update()

    def mouseReleaseEvent(self, _event: QtGui.QMouseEvent) -> None:
        self.panning = False
        self.dragging_comp = False

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() == Key_R and self.selected_comp_id:
            comp = self.board.components[self.selected_comp_id]
            comp.rotation_deg = (comp.rotation_deg + 90.0) % 360.0
            self.boardModified.emit()
            self.update()
        elif event.key() == Key_Escape:
            self.route_start_mm = None
            self.selected_comp_id = None
            self.update()

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing if QT_VERSION == 6 else QtGui.QPainter.Antialiasing, True)
        qp.fillRect(self.rect(), QtGui.QColor("#0f1318"))

        p_tl = self.mm_to_screen(0.0, self.board.height_mm)
        p_br = self.mm_to_screen(self.board.width_mm, 0.0)
        board_rect = QRectF(p_tl, p_br)
        qp.fillRect(board_rect, QtGui.QColor("#16291c"))

        if self.zoom_px_per_mm >= 6.0:
            qp.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 28), 1.0))
            step = max(1.0, self.grid_mm * 2.0)
            gx = 0.0
            while gx <= self.board.width_mm:
                gy = 0.0
                while gy <= self.board.height_mm:
                    qp.drawPoint(self.mm_to_screen(gx, gy))
                    gy += step
                gx += step

        qp.setPen(QtGui.QPen(QtGui.QColor("#e8d44d"), 2.0))
        qp.drawRect(board_rect)

        visible_layers = {l.layer_id: l for l in self.board.layers if l.visible}

        for zone in self.board.zones.values():
            if zone.layer_id not in visible_layers:
                continue
            lay_col = QtGui.QColor(visible_layers[zone.layer_id].color_hex)
            lay_col.setAlpha(38)
            poly = QtGui.QPolygonF([self.mm_to_screen(px, py) for px, py in zone.points_mm])
            qp.setBrush(QtGui.QBrush(lay_col))
            qp.setPen(QtGui.QPen(lay_col.lighter(130), 1.2, DashLine))
            qp.drawPolygon(poly)

        for lay in reversed(self.board.layers):
            if not lay.visible:
                continue
            col = QtGui.QColor(lay.color_hex)
            col.setAlpha(215)
            for tr in self.board.traces.values():
                if tr.layer_id != lay.layer_id:
                    continue
                p1 = self.mm_to_screen(tr.x1_mm, tr.y1_mm)
                p2 = self.mm_to_screen(tr.x2_mm, tr.y2_mm)
                w_px = max(2.0, tr.width_mm * self.zoom_px_per_mm)
                pen = QtGui.QPen(col, w_px, SolidLine, RoundCap, RoundJoin)
                qp.setPen(pen)
                qp.drawLine(p1, p2)

        for v in self.board.vias.values():
            vp = self.mm_to_screen(v.x_mm, v.y_mm)
            r_out = max(3.5, 0.5 * v.outer_dia_mm * self.zoom_px_per_mm)
            r_drl = max(1.5, 0.5 * v.drill_dia_mm * self.zoom_px_per_mm)
            qp.setBrush(QtGui.QColor("#d4af37"))
            qp.setPen(QtGui.QPen(QtGui.QColor("#fff2a8"), 1.0))
            qp.drawEllipse(vp, r_out, r_out)
            qp.setBrush(QtGui.QColor("#0f1318"))
            qp.drawEllipse(vp, r_drl, r_drl)

        font_small = QtGui.QFont("Consolas", 8)
        for cid, comp in self.board.components.items():
            for pad in comp.pads:
                wx, wy = comp.pad_world_pos(pad)
                cp = self.mm_to_screen(wx, wy)
                pw_px = max(4.0, pad.width_mm * self.zoom_px_per_mm)
                ph_px = max(4.0, pad.height_mm * self.zoom_px_per_mm)
                rect_pad = QRectF(cp.x() - 0.5 * pw_px, cp.y() - 0.5 * ph_px, pw_px, ph_px)
                qp.setBrush(QtGui.QColor("#d9a74a") if not pad.is_thermal_pad else QtGui.QColor("#b87333"))
                qp.setPen(QtGui.QPen(QtGui.QColor("#ffe299"), 1.0))
                if pad.shape == "CIRCLE":
                    qp.drawEllipse(rect_pad)
                else:
                    qp.drawRect(rect_pad)

                if self.zoom_px_per_mm >= 10.0 and pad.net_name:
                    qp.setFont(font_small)
                    qp.setPen(QtGui.QColor("#ffffff"))
                    qp.drawText(rect_pad, AlignCenter, pad.net_name[:6])

                if self.overlay_mode == "DC_VOLTS" and self.last_result:
                    v_val = self.last_result.pad_voltages_v.get(pad.pad_id)
                    if v_val is not None:
                        qp.setPen(QtGui.QColor("#5cff95"))
                        qp.drawText(QPointF(cp.x() + 5, cp.y() - 5), f"{v_val:.3f}V")

            c_pt = self.mm_to_screen(comp.x_mm, comp.y_mm)
            bw_px = comp.width_mm * self.zoom_px_per_mm
            bh_px = comp.height_mm * self.zoom_px_per_mm
            body_rect = QRectF(c_pt.x() - 0.5 * bw_px, c_pt.y() - 0.5 * bh_px, bw_px, bh_px)
            is_sel = (cid == self.selected_comp_id)
            qp.setBrush(QtGui.QColor(255, 255, 255, 18 if not is_sel else 45))
            qp.setPen(QtGui.QPen(QtGui.QColor("#00ffff" if is_sel else "#e6edf3"), 1.8 if is_sel else 1.2))
            qp.drawRect(body_rect)

            qp.setFont(QtGui.QFont("Segoe UI", 9, QtGui.QFont.Weight.Bold if QT_VERSION == 6 else QtGui.QFont.Bold))
            qp.setPen(QtGui.QColor("#fff8b0"))
            qp.drawText(QPointF(body_rect.left() + 2, body_rect.top() - 4), f"{comp.refdes} ({comp.value})")

            if self.overlay_mode == "THERMAL_TJ" and self.last_result:
                tj = self.last_result.comp_tj_c.get(comp.refdes)
                if tj is not None:
                    qp.setPen(QtGui.QColor("#ff6b57"))
                    qp.drawText(QPointF(c_pt.x() - 24, c_pt.y() + 4), f"Tj={tj:.1f}C")

        if self.show_ratsnest:
            qp.setPen(QtGui.QPen(QtGui.QColor("#7ee787"), 1.1, DotLine))
            for _net, x1, y1, x2, y2, _lid in AStarAutorouter.compute_ratsnest(self.board):
                qp.drawLine(self.mm_to_screen(x1, y1), self.mm_to_screen(x2, y2))

        if self.active_tool == "ROUTE" and self.route_start_mm is not None:
            p_start = self.mm_to_screen(*self.route_start_mm)
            p_cur = self.mm_to_screen(*self.cursor_mm)
            qp.setPen(QtGui.QPen(QtGui.QColor("#ffea47"), max(2.0, self.active_trace_width_mm * self.zoom_px_per_mm), DashLine))
            qp.drawLine(p_start, p_cur)

        for v in self.drc_violations:
            vp = self.mm_to_screen(v.x_mm, v.y_mm)
            qp.setBrush(NoBrush)
            qp.setPen(QtGui.QPen(QtGui.QColor("#ff2e55"), 2.4))
            qp.drawEllipse(vp, 11.0, 11.0)
            qp.drawLine(QPointF(vp.x() - 8, vp.y() - 8), QPointF(vp.x() + 8, vp.y() + 8))
            qp.drawLine(QPointF(vp.x() - 8, vp.y() + 8), QPointF(vp.x() + 8, vp.y() - 8))


# ============================================================================
# 10. INTERACTIVE 3D ISOMETRIC BOARD & THERMAL VIEWER WIDGET (TAB 4)
# ============================================================================

class PCB3DViewerWidget(QtWidgets.QWidget):
    def __init__(self, board: BoardProject, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.board = board
        self.yaw_deg: float = -38.0
        self.pitch_deg: float = 54.0
        self.scale_px_mm: float = 11.0
        self.z_explode_factor: float = 3.5
        self.pan_x: float = 0.0
        self.pan_y: float = 20.0
        self.color_by_thermal: bool = True
        self.last_result: Optional[ElectrothermalResult] = None
        self._last_mouse: QPointF = QPointF()
        self._mode: str = "NONE"

    def project_3d(self, x_mm: float, y_mm: float, z_mm: float) -> QPointF:
        cx = x_mm - 0.5 * self.board.width_mm
        cy = y_mm - 0.5 * self.board.height_mm
        cz = z_mm * self.z_explode_factor

        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        cy_a, sy_a = math.cos(yaw), math.sin(yaw)
        cp_a, sp_a = math.cos(pitch), math.sin(pitch)

        rx = cx * cy_a - cy * sy_a
        ry = cx * sy_a + cy * cy_a
        sx = self.width() * 0.5 + self.pan_x + rx * self.scale_px_mm
        sy = self.height() * 0.5 + self.pan_y - (ry * cp_a + cz * sp_a) * self.scale_px_mm
        return QPointF(sx, sy)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        self._last_mouse = mouse_pos_f(event)
        self._mode = "ORBIT" if event.button() == LeftButton else "PAN"

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        pos = mouse_pos_f(event)
        d = pos - self._last_mouse
        self._last_mouse = pos
        if self._mode == "ORBIT":
            self.yaw_deg = (self.yaw_deg + d.x() * 0.45) % 360.0
            self.pitch_deg = max(12.0, min(88.0, self.pitch_deg + d.y() * 0.35))
        elif self._mode == "PAN":
            self.pan_x += d.x()
            self.pan_y += d.y()
        self.update()

    def mouseReleaseEvent(self, _event: QtGui.QMouseEvent) -> None:
        self._mode = "NONE"

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        factor = 1.12 if event.angleDelta().y() > 0 else 1.0 / 1.12
        self.scale_px_mm = max(3.5, min(40.0, self.scale_px_mm * factor))
        self.update()

    def _draw_3d_box(
        self, qp: QtGui.QPainter, x0: float, y0: float, z0: float,
        x1: float, y1: float, z1: float, top_col: QtGui.QColor, edge_col: QtGui.QColor
    ) -> None:
        b0 = self.project_3d(x0, y0, z0)
        b1 = self.project_3d(x1, y0, z0)
        b2 = self.project_3d(x1, y1, z0)
        b3 = self.project_3d(x0, y1, z0)

        t0 = self.project_3d(x0, y0, z1)
        t1 = self.project_3d(x1, y0, z1)
        t2 = self.project_3d(x1, y1, z1)
        t3 = self.project_3d(x0, y1, z1)

        side_col1 = top_col.darker(135)
        side_col2 = top_col.darker(160)
        qp.setPen(QtGui.QPen(edge_col, 1.0))

        for quad, col in (
            ([b0, b1, t1, t0], side_col1),
            ([b1, b2, t2, t1], side_col2),
            ([b2, b3, t3, t2], side_col1),
            ([b3, b0, t0, t3], side_col2),
            ([t0, t1, t2, t3], top_col),
        ):
            qp.setBrush(col)
            qp.drawPolygon(QtGui.QPolygonF(quad))

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing if QT_VERSION == 6 else QtGui.QPainter.Antialiasing, True)
        qp.fillRect(self.rect(), QtGui.QColor("#0c1017"))

        n_lay = max(1, len(self.board.layers))
        total_h_mm = 1.6
        layer_z: Dict[str, float] = {}
        for idx, lay in enumerate(self.board.layers):
            frac = 1.0 - (idx / max(1, n_lay - 1))
            layer_z[lay.layer_id] = frac * total_h_mm

        self._draw_3d_box(
            qp, 0.0, 0.0, 0.0, self.board.width_mm, self.board.height_mm, total_h_mm,
            QtGui.QColor(26, 77, 46, 110), QtGui.QColor("#dfd05b")
        )

        for lay in reversed(self.board.layers):
            if not lay.visible:
                continue
            z_mm = layer_z[lay.layer_id]
            col = QtGui.QColor(lay.color_hex)
            col.setAlpha(220)
            for tr in self.board.traces.values():
                if tr.layer_id != lay.layer_id:
                    continue
                p1 = self.project_3d(tr.x1_mm, tr.y1_mm, z_mm)
                p2 = self.project_3d(tr.x2_mm, tr.y2_mm, z_mm)
                qp.setPen(QtGui.QPen(col, max(2.0, tr.width_mm * self.scale_px_mm * 0.85), SolidLine, RoundCap))
                qp.drawLine(p1, p2)

        for v in self.board.vias.values():
            p_bot = self.project_3d(v.x_mm, v.y_mm, 0.0)
            p_top = self.project_3d(v.x_mm, v.y_mm, total_h_mm)
            r_px = max(3.0, 0.45 * v.outer_dia_mm * self.scale_px_mm)
            qp.setPen(QtGui.QPen(QtGui.QColor("#d4af37"), r_px * 1.3, SolidLine, RoundCap))
            qp.drawLine(p_bot, p_top)
            qp.setBrush(QtGui.QColor("#ffea79"))
            qp.setPen(QtGui.QPen(QtGui.QColor("#4a3b00"), 1.0))
            qp.drawEllipse(p_top, r_px, r_px * 0.75)

        comps_sorted = sorted(self.board.components.values(), key=lambda c: c.y_mm, reverse=True)
        font_lbl = QtGui.QFont("Segoe UI", 8, QtGui.QFont.Weight.Bold if QT_VERSION == 6 else QtGui.QFont.Bold)
        qp.setFont(font_lbl)
        for comp in comps_sorted:
            z_base = total_h_mm
            z_top = total_h_mm + comp.height_3d_mm
            x0 = comp.x_mm - 0.5 * comp.width_mm
            x1 = comp.x_mm + 0.5 * comp.width_mm
            y0 = comp.y_mm - 0.5 * comp.height_mm
            y1 = comp.y_mm + 0.5 * comp.height_mm

            pkg_col = QtGui.QColor("#2c3440")
            if self.color_by_thermal and self.last_result and comp.refdes in self.last_result.comp_tj_c:
                tj = self.last_result.comp_tj_c[comp.refdes]
                u = max(0.0, min(1.0, (tj - 25.0) / 65.0))
                pkg_col = QtGui.QColor(int(60 + 195 * u), int(90 * (1.0 - u)), int(180 * (1.0 - u)))

            self._draw_3d_box(qp, x0, y0, z_base, x1, y1, z_top, pkg_col, QtGui.QColor("#9ecbff"))
            pt_lbl = self.project_3d(comp.x_mm, comp.y_mm, z_top + 0.25)
            qp.setPen(QtGui.QColor("#ffffff"))
            lbl = comp.refdes
            if self.color_by_thermal and self.last_result and comp.refdes in self.last_result.comp_tj_c:
                lbl += f" ({self.last_result.comp_tj_c[comp.refdes]:.1f}C)"
            qp.drawText(QPointF(pt_lbl.x() - 24, pt_lbl.y() - 4), lbl)

        qp.setPen(QtGui.QColor("#8b949e"))
        qp.drawText(
            QPointF(14, 24),
            f"3D Orbit Viewer | Yaw: {self.yaw_deg:.1f} deg | Pitch: {self.pitch_deg:.1f} deg | "
            f"Z-Explode: {self.z_explode_factor:.1f}x (Left-Drag: Orbit, Right-Drag: Pan, Wheel: Zoom)"
        )


# ============================================================================
# 11. MAIN 11-TAB APPLICATION WINDOW
# ============================================================================

class PCBMasterSuiteWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"PCB Master Suite (Universal Edition - Qt{QT_VERSION}) - Schematic, SPICE, 2D/3D CAD, Importers & Multi-Physics")
        self.resize(1620, 960)
        self.board = build_default_demo_board()
        self.undo_stack: List[str] = [self.board.to_json()]
        self.redo_stack: List[str] = []

        self._apply_dark_stylesheet()
        self._build_ui()
        self._run_electrothermal_solver()
        self._run_spice_simulation()

    def _apply_dark_stylesheet(self) -> None:
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #141820; color: #e1e7ef; font-family: 'Segoe UI'; font-size: 9.5pt; }
            QTabWidget::pane { border: 1px solid #2b3343; }
            QTabBar::tab { background: #1d2330; color: #aeb9c7; padding: 7px 11px; border: 1px solid #2b3343; font-weight: bold; }
            QTabBar::tab:selected { background: #2663c7; color: #ffffff; }
            QPushButton { background-color: #2358b0; color: white; border-radius: 4px; padding: 6px 12px; font-weight: bold; }
            QPushButton:hover { background-color: #2f6fd6; }
            QGroupBox { border: 1px solid #2e3846; border-radius: 5px; margin-top: 8px; padding-top: 10px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #79b8ff; }
            QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QTableWidget, QTextEdit {
                background-color: #1c222d; color: #f0f6fc; border: 1px solid #354152; border-radius: 3px; padding: 3px;
            }
            QHeaderView::section { background-color: #222936; color: #9ecbff; padding: 4px; border: 1px solid #323c4d; }
        """)

    def _record_undo(self) -> None:
        self.undo_stack.append(self.board.to_json())
        if len(self.undo_stack) > 40:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_v = QtWidgets.QVBoxLayout(central)

        # Top Global Action Toolbar
        top_bar = QtWidgets.QHBoxLayout()
        btn_save = QtWidgets.QPushButton("Save Project (.pcbproj)")
        btn_save.clicked.connect(self._on_save_project)
        btn_load = QtWidgets.QPushButton("Load Project")
        btn_load.clicked.connect(self._on_load_project)
        btn_kicad = QtWidgets.QPushButton("Import KiCad (.kicad_pcb)")
        btn_kicad.setStyleSheet("background-color: #047857;")
        btn_kicad.clicked.connect(self._on_import_kicad)
        btn_dxf = QtWidgets.QPushButton("Import DXF Outline")
        btn_dxf.clicked.connect(self._on_import_dxf)
        btn_undo = QtWidgets.QPushButton("Undo (Ctrl+Z)")
        btn_undo.setShortcut("Ctrl+Z")
        btn_undo.clicked.connect(self._on_undo)
        btn_sync = QtWidgets.QPushButton("Sync Schematic -> PCB")
        btn_sync.setStyleSheet("background-color: #6e40c9;")
        btn_sync.clicked.connect(self._on_sync_schematic_to_pcb)
        btn_spice = QtWidgets.QPushButton("Run SPICE Sim")
        btn_spice.setStyleSheet("background-color: #0e7490;")
        btn_spice.clicked.connect(self._run_spice_simulation)
        btn_autoroute = QtWidgets.QPushButton("Run A* Autoroute")
        btn_autoroute.setStyleSheet("background-color: #1f883d;")
        btn_autoroute.clicked.connect(self._on_autoroute)
        btn_drc = QtWidgets.QPushButton("Run Live DRC")
        btn_drc.clicked.connect(self._on_run_drc)
        btn_solve = QtWidgets.QPushButton("Solve Electrothermal FEA")
        btn_solve.setStyleSheet("background-color: #b84a27;")
        btn_solve.clicked.connect(self._run_electrothermal_solver)

        for b in (btn_save, btn_load, btn_kicad, btn_dxf, btn_undo, btn_sync, btn_spice, btn_autoroute, btn_drc, btn_solve):
            top_bar.addWidget(b)
        top_bar.addStretch()
        main_v.addLayout(top_bar)

        self.tabs = QtWidgets.QTabWidget()
        main_v.addWidget(self.tabs, 1)

        self._build_tab1_schematic_capture()
        self._build_tab2_spice_simulator()
        self._build_tab3_cad_canvas()
        self._build_tab4_3d_viewer()
        self._build_tab5_component_wizard()
        self._build_tab6_stackup_editor()
        self._build_tab7_dc_ir_drop()
        self._build_tab8_electrothermal_fea()
        self._build_tab9_signal_integrity()
        self._build_tab10_pdn_analyzer()
        self._build_tab11_fabrication_export()

        self.status_bar = self.statusBar()
        self.status_bar.showMessage(f"Ready. Running on Qt{QT_VERSION} Engine.")

    # ------------------------------------------------------------------------
    # TAB 1: INTERACTIVE SCHEMATIC CAPTURE & NETLIST SYNC
    # ------------------------------------------------------------------------
    def _build_tab1_schematic_capture(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        left_v = QtWidgets.QVBoxLayout()
        grp_sch = QtWidgets.QGroupBox("Schematic Capture Controls")
        fl = QtWidgets.QFormLayout(grp_sch)

        self.cmb_sch_tool = QtWidgets.QComboBox()
        self.cmb_sch_tool.addItems(["SELECT", "WIRE", "NET_LABEL"])
        self.cmb_sch_tool.currentTextChanged.connect(self._sync_sch_controls)

        self.ed_sch_net = QtWidgets.QLineEdit("VOUT_3V3")
        self.ed_sch_net.textChanged.connect(self._sync_sch_controls)

        btn_push = QtWidgets.QPushButton("Forward Annotate (Sync Schematic -> PCB)")
        btn_push.setStyleSheet("background-color: #6e40c9;")
        btn_push.clicked.connect(self._on_sync_schematic_to_pcb)

        fl.addRow("Schematic Tool:", self.cmb_sch_tool)
        fl.addRow("Active Net Name:", self.ed_sch_net)
        fl.addRow(btn_push)
        left_v.addWidget(grp_sch)

        grp_add = QtWidgets.QGroupBox("Quick Add Schematic Symbol")
        fl2 = QtWidgets.QFormLayout(grp_add)
        self.sch_new_ref = QtWidgets.QLineEdit("R1")
        self.sch_new_val = QtWidgets.QLineEdit("10k_0805")
        self.sch_new_pkg = QtWidgets.QComboBox()
        self.sch_new_pkg.addItems(["0805", "1206", "SOIC-8", "QFN-16"])
        btn_add_sym = QtWidgets.QPushButton("Add Symbol to Sheet & Board")
        btn_add_sym.clicked.connect(self._on_add_schematic_symbol)
        fl2.addRow("RefDes:", self.sch_new_ref)
        fl2.addRow("Value:", self.sch_new_val)
        fl2.addRow("Footprint:", self.sch_new_pkg)
        fl2.addRow(btn_add_sym)
        left_v.addWidget(grp_add)

        self.txt_netlist_view = QtWidgets.QTextEdit()
        self.txt_netlist_view.setReadOnly(True)
        left_v.addWidget(QtWidgets.QLabel("Live Extracted Netlist:"))
        left_v.addWidget(self.txt_netlist_view, 1)

        self.sch_canvas = SchematicCanvasWidget(self.board)
        self.sch_canvas.statusChanged.connect(lambda s: self.status_bar.showMessage(s))
        self.sch_canvas.schematicModified.connect(self._on_schematic_modified)

        layout.addLayout(left_v, 1)
        layout.addWidget(self.sch_canvas, 4)
        self.tabs.addTab(tab, "1. Schematic Capture")
        self._refresh_netlist_text()

    def _sync_sch_controls(self) -> None:
        self.sch_canvas.tool = self.cmb_sch_tool.currentText()
        self.sch_canvas.active_net = self.ed_sch_net.text().strip() or "NET_1"

    def _on_add_schematic_symbol(self) -> None:
        self._record_undo()
        ref = self.sch_new_ref.text().strip() or f"U{len(self.board.components) + 1}"
        val = self.sch_new_val.text().strip() or "PART"
        pkg = self.sch_new_pkg.currentText()
        if pkg == "SOIC-8":
            comp = FootprintWizard.create_soic(ref, val, 36.0, 42.0, 8, 1.27, 4.0, 0.5, sch_x=380.0, sch_y=360.0)
        elif pkg == "QFN-16":
            comp = FootprintWizard.create_qfn(ref, val, 36.0, 42.0, 7.0, 4, 0.8, 1.0, sch_x=380.0, sch_y=360.0)
        else:
            comp = FootprintWizard.create_smd_2pin(ref, val, 36.0, 42.0, pkg, "", "", 0.05, sch_x=380.0, sch_y=360.0)
        self.board.components[comp.comp_id] = comp
        self._refresh_netlist_text()
        self.sch_canvas.update()
        self.canvas.update()
        self.viewer_3d.update()

    def _on_schematic_modified(self) -> None:
        self._record_undo()
        self._refresh_netlist_text()
        self.canvas.update()
        self.viewer_3d.update()

    def _on_sync_schematic_to_pcb(self) -> None:
        count = self.board.sync_schematic_to_pcb()
        self._refresh_netlist_text()
        self._on_run_drc()
        self.canvas.update()
        self.viewer_3d.update()
        self.status_bar.showMessage(f"Forward Annotation Complete: Synchronized {count} pad net assignments to 2D/3D PCB.")

    def _refresh_netlist_text(self) -> None:
        net_map: Dict[str, List[str]] = {}
        for comp in self.board.components.values():
            for pad in comp.pads:
                if pad.net_name:
                    net_map.setdefault(pad.net_name, []).append(f"{comp.refdes}.{pad.pin_number}")
        lines = ["=== SYNCHRONIZED NETLIST ==="]
        for net, pins in sorted(net_map.items()):
            lines.append(f"{net:12s} : {', '.join(pins)}")
        self.txt_netlist_view.setPlainText("\n".join(lines))

    # ------------------------------------------------------------------------
    # TAB 2: SPICE TIME-DOMAIN TRANSIENT & FFT CIRCUIT SIMULATOR
    # ------------------------------------------------------------------------
    def _build_tab2_spice_simulator(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        left_v = QtWidgets.QVBoxLayout()
        grp_preset = QtWidgets.QGroupBox("SPICE Circuit Deck & Transient Settings")
        fl = QtWidgets.QFormLayout(grp_preset)

        self.cmb_spice_preset = QtWidgets.QComboBox()
        self.cmb_spice_preset.addItems([
            "Preset 1: 12V-to-3.3V Buck Converter Start-Up & Load Step",
            "Preset 2: 100 MHz Clock Trace RLC Ringing & ESD Clamping",
        ])
        self.cmb_spice_preset.currentIndexChanged.connect(self._on_spice_preset_changed)

        self.spn_spice_tstop = QtWidgets.QDoubleSpinBox()
        self.spn_spice_tstop.setRange(0.01, 5000.0)
        self.spn_spice_tstop.setDecimals(2)
        self.spn_spice_tstop.setValue(80.0)

        self.spn_spice_dt = QtWidgets.QDoubleSpinBox()
        self.spn_spice_dt.setRange(0.0002, 50.0)
        self.spn_spice_dt.setDecimals(4)
        self.spn_spice_dt.setValue(0.05)

        self.ed_spice_fft_node = QtWidgets.QLineEdit("VOUT_3V3")

        btn_run_spice = QtWidgets.QPushButton("Run Time-Domain Transient SPICE Solver")
        btn_run_spice.setStyleSheet("background-color: #0e7490;")
        btn_run_spice.clicked.connect(self._run_spice_simulation)

        fl.addRow("Circuit Preset:", self.cmb_spice_preset)
        fl.addRow("Stop Time (us):", self.spn_spice_tstop)
        fl.addRow("Time Step dt (us):", self.spn_spice_dt)
        fl.addRow("FFT Target Node:", self.ed_spice_fft_node)
        fl.addRow(btn_run_spice)
        left_v.addWidget(grp_preset)

        left_v.addWidget(QtWidgets.QLabel("Editable SPICE Netlist (R, L, C, V, I, D, PULSE, SIN, DC):"))
        self.txt_spice_netlist = QtWidgets.QTextEdit()
        self.txt_spice_netlist.setFont(QtGui.QFont("Consolas", 9))
        self.txt_spice_netlist.setPlainText(SPICE_PRESET_BUCK)
        left_v.addWidget(self.txt_spice_netlist, 1)

        self.txt_spice_report = QtWidgets.QTextEdit()
        self.txt_spice_report.setReadOnly(True)
        self.txt_spice_report.setFont(QtGui.QFont("Consolas", 8))
        left_v.addWidget(self.txt_spice_report, 1)

        right_plots = pg.GraphicsLayoutWidget()
        self.plt_spice_v = right_plots.addPlot(title="SPICE Node Voltage Waveforms V(t)")
        self.plt_spice_v.setLabel("bottom", "Time", units="s")
        self.plt_spice_v.setLabel("left", "Voltage", units="V")
        self.plt_spice_v.addLegend()
        self.plt_spice_v.showGrid(x=True, y=True, alpha=0.3)

        right_plots.nextRow()
        self.plt_spice_i = right_plots.addPlot(title="SPICE Inductor / Source Branch Currents I(t)")
        self.plt_spice_i.setLabel("bottom", "Time", units="s")
        self.plt_spice_i.setLabel("left", "Current", units="A")
        self.plt_spice_i.addLegend()
        self.plt_spice_i.showGrid(x=True, y=True, alpha=0.3)

        right_plots.nextRow()
        self.plt_spice_fft = right_plots.addPlot(title="Waveform Spectral Content (FFT Magnitude dBV)")
        self.plt_spice_fft.setLabel("bottom", "Frequency", units="Hz")
        self.plt_spice_fft.setLabel("left", "Magnitude (dBV)")
        self.plt_spice_fft.showGrid(x=True, y=True, alpha=0.3)

        layout.addLayout(left_v, 2)
        layout.addWidget(right_plots, 3)
        self.tabs.addTab(tab, "2. SPICE Transient Sim")

    def _on_spice_preset_changed(self, idx: int) -> None:
        if idx == 0:
            self.txt_spice_netlist.setPlainText(SPICE_PRESET_BUCK)
            self.spn_spice_tstop.setValue(80.0)
            self.spn_spice_dt.setValue(0.05)
            self.ed_spice_fft_node.setText("VOUT_3V3")
        else:
            self.txt_spice_netlist.setPlainText(SPICE_PRESET_CLOCK_SI)
            self.spn_spice_tstop.setValue(0.04)
            self.spn_spice_dt.setValue(0.0002)
            self.ed_spice_fft_node.setText("CLK_100M")
        self._run_spice_simulation()

    def _run_spice_simulation(self) -> None:
        try:
            netlist = self.txt_spice_netlist.toPlainText()
            t_stop_s = self.spn_spice_tstop.value() * 1e-6
            dt_s = self.spn_spice_dt.value() * 1e-6
            fft_node = self.ed_spice_fft_node.text().strip().upper()

            res = SpiceTransientEngine.simulate(netlist, t_stop_s=t_stop_s, dt_s=dt_s, fft_node=fft_node)
            self.txt_spice_report.setPlainText(res.summary_text)

            self.plt_spice_v.clear()
            colors = ["#58a6ff", "#3fb950", "#ff7b72", "#d2a8ff", "#ffdf5d", "#f0883e"]
            for i, (n_name, wave) in enumerate(res.node_voltages.items()):
                col = colors[i % len(colors)]
                self.plt_spice_v.plot(res.time_s, wave, pen=pg.mkPen(col, width=2.0), name=f"V({n_name})")

            self.plt_spice_i.clear()
            for i, (b_name, i_wave) in enumerate(res.branch_currents.items()):
                col = colors[(i + 2) % len(colors)]
                self.plt_spice_i.plot(res.time_s, i_wave, pen=pg.mkPen(col, width=2.0), name=b_name)

            self.plt_spice_fft.clear()
            self.plt_spice_fft.plot(res.fft_freq_hz[1:], res.fft_mag_db[1:], pen=pg.mkPen("#ffdf5d", width=1.8))
            self.status_bar.showMessage("SPICE Time-Domain Transient Simulation converged.")
        except Exception as exc:
            self.txt_spice_report.setPlainText(f"SPICE Simulation Error:\n{exc}")

    # ------------------------------------------------------------------------
    # TAB 3: INTERACTIVE 2D PCB CAD CANVAS & AUTOROUTER
    # ------------------------------------------------------------------------
    def _build_tab3_cad_canvas(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        left_ctrl = QtWidgets.QVBoxLayout()
        grp_tool = QtWidgets.QGroupBox("CAD Editing Mode")
        fl = QtWidgets.QFormLayout(grp_tool)

        self.cmb_tool = QtWidgets.QComboBox()
        self.cmb_tool.addItems(["SELECT", "ROUTE", "VIA"])
        self.cmb_tool.currentTextChanged.connect(self._sync_canvas_controls)

        self.cmb_active_layer = QtWidgets.QComboBox()
        for l in self.board.layers:
            self.cmb_active_layer.addItem(f"{l.layer_id}: {l.name}", l.layer_id)
        self.cmb_active_layer.currentIndexChanged.connect(self._sync_canvas_controls)

        self.cmb_active_net = QtWidgets.QComboBox()
        self.cmb_active_net.addItems(sorted(list(self.board.nets)))
        self.cmb_active_net.currentTextChanged.connect(self._sync_canvas_controls)

        self.spn_trace_w = QtWidgets.QDoubleSpinBox()
        self.spn_trace_w.setRange(0.10, 5.0)
        self.spn_trace_w.setSingleStep(0.05)
        self.spn_trace_w.setValue(0.40)
        self.spn_trace_w.valueChanged.connect(self._sync_canvas_controls)

        self.cmb_overlay = QtWidgets.QComboBox()
        self.cmb_overlay.addItems(["NONE", "DC_VOLTS", "THERMAL_TJ"])
        self.cmb_overlay.currentTextChanged.connect(self._sync_canvas_controls)

        fl.addRow("Active Tool:", self.cmb_tool)
        fl.addRow("Active Layer:", self.cmb_active_layer)
        fl.addRow("Active Net:", self.cmb_active_net)
        fl.addRow("Trace Width (mm):", self.spn_trace_w)
        fl.addRow("Physics Overlay:", self.cmb_overlay)
        left_ctrl.addWidget(grp_tool)

        grp_vis = QtWidgets.QGroupBox("Layer Visibility")
        vl = QtWidgets.QVBoxLayout(grp_vis)
        self.layer_Checkboxes: Dict[str, QtWidgets.QCheckBox] = {}
        for l in self.board.layers:
            cb = QtWidgets.QCheckBox(f"L{l.display_number}: {l.name}")
            cb.setChecked(l.visible)
            cb.stateChanged.connect(self._sync_layer_visibility)
            vl.addWidget(cb)
            self.layer_Checkboxes[l.layer_id] = cb
        left_ctrl.addWidget(grp_vis)

        grp_drc = QtWidgets.QGroupBox("DRC & Autorouter Log")
        dl = QtWidgets.QVBoxLayout(grp_drc)
        self.txt_drc_log = QtWidgets.QTextEdit()
        self.txt_drc_log.setReadOnly(True)
        dl.addWidget(self.txt_drc_log)
        left_ctrl.addWidget(grp_drc, 1)

        self.canvas = PCBCanvasWidget(self.board)
        self.canvas.statusChanged.connect(lambda s: self.status_bar.showMessage(s))
        self.canvas.boardModified.connect(self._record_undo)

        layout.addLayout(left_ctrl, 1)
        layout.addWidget(self.canvas, 4)
        self.tabs.addTab(tab, "3. 2D PCB CAD")

    def _sync_canvas_controls(self) -> None:
        self.canvas.active_tool = self.cmb_tool.currentText()
        self.canvas.active_layer_id = self.cmb_active_layer.currentData() or "L1"
        self.canvas.active_net_name = self.cmb_active_net.currentText()
        self.canvas.active_trace_width_mm = self.spn_trace_w.value()
        self.canvas.overlay_mode = self.cmb_overlay.currentText()
        self.canvas.update()

    def _sync_layer_visibility(self) -> None:
        for l in self.board.layers:
            if l.layer_id in self.layer_Checkboxes:
                l.visible = self.layer_Checkboxes[l.layer_id].isChecked()
        self.canvas.update()
        self.viewer_3d.update()

    # ------------------------------------------------------------------------
    # TAB 4: INTERACTIVE 3D BOARD & THERMAL EXTRUSION VIEWER
    # ------------------------------------------------------------------------
    def _build_tab4_3d_viewer(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        ctrl_h = QtWidgets.QHBoxLayout()
        chk_th = QtWidgets.QCheckBox("Shade 3D Components by Solved Junction Temp (Tj)")
        chk_th.setChecked(True)
        chk_th.stateChanged.connect(lambda st: self._update_3d_settings(st > 0, sld_z.value() / 10.0))

        sld_z = QtWidgets.QSlider(Horizontal)
        sld_z.setRange(10, 120)
        sld_z.setValue(35)
        sld_z.valueChanged.connect(lambda v: self._update_3d_settings(chk_th.isChecked(), v / 10.0))

        btn_reset_cam = QtWidgets.QPushButton("Reset 3D Camera")
        btn_reset_cam.clicked.connect(self._reset_3d_camera)

        ctrl_h.addWidget(chk_th)
        ctrl_h.addSpacing(20)
        ctrl_h.addWidget(QtWidgets.QLabel("Z-Stackup Explosion Factor:"))
        ctrl_h.addWidget(sld_z, 1)
        ctrl_h.addWidget(btn_reset_cam)

        self.viewer_3d = PCB3DViewerWidget(self.board)
        layout.addLayout(ctrl_h)
        layout.addWidget(self.viewer_3d, 1)
        self.tabs.addTab(tab, "4. 3D Board Viewer")

    def _update_3d_settings(self, thermal_color: bool, z_factor: float) -> None:
        self.viewer_3d.color_by_thermal = thermal_color
        self.viewer_3d.z_explode_factor = z_factor
        self.viewer_3d.update()

    def _reset_3d_camera(self) -> None:
        self.viewer_3d.yaw_deg = -38.0
        self.viewer_3d.pitch_deg = 54.0
        self.viewer_3d.scale_px_mm = 11.0
        self.viewer_3d.pan_x = 0.0
        self.viewer_3d.pan_y = 20.0
        self.viewer_3d.update()

    # ------------------------------------------------------------------------
    # TAB 5: PARAMETRIC FOOTPRINT & COMPONENT WIZARD
    # ------------------------------------------------------------------------
    def _build_tab5_component_wizard(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        grp = QtWidgets.QGroupBox("Parametric Package & Footprint Generator")
        fl = QtWidgets.QFormLayout(grp)

        self.wiz_refdes = QtWidgets.QLineEdit("U3")
        self.wiz_val = QtWidgets.QLineEdit("LDO_REG_1V8")
        self.wiz_pkg = QtWidgets.QComboBox()
        self.wiz_pkg.addItems(["QFN (Exposed Thermal Pad)", "SOIC", "0805", "1206", "IND_6x6"])
        self.wiz_pins = QtWidgets.QSpinBox()
        self.wiz_pins.setRange(2, 64)
        self.wiz_pins.setValue(16)
        self.wiz_x = QtWidgets.QDoubleSpinBox()
        self.wiz_x.setRange(2.0, 78.0)
        self.wiz_x.setValue(38.0)
        self.wiz_y = QtWidgets.QDoubleSpinBox()
        self.wiz_y.setRange(2.0, 52.0)
        self.wiz_y.setValue(40.0)
        self.wiz_pwr = QtWidgets.QDoubleSpinBox()
        self.wiz_pwr.setRange(0.0, 50.0)
        self.wiz_pwr.setValue(1.50)

        btn_add_comp = QtWidgets.QPushButton("Generate & Place Component on Board")
        btn_add_comp.clicked.connect(self._on_wizard_place_component)

        fl.addRow("Reference Designator:", self.wiz_refdes)
        fl.addRow("Component Value / Part:", self.wiz_val)
        fl.addRow("Package Family:", self.wiz_pkg)
        fl.addRow("Pin Count (QFN/SOIC):", self.wiz_pins)
        fl.addRow("Center X (mm):", self.wiz_x)
        fl.addRow("Center Y (mm):", self.wiz_y)
        fl.addRow("Thermal Dissipation (W):", self.wiz_pwr)
        fl.addRow(btn_add_comp)

        self.tbl_comps = QtWidgets.QTableWidget(0, 6)
        self.tbl_comps.setHorizontalHeaderLabels(["RefDes", "Package", "Value", "Pos (mm)", "Power (W)", "Pad Count"])
        self._refresh_component_table()

        layout.addWidget(grp, 1)
        layout.addWidget(self.tbl_comps, 2)
        self.tabs.addTab(tab, "5. Footprint Wizard")

    def _on_wizard_place_component(self) -> None:
        self._record_undo()
        ref = self.wiz_refdes.text().strip() or f"U{len(self.board.components) + 1}"
        val = self.wiz_val.text().strip()
        pkg = self.wiz_pkg.currentText()
        x, y, pwr = self.wiz_x.value(), self.wiz_y.value(), self.wiz_pwr.value()

        if pkg.startswith("QFN"):
            pps = max(2, self.wiz_pins.value() // 4)
            comp = FootprintWizard.create_qfn(ref, val, x, y, 7.0, pps, 0.8, pwr)
        elif pkg == "SOIC":
            comp = FootprintWizard.create_soic(ref, val, x, y, self.wiz_pins.value(), 1.27, 4.0, pwr)
        else:
            comp = FootprintWizard.create_smd_2pin(ref, val, x, y, pkg, "VOUT_3V3", "GND", pwr)

        self.board.components[comp.comp_id] = comp
        self._refresh_component_table()
        self._refresh_netlist_text()
        self.sch_canvas.update()
        self.canvas.update()
        self.viewer_3d.update()
        self.status_bar.showMessage(f"Placed {ref} ({comp.package_name}) at ({x:.1f}, {y:.1f}) mm.")

    def _refresh_component_table(self) -> None:
        self.tbl_comps.setRowCount(0)
        for comp in self.board.components.values():
            r = self.tbl_comps.rowCount()
            self.tbl_comps.insertRow(r)
            self.tbl_comps.setItem(r, 0, QtWidgets.QTableWidgetItem(comp.refdes))
            self.tbl_comps.setItem(r, 1, QtWidgets.QTableWidgetItem(comp.package_name))
            self.tbl_comps.setItem(r, 2, QtWidgets.QTableWidgetItem(comp.value))
            self.tbl_comps.setItem(r, 3, QtWidgets.QTableWidgetItem(f"({comp.x_mm:.1f}, {comp.y_mm:.1f})"))
            self.tbl_comps.setItem(r, 4, QtWidgets.QTableWidgetItem(f"{comp.power_w:.2f}"))
            self.tbl_comps.setItem(r, 5, QtWidgets.QTableWidgetItem(str(len(comp.pads))))

    # ------------------------------------------------------------------------
    # TAB 6: PHYSICAL STACKUP & MATERIALS MANAGER
    # ------------------------------------------------------------------------
    def _build_tab6_stackup_editor(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        self.tbl_stackup = QtWidgets.QTableWidget(0, 6)
        self.tbl_stackup.setHorizontalHeaderLabels([
            "Layer / Core", "Type", "Thickness (um)", "Conductivity / Er", "Thermal k (W/mK)", "Notes"
        ])
        self._refresh_stackup_table()
        layout.addWidget(self.tbl_stackup)
        self.tabs.addTab(tab, "6. Physical Stackup")

    def _refresh_stackup_table(self) -> None:
        self.tbl_stackup.setRowCount(0)
        for i, lay in enumerate(self.board.layers):
            r = self.tbl_stackup.rowCount()
            self.tbl_stackup.insertRow(r)
            self.tbl_stackup.setItem(r, 0, QtWidgets.QTableWidgetItem(f"L{lay.display_number}: {lay.name}"))
            self.tbl_stackup.setItem(r, 1, QtWidgets.QTableWidgetItem("COPPER"))
            self.tbl_stackup.setItem(r, 2, QtWidgets.QTableWidgetItem(f"{lay.thickness_um:.1f}"))
            self.tbl_stackup.setItem(r, 3, QtWidgets.QTableWidgetItem(f"{lay.conductivity_s_m:.2e} S/m"))
            self.tbl_stackup.setItem(r, 4, QtWidgets.QTableWidgetItem(f"{lay.thermal_k_w_mk:.1f}"))
            self.tbl_stackup.setItem(r, 5, QtWidgets.QTableWidgetItem("1 oz Rolled/ED Copper"))

            if i < len(self.board.dielectrics):
                d = self.board.dielectrics[i]
                rd = self.tbl_stackup.rowCount()
                self.tbl_stackup.insertRow(rd)
                self.tbl_stackup.setItem(rd, 0, QtWidgets.QTableWidgetItem(d.name))
                self.tbl_stackup.setItem(rd, 1, QtWidgets.QTableWidgetItem("DIELECTRIC"))
                self.tbl_stackup.setItem(rd, 2, QtWidgets.QTableWidgetItem(f"{d.thickness_um:.1f}"))
                self.tbl_stackup.setItem(rd, 3, QtWidgets.QTableWidgetItem(f"Er = {d.er:.2f}, tanD = {d.loss_tangent:.3f}"))
                self.tbl_stackup.setItem(rd, 4, QtWidgets.QTableWidgetItem(f"{d.thermal_k_w_mk:.2f}"))
                self.tbl_stackup.setItem(rd, 5, QtWidgets.QTableWidgetItem("FR4 / Prepreg Isolation"))

    # ------------------------------------------------------------------------
    # TAB 7: DC IR-DROP & CURRENT DENSITY SOLVER
    # ------------------------------------------------------------------------
    def _build_tab7_dc_ir_drop(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        self.tbl_dc_traces = QtWidgets.QTableWidget(0, 6)
        self.tbl_dc_traces.setHorizontalHeaderLabels([
            "Trace / Via ID", "Net", "Current (A)", "Current Density (A/mm^2)", "Joule Loss (mW)", "IPC-2152 Status"
        ])
        self.plt_dc_voltages = pg.PlotWidget(title="Solved Node / Pad DC Voltages (V)")
        self.plt_dc_voltages.setLabel("left", "Voltage", units="V")
        self.plt_dc_voltages.setLabel("bottom", "Pad Index")

        layout.addWidget(self.tbl_dc_traces, 3)
        layout.addWidget(self.plt_dc_voltages, 2)
        self.tabs.addTab(tab, "7. DC IR-Drop")

    # ------------------------------------------------------------------------
    # TAB 8: COUPLED 3D ELECTROTHERMAL FEA SOLVER
    # ------------------------------------------------------------------------
    def _build_tab8_electrothermal_fea(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        left_v = QtWidgets.QVBoxLayout()
        grp_bc = QtWidgets.QGroupBox("Thermal Boundary Conditions")
        fl = QtWidgets.QFormLayout(grp_bc)
        self.spn_tamb = QtWidgets.QDoubleSpinBox()
        self.spn_tamb.setRange(-40.0, 125.0)
        self.spn_tamb.setValue(self.board.ambient_temp_c)
        self.spn_hconv = QtWidgets.QDoubleSpinBox()
        self.spn_hconv.setRange(2.0, 250.0)
        self.spn_hconv.setValue(self.board.convection_h_w_m2k)
        btn_rerun_th = QtWidgets.QPushButton("Solve Coupled Electrothermal Mesh")
        btn_rerun_th.clicked.connect(self._run_electrothermal_solver)
        fl.addRow("Ambient Temperature (C):", self.spn_tamb)
        fl.addRow("Convection Coeff h (W/m^2K):", self.spn_hconv)
        fl.addRow(btn_rerun_th)
        left_v.addWidget(grp_bc)

        self.txt_thermal_report = QtWidgets.QTextEdit()
        self.txt_thermal_report.setReadOnly(True)
        left_v.addWidget(self.txt_thermal_report, 1)

        right_plots = pg.GraphicsLayoutWidget()
        self.p_top_heat = right_plots.addPlot(title="Top Copper Steady-State Temperature Field (C)")
        self.p_top_heat.setLabel("bottom", "X (mm)")
        self.p_top_heat.setLabel("left", "Y (mm)")
        self.img_top_heat = pg.ImageItem()
        self.p_top_heat.addItem(self.img_top_heat)
        cmap = pg.colormap.get("inferno")
        self.img_top_heat.setLookupTable(cmap.getLookupTable())

        right_plots.nextRow()
        self.p_bot_heat = right_plots.addPlot(title="Bottom Copper Steady-State Temperature Field (C)")
        self.p_bot_heat.setLabel("bottom", "X (mm)")
        self.p_bot_heat.setLabel("left", "Y (mm)")
        self.img_bot_heat = pg.ImageItem()
        self.p_bot_heat.addItem(self.img_bot_heat)
        self.img_bot_heat.setLookupTable(cmap.getLookupTable())

        layout.addLayout(left_v, 1)
        layout.addWidget(right_plots, 3)
        self.tabs.addTab(tab, "8. 3D Thermal FEA")

    # ------------------------------------------------------------------------
    # TAB 9: HIGH-SPEED SIGNAL INTEGRITY & IMPEDANCE CALCULATOR
    # ------------------------------------------------------------------------
    def _build_tab9_signal_integrity(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        grp = QtWidgets.QGroupBox("Transmission Line Parameters")
        fl = QtWidgets.QFormLayout(grp)
        self.si_mode = QtWidgets.QComboBox()
        self.si_mode.addItems(["Microstrip", "Stripline"])
        self.si_w = QtWidgets.QDoubleSpinBox()
        self.si_w.setRange(0.05, 5.0)
        self.si_w.setSingleStep(0.02)
        self.si_w.setValue(0.32)
        self.si_s = QtWidgets.QDoubleSpinBox()
        self.si_s.setRange(0.05, 5.0)
        self.si_s.setSingleStep(0.02)
        self.si_s.setValue(0.20)
        self.si_h = QtWidgets.QDoubleSpinBox()
        self.si_h.setRange(25.0, 3000.0)
        self.si_h.setValue(200.0)
        self.si_len = QtWidgets.QDoubleSpinBox()
        self.si_len.setRange(1.0, 500.0)
        self.si_len.setValue(45.0)

        for w in (self.si_mode, self.si_w, self.si_s, self.si_h, self.si_len):
            if isinstance(w, QtWidgets.QComboBox):
                w.currentTextChanged.connect(self._update_si_tab)
            else:
                w.valueChanged.connect(self._update_si_tab)

        self.txt_si_out = QtWidgets.QTextEdit()
        self.txt_si_out.setReadOnly(True)

        fl.addRow("Geometry Topology:", self.si_mode)
        fl.addRow("Trace Width W (mm):", self.si_w)
        fl.addRow("Diff-Pair Spacing S (mm):", self.si_s)
        fl.addRow("Dielectric Height H (um):", self.si_h)
        fl.addRow("Route Length (mm):", self.si_len)
        fl.addRow("Synthesized Results:", self.txt_si_out)

        self.plt_si = pg.PlotWidget(title="High-Speed Insertion Loss |S21(f)| (Skin Effect + Dielectric Loss)")
        self.plt_si.setLabel("bottom", "Frequency", units="GHz")
        self.plt_si.setLabel("left", "Insertion Loss", units="dB")
        self.plt_si.showGrid(x=True, y=True, alpha=0.3)

        layout.addWidget(grp, 1)
        layout.addWidget(self.plt_si, 2)
        self.tabs.addTab(tab, "9. Signal Integrity")
        self._update_si_tab()

    def _update_si_tab(self) -> None:
        res = PhysicsEngine.compute_transmission_line(
            self.si_mode.currentText(),
            self.si_w.value(),
            self.si_s.value(),
            self.si_h.value(),
            35.0,
            4.2,
            0.016,
            self.si_len.value(),
        )
        self.txt_si_out.setPlainText(
            f"Single-Ended Z0     : {res['z0_ohm']:.2f} Ohm\n"
            f"Differential Z_diff : {res['z_diff_ohm']:.2f} Ohm\n"
            f"Odd / Even Mode     : {res['z_odd_ohm']:.2f} / {res['z_even_ohm']:.2f} Ohm\n"
            f"Effective Er_eff    : {res['er_eff']:.3f}\n"
            f"Propagation Delay   : {res['delay_ps']:.1f} ps\n"
            f"Coupled NEXT Est.   : {res['next_db']:.1f} dB\n"
            f"Loss @ 10 GHz       : {res['s21_db'][100]:.2f} dB"
        )
        self.plt_si.clear()
        self.plt_si.plot(res["freqs_ghz"], res["s21_db"], pen=pg.mkPen("#38b866", width=2.5))

    # ------------------------------------------------------------------------
    # TAB 10: PDN DECOUPLING & CAVITY RESONANCE ANALYZER
    # ------------------------------------------------------------------------
    def _build_tab10_pdn_analyzer(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)

        left_v = QtWidgets.QVBoxLayout()
        self.txt_pdn_modes = QtWidgets.QTextEdit()
        self.txt_pdn_modes.setReadOnly(True)
        left_v.addWidget(QtWidgets.QLabel("Power/Ground Plane Cavity Resonances & Decap Bank:"))
        left_v.addWidget(self.txt_pdn_modes)

        self.plt_pdn = pg.PlotWidget(title="PDN Impedance Profile |Z_PDN(f)| vs Target Impedance")
        self.plt_pdn.setLogMode(x=True, y=True)
        self.plt_pdn.setLabel("bottom", "Frequency (Hz)")
        self.plt_pdn.setLabel("left", "Impedance |Z| (Ohm)")
        self.plt_pdn.showGrid(x=True, y=True, alpha=0.3)

        layout.addLayout(left_v, 1)
        layout.addWidget(self.plt_pdn, 2)
        self.tabs.addTab(tab, "10. PDN & Decoupling")
        self._update_pdn_tab()

    def _update_pdn_tab(self) -> None:
        freqs = np.logspace(3, 9, 400)
        w = 2.0 * np.pi * freqs

        area_m2 = (self.board.width_mm * 1e-3) * (self.board.height_mm * 1e-3)
        c_plane = 8.854e-12 * 4.2 * area_m2 / (200e-6)
        y_total = 1j * w * c_plane

        for stage in self.board.decap_bank:
            c = stage.c_uf * 1e-6
            r = stage.esr_mohm * 1e-3
            l = stage.esl_nh * 1e-9
            z_stage = (r + 1j * w * l + 1.0 / (1j * w * c)) / max(1, stage.count)
            y_total += 1.0 / z_stage

        z_pdn = np.abs(1.0 / y_total)
        z_target = np.full_like(freqs, 0.050)

        self.plt_pdn.clear()
        self.plt_pdn.plot(freqs, z_pdn, pen=pg.mkPen("#58a6ff", width=2.5), name="|Z_PDN(f)|")
        self.plt_pdn.plot(freqs, z_target, pen=pg.mkPen("#ff6b57", width=2.0, style=DashLine), name="Z_target (50 mOhm)")

        c0 = 2.99792458e8
        er = 4.4
        lx = self.board.width_mm * 1e-3
        ly = self.board.height_mm * 1e-3
        lines = [f"Plane Parallel Capacitance: {c_plane * 1e12:.1f} pF\n", "Bare-Board Cavity Resonant Modes (TM_mn):"]
        for m in range(0, 3):
            for n in range(0, 3):
                if m == 0 and n == 0:
                    continue
                f_mn = (c0 / (2.0 * math.sqrt(er))) * math.hypot(m / lx, n / ly)
                lines.append(f"  Mode TM_{m}{n} : {f_mn * 1e-9:.3f} GHz")
        self.txt_pdn_modes.setPlainText("\n".join(lines))

    # ------------------------------------------------------------------------
    # TAB 11: FABRICATION CAM EXPORTER
    # ------------------------------------------------------------------------
    def _build_tab11_fabrication_export(self) -> None:
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        top_h = QtWidgets.QHBoxLayout()
        self.ed_export_dir = QtWidgets.QLineEdit(str(pathlib.Path("./CAM_Fabrication_Output").resolve()))
        btn_exp = QtWidgets.QPushButton("Export Complete Manufacturing Package (Gerber + Excellon + BOM + CPL)")
        btn_exp.clicked.connect(self._on_export_cam)
        top_h.addWidget(QtWidgets.QLabel("Output Folder:"))
        top_h.addWidget(self.ed_export_dir, 1)
        top_h.addWidget(btn_exp)

        self.txt_cam_preview = QtWidgets.QTextEdit()
        self.txt_cam_preview.setFont(QtGui.QFont("Consolas", 9))

        layout.addLayout(top_h)
        layout.addWidget(self.txt_cam_preview, 1)
        self.tabs.addTab(tab, "11. CAM Export")

    # ------------------------------------------------------------------------
    # GLOBAL ACTIONS & CALLBACKS
    # ------------------------------------------------------------------------
    def _on_undo(self) -> None:
        if len(self.undo_stack) > 1:
            self.redo_stack.append(self.undo_stack.pop())
            self.board = BoardProject.from_json(self.undo_stack[-1])
            self.sch_canvas.board = self.board
            self.canvas.board = self.board
            self.viewer_3d.board = self.board
            self._refresh_component_table()
            self._refresh_netlist_text()
            self.sch_canvas.update()
            self.canvas.update()
            self.viewer_3d.update()
            self.status_bar.showMessage("Undo applied.")

    def _on_save_project(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save PCB Project", f"{self.board.name}.pcbproj", "PCB Project (*.pcbproj)")
        if path:
            pathlib.Path(path).write_text(self.board.to_json(), encoding="utf-8")
            self.status_bar.showMessage(f"Saved project to {path}")

    def _on_load_project(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load PCB Project", "", "PCB Project (*.pcbproj)")
        if path:
            self.board = BoardProject.from_json(pathlib.Path(path).read_text(encoding="utf-8"))
            self.sch_canvas.board = self.board
            self.canvas.board = self.board
            self.viewer_3d.board = self.board
            self._refresh_component_table()
            self._refresh_netlist_text()
            self._run_electrothermal_solver()
            self.sch_canvas.update()
            self.canvas.update()
            self.viewer_3d.update()

    def _on_import_kicad(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import KiCad Board", "", "KiCad PCB (*.kicad_pcb)")
        if path:
            try:
                self._record_undo()
                imported = CADFormatImporter.import_kicad_pcb(path)
                self.board = imported
                self.sch_canvas.board = self.board
                self.canvas.board = self.board
                self.viewer_3d.board = self.board
                self._refresh_component_table()
                self._refresh_netlist_text()
                self._run_electrothermal_solver()
                self.sch_canvas.update()
                self.canvas.update()
                self.viewer_3d.update()
                self.status_bar.showMessage(f"Imported KiCad PCB: {len(self.board.components)} components, {len(self.board.traces)} traces.")
            except Exception as exc:
                QtWidgets.QMessageBox.critical(self, "KiCad Import Error", str(exc))

    def _on_import_dxf(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import DXF Outline", "", "AutoCAD DXF (*.dxf)")
        if path:
            try:
                self._record_undo()
                count = CADFormatImporter.import_dxf_outline(path, self.board)
                self.canvas.update()
                self.viewer_3d.update()
                self.status_bar.showMessage(f"Imported DXF Outline: {count} boundary points loaded.")
            except Exception as exc:
                QtWidgets.QMessageBox.critical(self, "DXF Import Error", str(exc))

    def _on_autoroute(self) -> None:
        self._record_undo()
        routed = AStarAutorouter.autoroute_all(self.board, grid_step_mm=0.5, trace_width_mm=self.spn_trace_w.value())
        self._on_run_drc()
        self._run_electrothermal_solver()
        self.canvas.update()
        self.viewer_3d.update()
        self.status_bar.showMessage(f"A* Autorouter completed: routed {routed} connections.")

    def _on_run_drc(self) -> None:
        viols = DRCEngine.check_board(self.board)
        self.canvas.drc_violations = viols
        unrouted = AStarAutorouter.compute_ratsnest(self.board)
        lines = [
            f"=== DRC & CONNECTIVITY REPORT ===",
            f"Geometric Violations : {len(viols)}",
            f"Unrouted Airwires    : {len(unrouted)}",
            ""
        ]
        for v in viols:
            lines.append(f"[{v.code}] {v.message} @ ({v.x_mm:.2f}, {v.y_mm:.2f}) mm")
        for net, x1, y1, x2, y2, lid in unrouted:
            lines.append(f"[UNROUTED] Net {net} ({lid}) from ({x1:.1f}, {y1:.1f}) to ({x2:.1f}, {y2:.1f}) mm")
        self.txt_drc_log.setPlainText("\n".join(lines))
        self.canvas.update()

    def _run_electrothermal_solver(self) -> None:
        self.board.ambient_temp_c = self.spn_tamb.value()
        self.board.convection_h_w_m2k = self.spn_hconv.value()
        res = PhysicsEngine.solve_coupled_electrothermal(self.board)
        self.canvas.last_result = res
        self.viewer_3d.last_result = res

        self.tbl_dc_traces.setRowCount(0)
        for tid, curr in res.trace_currents_a.items():
            tr = self.board.traces[tid]
            j_dens = res.trace_current_density_a_mm2[tid]
            loss_mw = res.trace_losses_w[tid] * 1000.0
            status = "PASS (<35 A/mm^2)" if j_dens < 35.0 else "HIGH CURRENT DENSITY"
            r = self.tbl_dc_traces.rowCount()
            self.tbl_dc_traces.insertRow(r)
            self.tbl_dc_traces.setItem(r, 0, QtWidgets.QTableWidgetItem(tid))
            self.tbl_dc_traces.setItem(r, 1, QtWidgets.QTableWidgetItem(tr.net_name))
            self.tbl_dc_traces.setItem(r, 2, QtWidgets.QTableWidgetItem(f"{curr:.3f}"))
            self.tbl_dc_traces.setItem(r, 3, QtWidgets.QTableWidgetItem(f"{j_dens:.2f}"))
            self.tbl_dc_traces.setItem(r, 4, QtWidgets.QTableWidgetItem(f"{loss_mw:.2f}"))
            self.tbl_dc_traces.setItem(r, 5, QtWidgets.QTableWidgetItem(status))

        self.plt_dc_voltages.clear()
        v_list = list(res.pad_voltages_v.values())
        if v_list:
            bg = pg.BarGraphItem(x=np.arange(len(v_list)), height=v_list, width=0.6, brush="#3b82f6")
            self.plt_dc_voltages.addItem(bg)

        self.img_top_heat.setImage(res.top_temp_map_c)
        self.img_top_heat.setRect(QRectF(0, 0, self.board.width_mm, self.board.height_mm))
        self.img_bot_heat.setImage(res.bot_temp_map_c)
        self.img_bot_heat.setRect(QRectF(0, 0, self.board.width_mm, self.board.height_mm))

        t_lines = [
            f"=== COUPLED 3D ELECTROTHERMAL REPORT ===",
            f"Coupled Iterations     : {res.iterations} (max dT = {res.max_delta_t_c:.4e} C)",
            f"Component Power Total  : {res.total_comp_w:.3f} W",
            f"Trace/Via Joule Losses : {res.total_joule_w * 1000.0:.2f} mW",
            f"Peak Top Copper Temp   : {np.max(res.top_temp_map_c):.2f} C",
            f"Peak Bot Copper Temp   : {np.max(res.bot_temp_map_c):.2f} C",
            "",
            "Component Junction Temperatures (Tj):"
        ]
        for ref, tj in res.comp_tj_c.items():
            tb = res.comp_tboard_c[ref]
            t_lines.append(f"  {ref:6s} : Tj = {tj:6.2f} C | T_board = {tb:6.2f} C")
        self.txt_thermal_report.setPlainText("\n".join(t_lines))
        self._on_run_drc()
        self.viewer_3d.update()

    def _on_export_cam(self) -> None:
        out_dir = self.ed_export_dir.text().strip()
        files = FabricationCAMExporter.export_all(self.board, out_dir)
        preview = [f"Exported {len(files)} manufacturing files to {out_dir}:\n"]
        for f in files:
            preview.append(f"  -> {f}")
        if files:
            sample = pathlib.Path(files[0]).read_text(encoding="utf-8")
            preview.append("\n--- SAMPLE GERBER RS-274X OUTPUT (TOP LAYER) ---")
            preview.append(sample[:2000])
        self.txt_cam_preview.setPlainText("\n".join(preview))
        self.status_bar.showMessage(f"Exported {len(files)} CAM files to {out_dir}")


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = PCBMasterSuiteWindow()
    win.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())


if __name__ == "__main__":
    main()