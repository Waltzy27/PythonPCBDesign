import { BoardProject, Component, mkPad, mkTrace, mkVia, Pad } from "./model";

const r2 = (v: number) => Math.round(v * 100) / 100;

export function createQfn(refdes: string, value: string, x: number, y: number, body = 7, pps = 4, pitch = 0.8, power = 2.4, layer = "L1", schX = 520, schY = 220): Component {
  const pads: Pad[] = [];
  const total = pps * 4, span = (pps - 1) * pitch, hb = body / 2;
  let n = 1;
  const add = (px: number, py: number, w: number, h: number) => { pads.push(mkPad(`${refdes}_P${n}`, String(n), `IO${n}`, px, py, w, h, layer)); n++; };
  for (let i = 0; i < pps; i++) add(-hb, -span / 2 + i * pitch, 0.8, 0.4);
  for (let i = 0; i < pps; i++) add(-span / 2 + i * pitch, hb, 0.4, 0.8);
  for (let i = 0; i < pps; i++) add(hb, span / 2 - i * pitch, 0.8, 0.4);
  for (let i = 0; i < pps; i++) add(span / 2 - i * pitch, -hb, 0.4, 0.8);
  const ep = r2(body * 0.55);
  pads.push(mkPad(`${refdes}_EP`, String(total + 1), "EP_GND", 0, 0, ep, ep, layer, { net_name: "GND", is_thermal_pad: true }));
  return { comp_id: `COMP_${refdes}`, refdes, package_name: `QFN-${total}_EP`, value, x_mm: x, y_mm: y, width_mm: body, height_mm: body, rotation_deg: 0, layer_id: layer, power_w: power, r_theta_jb: 6.5, r_theta_ja: 32, max_tj_c: 105, height_3d_mm: 1.2, sch_x: schX, sch_y: schY, pads };
}

export function createSoic(refdes: string, value: string, x: number, y: number, pinCount = 8, pitch = 1.27, bodyW = 4, power = 1.2, layer = "L1", schX = 240, schY = 220): Component {
  const half = Math.max(1, Math.floor(pinCount / 2)), span = (half - 1) * pitch, bodyH = Math.max(3, span + 1.8);
  const pads: Pad[] = [];
  for (let i = 0; i < half; i++) { const n = i + 1; pads.push(mkPad(`${refdes}_P${n}`, String(n), `P${n}`, -bodyW / 2, -span / 2 + i * pitch, 1.4, 0.6, layer)); }
  for (let i = 0; i < half; i++) { const n = half + i + 1; pads.push(mkPad(`${refdes}_P${n}`, String(n), `P${n}`, bodyW / 2, span / 2 - i * pitch, 1.4, 0.6, layer)); }
  return { comp_id: `COMP_${refdes}`, refdes, package_name: `SOIC-${pinCount}`, value, x_mm: x, y_mm: y, width_mm: bodyW, height_mm: bodyH, rotation_deg: 0, layer_id: layer, power_w: power, r_theta_jb: 11, r_theta_ja: 48, max_tj_c: 105, height_3d_mm: 1.75, sch_x: schX, sch_y: schY, pads };
}

const DIMS: Record<string, number[]> = {
  "0402": [1.0, 0.5, 0.5, 0.5, 0.6, 0.5], "0603": [1.6, 0.8, 0.8, 0.8, 0.9, 0.8], "0805": [2.0, 1.25, 1.0, 1.25, 1.1, 1.1],
  "1206": [3.2, 1.6, 1.6, 1.6, 1.5, 1.3], IND_6x6: [6.0, 6.0, 2.2, 2.0, 4.5, 4.2],
};

export function createSmd2pin(refdes: string, value: string, x: number, y: number, pkg = "0805", net1 = "", net2 = "", power = 0.05, layer = "L1", schX = 100, schY = 100): Component {
  const [bw, bh, dx, pw, ph, h3d] = DIMS[pkg] ?? DIMS["0805"];
  const pads = [
    mkPad(`${refdes}_P1`, "1", "1", -dx, 0, pw, ph, layer, { net_name: net1 }),
    mkPad(`${refdes}_P2`, "2", "2", dx, 0, pw, ph, layer, { net_name: net2 }),
  ];
  return { comp_id: `COMP_${refdes}`, refdes, package_name: pkg, value, x_mm: x, y_mm: y, width_mm: bw, height_mm: bh, rotation_deg: 0, layer_id: layer, power_w: power, r_theta_jb: 18, r_theta_ja: 65, max_tj_c: 105, height_3d_mm: h3d, sch_x: schX, sch_y: schY, pads };
}

export function buildDefaultDemoBoard(): BoardProject {
  const b = new BoardProject();
  const cu = (id: string, name: string, n: number, color: string) => ({ layer_id: id, name, display_number: n, thickness_um: 35, conductivity_s_m: 5.8e7, tempco_per_k: 0.00393, thermal_k_w_mk: 385, color_hex: color, visible: true });
  b.layers = [cu("L1", "Top_Signal_Power", 1, "#e64a4a"), cu("L2", "Inner_GND_Plane", 2, "#38b866"), cu("L3", "Inner_Power_Plane", 3, "#e39b27"), cu("L4", "Bottom_Signal", 4, "#4692f0")];
  b.dielectrics = [
    { diel_id: "D1", name: "Prepreg_2116", thickness_um: 200, er: 4.2, loss_tangent: 0.016, thermal_k_w_mk: 0.35 },
    { diel_id: "D2", name: "FR4_Core_1.1mm", thickness_um: 1100, er: 4.5, loss_tangent: 0.02, thermal_k_w_mk: 0.3 },
    { diel_id: "D3", name: "Prepreg_2116", thickness_um: 200, er: 4.2, loss_tangent: 0.016, thermal_k_w_mk: 0.35 },
  ];
  const j1 = createSmd2pin("J1", "12V_IN_CONN", 12, 26, "1206", "VIN_12V", "GND", 0.05, "L1", 90, 210);
  const u1 = createSoic("U1", "TPS5430_BUCK", 28, 26, 8, 1.27, 4, 1.85, "L1", 290, 210);
  u1.pads[0].net_name = "VIN_12V"; u1.pads[1].net_name = "GND"; u1.pads[4].net_name = "SW_NODE"; u1.pads[7].net_name = "VOUT_3V3";
  const l1 = createSmd2pin("L1", "4.7uH_5A", 43, 26, "IND_6x6", "SW_NODE", "VOUT_3V3", 0.45, "L1", 480, 150);
  const u2 = createQfn("U2", "STM32H7_FPGA", 62, 26, 8, 4, 1.2, 3.2, "L1", 690, 230);
  u2.pads[0].net_name = "VOUT_3V3"; u2.pads[1].net_name = "DATA_0"; u2.pads[2].net_name = "DATA_1"; u2.pads[3].net_name = "CLK_100M"; u2.pads[5].net_name = "GND";
  const c1 = createSmd2pin("C1", "10uF_25V", 20, 16, "0805", "VIN_12V", "GND", 0.01, "L1", 185, 360);
  const c2 = createSmd2pin("C2", "22uF_10V", 52, 16, "0805", "VOUT_3V3", "GND", 0.01, "L1", 545, 360);
  for (const c of [j1, u1, l1, u2, c1, c2]) b.components[c.comp_id] = c;
  const W = (id: string, a: string, ap: string, bb: string, bp: string, net: string) => (b.sch_wires[id] = { wire_id: id, from_comp_id: a, from_pin_number: ap, to_comp_id: bb, to_pin_number: bp, net_name: net });
  W("W1", "COMP_J1", "1", "COMP_U1", "1", "VIN_12V"); W("W2", "COMP_C1", "1", "COMP_U1", "1", "VIN_12V");
  W("W3", "COMP_U1", "5", "COMP_L1", "1", "SW_NODE"); W("W4", "COMP_L1", "2", "COMP_U2", "1", "VOUT_3V3");
  W("W5", "COMP_C2", "1", "COMP_U2", "1", "VOUT_3V3"); W("W6", "COMP_J1", "2", "COMP_U1", "2", "GND");
  W("W7", "COMP_C1", "2", "COMP_C2", "2", "GND");
  const P = b.allWorldPads();
  const pos = (id: string) => P.get(id)!;
  const j1p1 = pos("J1_P1"), u1p1 = pos("U1_P1"), u1p5 = pos("U1_P5"), l1p1 = pos("L1_P1"), l1p2 = pos("L1_P2"), u2p1 = pos("U2_P1");
  b.traces["TR_VIN_1"] = mkTrace("TR_VIN_1", "VIN_12V", "L1", j1p1.x, j1p1.y, u1p1.x, j1p1.y, 0.8);
  b.traces["TR_VIN_2"] = mkTrace("TR_VIN_2", "VIN_12V", "L1", u1p1.x, j1p1.y, u1p1.x, u1p1.y, 0.8);
  b.traces["TR_SW_1"] = mkTrace("TR_SW_1", "SW_NODE", "L1", u1p5.x, u1p5.y, l1p1.x, l1p1.y, 1.0);
  b.vias["VIA_VOUT_1"] = mkVia("VIA_VOUT_1", "VOUT_3V3", 49.5, 26, 0.8, 0.4);
  b.vias["VIA_VOUT_2"] = mkVia("VIA_VOUT_2", "VOUT_3V3", 55.5, 24.2, 0.8, 0.4);
  b.traces["TR_VOUT_1"] = mkTrace("TR_VOUT_1", "VOUT_3V3", "L1", l1p2.x, l1p2.y, 49.5, 26, 0.9);
  b.traces["TR_VOUT_L4"] = mkTrace("TR_VOUT_L4", "VOUT_3V3", "L4", 49.5, 26, 55.5, 24.2, 0.9);
  b.traces["TR_VOUT_2"] = mkTrace("TR_VOUT_2", "VOUT_3V3", "L1", 55.5, 24.2, u2p1.x, u2p1.y, 0.9);
  for (const dx of [-1.2, 1.2]) for (const dy of [-1.2, 1.2]) { const id = `VIA_TH_${dx}_${dy}`; b.vias[id] = mkVia(id, "GND", 62 + dx, 26 + dy, 0.6, 0.3); }
  b.zones["ZONE_GND_L2"] = { zone_id: "ZONE_GND_L2", net_name: "GND", layer_id: "L2", points_mm: [[3, 3], [77, 3], [77, 52], [3, 52]], is_keepout: false };
  b.dc_sources = { J1_P1: 12, L1_P1: 3.35, J1_P2: 0, U2_EP: 0 };
  b.dc_sinks = { U1_P1: 0.85, U2_P1: 2.6 };
  return b;
}
