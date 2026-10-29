import { uid } from "./util";

export interface CopperLayer { layer_id: string; name: string; display_number: number; thickness_um: number; conductivity_s_m: number; tempco_per_k: number; thermal_k_w_mk: number; color_hex: string; visible: boolean; }
export interface DielectricLayer { diel_id: string; name: string; thickness_um: number; er: number; loss_tangent: number; thermal_k_w_mk: number; }
export interface Pad { pad_id: string; pin_number: string; pin_name: string; rel_x_mm: number; rel_y_mm: number; width_mm: number; height_mm: number; layer_id: string; shape: string; drill_mm: number; net_name: string; is_thermal_pad: boolean; }
export interface Component { comp_id: string; refdes: string; package_name: string; value: string; x_mm: number; y_mm: number; width_mm: number; height_mm: number; rotation_deg: number; layer_id: string; power_w: number; r_theta_jb: number; r_theta_ja: number; max_tj_c: number; height_3d_mm: number; sch_x: number; sch_y: number; pads: Pad[]; }
export interface SchematicWire { wire_id: string; from_comp_id: string; from_pin_number: string; to_comp_id: string; to_pin_number: string; net_name: string; }
export interface Trace { trace_id: string; net_name: string; layer_id: string; x1_mm: number; y1_mm: number; x2_mm: number; y2_mm: number; width_mm: number; }
export interface Via { via_id: string; net_name: string; x_mm: number; y_mm: number; outer_dia_mm: number; drill_dia_mm: number; plating_um: number; }
export interface CopperZone { zone_id: string; net_name: string; layer_id: string; points_mm: [number, number][]; is_keepout: boolean; }
export interface DecapStage { name: string; count: number; c_uf: number; esr_mohm: number; esl_nh: number; }
export interface DesignRules { min_clearance_mm: number; min_trace_width_mm: number; min_via_drill_mm: number; min_annular_ring_mm: number; edge_clearance_mm: number; }

export const mkPad = (pad_id: string, pin_number: string, pin_name: string, rel_x_mm: number, rel_y_mm: number, width_mm: number, height_mm: number, layer_id: string, o: Partial<Pad> = {}): Pad =>
  ({ pad_id, pin_number, pin_name, rel_x_mm, rel_y_mm, width_mm, height_mm, layer_id, shape: "RECT", drill_mm: 0, net_name: "", is_thermal_pad: false, ...o });

export const mkTrace = (trace_id: string, net_name: string, layer_id: string, x1_mm: number, y1_mm: number, x2_mm: number, y2_mm: number, width_mm = 0.35): Trace =>
  ({ trace_id, net_name, layer_id, x1_mm, y1_mm, x2_mm, y2_mm, width_mm });

export const mkVia = (via_id: string, net_name: string, x_mm: number, y_mm: number, outer_dia_mm = 0.7, drill_dia_mm = 0.35): Via =>
  ({ via_id, net_name, x_mm, y_mm, outer_dia_mm, drill_dia_mm, plating_um: 25 });

export const traceLength = (t: Trace): number => Math.hypot(t.x2_mm - t.x1_mm, t.y2_mm - t.y1_mm);

export function padWorldPos(c: Component, p: Pad): [number, number] {
  const rad = (c.rotation_deg * Math.PI) / 180, ca = Math.cos(rad), sa = Math.sin(rad);
  return [c.x_mm + (p.rel_x_mm * ca - p.rel_y_mm * sa), c.y_mm + (p.rel_x_mm * sa + p.rel_y_mm * ca)];
}

export class BoardProject {
  name = "Mixed_Signal_Power_Board";
  width_mm = 80;
  height_mm = 55;
  rules: DesignRules = { min_clearance_mm: 0.15, min_trace_width_mm: 0.15, min_via_drill_mm: 0.2, min_annular_ring_mm: 0.12, edge_clearance_mm: 0.25 };
  layers: CopperLayer[] = [];
  dielectrics: DielectricLayer[] = [];
  nets = new Set<string>(["GND", "VIN_12V", "VOUT_3V3", "SW_NODE", "CLK_100M", "DATA_0", "DATA_1"]);
  components: Record<string, Component> = {};
  sch_wires: Record<string, SchematicWire> = {};
  traces: Record<string, Trace> = {};
  vias: Record<string, Via> = {};
  zones: Record<string, CopperZone> = {};
  dc_sources: Record<string, number> = {};
  dc_sinks: Record<string, number> = {};
  ambient_temp_c = 25;
  convection_h_w_m2k = 15;
  decap_bank: DecapStage[] = [
    { name: "Bulk_Polymer_47uF", count: 2, c_uf: 47, esr_mohm: 15, esl_nh: 1.8 },
    { name: "MLCC_0805_4.7uF", count: 4, c_uf: 4.7, esr_mohm: 5, esl_nh: 0.85 },
    { name: "MLCC_0402_100nF", count: 12, c_uf: 0.1, esr_mohm: 12, esl_nh: 0.45 },
    { name: "MLCC_0201_10nF", count: 8, c_uf: 0.01, esr_mohm: 25, esl_nh: 0.3 },
  ];

  getLayer(id: string): CopperLayer { return this.layers.find((l) => l.layer_id === id) ?? this.layers[0]; }

  allWorldPads(): Map<string, { comp: Component; pad: Pad; x: number; y: number }> {
    const out = new Map();
    for (const comp of Object.values(this.components))
      for (const pad of comp.pads) { const [x, y] = padWorldPos(comp, pad); out.set(pad.pad_id, { comp, pad, x, y }); }
    return out;
  }

  syncSchematicToPcb(): number {
    let updated = 0;
    for (const w of Object.values(this.sch_wires)) {
      this.nets.add(w.net_name);
      for (const [cid, pn] of [[w.from_comp_id, w.from_pin_number], [w.to_comp_id, w.to_pin_number]]) {
        const c = this.components[cid];
        if (!c) continue;
        for (const p of c.pads) if (p.pin_number === pn && p.net_name !== w.net_name) { p.net_name = w.net_name; updated++; }
      }
    }
    return updated;
  }

  toJSON(): string {
    return JSON.stringify({
      name: this.name, width_mm: this.width_mm, height_mm: this.height_mm, rules: this.rules, layers: this.layers,
      dielectrics: this.dielectrics, nets: [...this.nets].sort(), components: this.components, sch_wires: this.sch_wires,
      traces: this.traces, vias: this.vias, zones: this.zones, dc_sources: this.dc_sources, dc_sinks: this.dc_sinks,
      ambient_temp_c: this.ambient_temp_c, convection_h_w_m2k: this.convection_h_w_m2k, decap_bank: this.decap_bank,
    }, null, 2);
  }

  static fromJSON(raw: string): BoardProject {
    const d = JSON.parse(raw), b = new BoardProject();
    b.name = d.name ?? "Project";
    b.width_mm = Number(d.width_mm ?? 80);
    b.height_mm = Number(d.height_mm ?? 55);
    b.rules = { ...b.rules, ...(d.rules ?? {}) };
    b.layers = d.layers ?? [];
    b.dielectrics = d.dielectrics ?? [];
    b.nets = new Set(d.nets ?? []);
    b.components = d.components ?? {};
    b.sch_wires = d.sch_wires ?? {};
    b.traces = d.traces ?? {};
    b.vias = d.vias ?? {};
    b.zones = d.zones ?? {};
    b.dc_sources = d.dc_sources ?? {};
    b.dc_sinks = d.dc_sinks ?? {};
    b.ambient_temp_c = Number(d.ambient_temp_c ?? 25);
    b.convection_h_w_m2k = Number(d.convection_h_w_m2k ?? 15);
    if (d.decap_bank) b.decap_bank = d.decap_bank;
    return b;
  }
}

export const newId = (prefix: string): string => `${prefix}_${uid(6)}`;
