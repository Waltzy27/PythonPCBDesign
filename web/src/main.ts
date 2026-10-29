import { buildDefaultDemoBoard } from "./footprints";

const b = buildDefaultDemoBoard();
const lines = [
  `Board: ${b.name} (${b.width_mm} x ${b.height_mm} mm), ${b.layers.length} copper layers`,
  `Components: ${Object.values(b.components).map((c) => `${c.refdes} (${c.package_name})`).join(", ")}`,
  `Traces: ${Object.keys(b.traces).length}, Vias: ${Object.keys(b.vias).length}`,
  `Schematic wires synchronised to pads: ${b.syncSchematicToPcb()} changes`,
];
document.getElementById("app")!.innerHTML =
  `<h1>PCB Master Suite (Web) - stage 1 scaffold</h1><p>Solvers and interactive UI are being ported from the Python app.</p><pre>${lines.join("\n")}</pre>`;
