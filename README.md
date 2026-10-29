# PythonPCBDesign (PCB Master Suite)

A PCB design and multi-physics workbench: schematic capture, SPICE transient + FFT, 2D PCB CAD with an A* autorouter, 3D viewer, footprint wizard, stackup, DC IR-drop, coupled electrothermal solver, signal integrity, PDN analysis, Gerber/Excellon/BOM/CPL export, and KiCad/DXF import.

## Contents

- `PCB_Design_QT6.py` - original desktop application (Python 3.8+, PyQt6/PyQt5, NumPy, SciPy, pyqtgraph).
- `web/` - TypeScript port that runs in the browser (Vite). Deployed to GitHub Pages by `.github/workflows/pages.yml`.

## Web port status

Stage 1 (current): project scaffold, data model (same JSON field names as the Python `.pcbproj`), footprint generators and the demo board.
Next: solvers (SPICE, DRC, router, electrothermal, SI, PDN, CAM, KiCad/DXF import), then the interactive UI.

### Run locally

```bash
cd web
npm install
npm run dev
```

## License

GPL-3.0-or-later for the whole project, including the web port. See `LICENSE` and `THIRD_PARTY_NOTICES.md`. The desktop app depends on PyQt, which is GPL v3 (or commercial), so the licenses are kept consistent.

## Disclaimer

Results are simplified engineering estimates. Verify any design independently before fabrication.
